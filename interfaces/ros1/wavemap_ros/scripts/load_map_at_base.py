#!/usr/bin/env python3
"""Load a saved wavemap at the robot's current base pose.

Reads alma_map_origin.yaml (written by save_map_with_pose.py in sim) and
publishes a static TF 'wavemap_origin' (child of 'map') such that the loaded
map has the same geometric relationship to the robot as it did at save time.

Publishing as a child of 'map' avoids a TF cycle with the sim/SLAM stack
that already owns the map→odom edge.

Horizontal alignment (x, y, yaw) is derived from the base frame as before.
Vertical alignment (z) is anchored to the real foot contact height so that
map voxels land on the actual floor surface, not a sim-height proxy.
Roll and pitch are zeroed.

Optional YAML key 'floor_z_in_wavemap' (default 0.0): the z of the ground
surface in wavemap_origin frame from the sim run. Non-zero only if wavemap_origin
was mounted above or below ground level in sim.

Requires wavemap_anymal_esdf_only.yaml to have world_frame: "wavemap_origin".

Usage:
    rosrun wavemap_ros load_map_at_base.py
"""
import os
import yaml
import numpy as np
import rospy
import tf2_ros
import geometry_msgs.msg
from tf.transformations import (quaternion_matrix, quaternion_from_matrix,
                                euler_from_quaternion, quaternion_from_euler)
from wavemap_msgs.srv import FilePath

_PKG_DIR   = os.path.join(os.path.dirname(__file__), '..')
MAP_FILE   = os.path.realpath(os.path.join(_PKG_DIR, 'maps', 'alma_map.wvmp'))
POSE_FILE  = os.path.realpath(os.path.join(_PKG_DIR, 'maps', 'alma_map_origin.yaml'))
WORLD_FRAME   = 'map'
BASE_FRAME    = 'base'
WAVEMAP_FRAME = 'wavemap_origin'  # must match world_frame in esdf_only yaml

# ANYmal foot frames used to anchor the map floor to the real contact surface.
FOOT_FRAMES = ['LF_FOOT', 'RF_FOOT', 'LH_FOOT', 'RH_FOOT']


def pose_to_matrix(t, q):
    M = quaternion_matrix(q)
    M[:3, 3] = t
    return M


def lookup_ground_z(buf, world_frame, foot_frames):
    """Return mean foot-contact z in world_frame, or None if unavailable."""
    zs = []
    for ff in foot_frames:
        try:
            ft = buf.lookup_transform(world_frame, ff,
                                      rospy.Time(0), rospy.Duration(1.0))
            zs.append(ft.transform.translation.z)
        except tf2_ros.TransformException:
            pass
    return float(np.mean(zs)) if zs else None


def main():
    rospy.init_node('load_map_at_base')

    if not os.path.exists(POSE_FILE):
        rospy.logerr(f'Pose file not found: {POSE_FILE}')
        return
    if not os.path.exists(MAP_FILE):
        rospy.logerr(f'Map file not found: {MAP_FILE}')
        return

    with open(POSE_FILE) as f:
        o = yaml.safe_load(f)

    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    rospy.sleep(1.0)

    try:
        T = buf.lookup_transform(WORLD_FRAME, BASE_FRAME,
                                 rospy.Time(0), rospy.Duration(5.0))
    except tf2_ros.TransformException as e:
        rospy.logerr(f'TF lookup {WORLD_FRAME}<-{BASE_FRAME} failed: {e}')
        return

    tn = T.transform.translation
    rn = T.transform.rotation

    # Horizontal: T_map_wavemap_origin = T_map_base_now * T_map_base_saved^{-1}
    M_real = pose_to_matrix(
        [tn.x, tn.y, tn.z],
        [rn.x, rn.y, rn.z, rn.w])
    M_sim = pose_to_matrix(
        [o['x'], o['y'], o['z']],
        [o['qx'], o['qy'], o['qz'], o['qw']])
    M_result = M_real.dot(np.linalg.inv(M_sim))

    tx = M_result[0, 3]
    ty = M_result[1, 3]
    q_result = quaternion_from_matrix(M_result)
    _, _, yaw = euler_from_quaternion(q_result)
    q_flat = quaternion_from_euler(0.0, 0.0, yaw)

    # Vertical: anchor map floor to real foot-contact height.
    # floor_z_in_wavemap is where the ground sits in wavemap_origin coords at
    # save time (0.0 when wavemap_origin was at ground level in sim — the default).
    floor_z_in_wavemap = float(o.get('floor_z_in_wavemap', 0.0))

    ground_z = lookup_ground_z(buf, WORLD_FRAME, FOOT_FRAMES)
    if ground_z is None:
        rospy.logwarn('Foot frames unavailable; falling back to z=0 for ground alignment')
        ground_z = 0.0
    else:
        rospy.loginfo(f'Foot-anchored ground z in {WORLD_FRAME}: {ground_z:.3f} m '
                      f'({len([f for f in FOOT_FRAMES])} feet queried)')

    tz = ground_z - floor_z_in_wavemap

    msg = geometry_msgs.msg.TransformStamped()
    msg.header.stamp    = rospy.Time.now()
    msg.header.frame_id = WORLD_FRAME
    msg.child_frame_id  = WAVEMAP_FRAME
    msg.transform.translation.x = tx
    msg.transform.translation.y = ty
    msg.transform.translation.z = tz
    msg.transform.rotation.x = q_flat[0]
    msg.transform.rotation.y = q_flat[1]
    msg.transform.rotation.z = q_flat[2]
    msg.transform.rotation.w = q_flat[3]

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    broadcaster.sendTransform(msg)
    rospy.loginfo(f'Published static TF: {WORLD_FRAME} -> {WAVEMAP_FRAME} '
                  f'at ({tx:.3f}, {ty:.3f}, {tz:.3f}), yaw={np.degrees(yaw):.1f} deg')
    rospy.sleep(0.5)  # let TF propagate before loading

    rospy.wait_for_service('/wavemap/load_map', timeout=10.0)
    resp = rospy.ServiceProxy('/wavemap/load_map', FilePath)(MAP_FILE)
    if resp.success:
        rospy.loginfo('Map loaded successfully — ESDF will publish on /wavemap/esdf')
    else:
        rospy.logerr('Map load service returned failure')
        return

    rospy.spin()  # keep static TF broadcaster alive


if __name__ == '__main__':
    main()
