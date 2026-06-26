#!/usr/bin/env python3
"""Load a saved wavemap at the robot's current base pose.

Reads alma_map_origin.yaml (written by save_map_with_pose.py in sim) and
publishes a static TF 'wavemap_origin' such that the loaded map has the same
geometric relationship to the robot as it did at save time.

The offset is projected onto the horizontal plane (roll/pitch/height agnostic):
only x, y, and yaw are used; z, roll, and pitch are zeroed out.

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

MAP_FILE   = os.path.expanduser('~/wavemap/alma_map.wvmp')
POSE_FILE  = os.path.expanduser('~/wavemap/alma_map_origin.yaml')
WORLD_FRAME = 'odom'          # real robot's world frame
BASE_FRAME  = 'base'
WAVEMAP_FRAME = 'wavemap_origin'  # must match world_frame in esdf_only yaml


def pose_to_matrix(t, q):
    """Build a 4x4 homogeneous transform from translation and quaternion."""
    M = quaternion_matrix(q)   # already 4x4
    M[:3, 3] = t
    return M


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

    # T_odom_wavemap = T_odom_base_real * T_map_base_sim^{-1}
    # This makes the robot-to-map relationship identical to save time.
    M_real = pose_to_matrix(
        [tn.x, tn.y, tn.z],
        [rn.x, rn.y, rn.z, rn.w])
    M_sim = pose_to_matrix(
        [o['x'], o['y'], o['z']],
        [o['qx'], o['qy'], o['qz'], o['qw']])
    M_result = M_real.dot(np.linalg.inv(M_sim))

    # Project to horizontal: keep x, y only; extract yaw only.
    tx = M_result[0, 3]
    ty = M_result[1, 3]
    q_result = quaternion_from_matrix(M_result)
    _, _, yaw = euler_from_quaternion(q_result)
    q_flat = quaternion_from_euler(0.0, 0.0, yaw)

    msg = geometry_msgs.msg.TransformStamped()
    msg.header.stamp    = rospy.Time.now()
    msg.header.frame_id = WORLD_FRAME
    msg.child_frame_id  = WAVEMAP_FRAME
    msg.transform.translation.x = tx
    msg.transform.translation.y = ty
    msg.transform.translation.z = 0.0
    msg.transform.rotation.x = q_flat[0]
    msg.transform.rotation.y = q_flat[1]
    msg.transform.rotation.z = q_flat[2]
    msg.transform.rotation.w = q_flat[3]

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    broadcaster.sendTransform(msg)
    rospy.loginfo(f'Published TF: {WORLD_FRAME} -> {WAVEMAP_FRAME} '
                  f'at ({tx:.3f}, {ty:.3f}, 0.0), yaw={np.degrees(yaw):.1f} deg')
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
