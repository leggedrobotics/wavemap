#!/usr/bin/env python3
"""Save wavemap to disk together with the base pose at save time.

The pose is used by load_map_at_base.py in real to spawn the map at the
current robot position with the same relative geometry as in sim.

Usage:
    rosrun wavemap_ros save_map_with_pose.py
"""
import os
import yaml
import numpy as np
import rospy
import tf2_ros
from wavemap_msgs.srv import FilePath

_PKG_DIR   = os.path.join(os.path.dirname(__file__), '..')
MAP_FILE   = os.path.realpath(os.path.join(_PKG_DIR, 'maps', 'alma_map.wvmp'))
POSE_FILE  = os.path.realpath(os.path.join(_PKG_DIR, 'maps', 'alma_map_origin.yaml'))
WORLD_FRAME = 'map'
BASE_FRAME  = 'base'

# ANYmal foot frames used to anchor the map floor to the real contact surface.
# Must match FOOT_FRAMES in load_map_at_base.py.
FOOT_FRAMES = ['LF_FOOT', 'RF_FOOT', 'LH_FOOT', 'RH_FOOT']


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
    rospy.init_node('save_map_with_pose')

    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    rospy.sleep(1.0)

    try:
        T = buf.lookup_transform(WORLD_FRAME, BASE_FRAME,
                                 rospy.Time(0), rospy.Duration(5.0))
    except tf2_ros.TransformException as e:
        rospy.logerr(f'TF lookup {WORLD_FRAME}<-{BASE_FRAME} failed: {e}')
        return

    t, r = T.transform.translation, T.transform.rotation

    floor_z_in_wavemap = lookup_ground_z(buf, WORLD_FRAME, FOOT_FRAMES)
    if floor_z_in_wavemap is None:
        rospy.logwarn('Foot frames unavailable; defaulting floor_z_in_wavemap to 0.0 '
                      '(correct only if the world frame origin is at ground level)')
        floor_z_in_wavemap = 0.0
    else:
        rospy.loginfo(f'Foot-anchored ground z in {WORLD_FRAME}: {floor_z_in_wavemap:.3f} m')

    rospy.wait_for_service('/wavemap/save_map', timeout=10.0)
    resp = rospy.ServiceProxy('/wavemap/save_map', FilePath)(MAP_FILE)
    if not resp.success:
        rospy.logerr('Map save service returned failure')
        return

    with open(POSE_FILE, 'w') as f:
        yaml.dump({'x': t.x, 'y': t.y, 'z': t.z,
                   'qx': r.x, 'qy': r.y, 'qz': r.z, 'qw': r.w,
                   'floor_z_in_wavemap': floor_z_in_wavemap}, f)

    rospy.loginfo(f'Map  -> {MAP_FILE}')
    rospy.loginfo(f'Pose -> {POSE_FILE}')
    rospy.loginfo(f'Base was at ({t.x:.3f}, {t.y:.3f}, {t.z:.3f}) in "{WORLD_FRAME}"')


if __name__ == '__main__':
    main()
