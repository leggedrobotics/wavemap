#!/usr/bin/env python3
"""Save wavemap to disk together with the base pose at save time.

The pose is used by load_map_at_base.py in real to spawn the map at the
current robot position with the same relative geometry as in sim.

Usage:
    rosrun wavemap_ros save_map_with_pose.py
"""
import os
import yaml
import rospy
import tf2_ros
from wavemap_msgs.srv import FilePath

_PKG_DIR   = os.path.join(os.path.dirname(__file__), '..')
MAP_FILE   = os.path.realpath(os.path.join(_PKG_DIR, 'maps', 'alma_map.wvmp'))
POSE_FILE  = os.path.realpath(os.path.join(_PKG_DIR, 'maps', 'alma_map_origin.yaml'))
WORLD_FRAME = 'map'
BASE_FRAME  = 'base'


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

    rospy.wait_for_service('/wavemap/save_map', timeout=10.0)
    resp = rospy.ServiceProxy('/wavemap/save_map', FilePath)(MAP_FILE)
    if not resp.success:
        rospy.logerr('Map save service returned failure')
        return

    with open(POSE_FILE, 'w') as f:
        yaml.dump({'x': t.x, 'y': t.y, 'z': t.z,
                   'qx': r.x, 'qy': r.y, 'qz': r.z, 'qw': r.w}, f)

    rospy.loginfo(f'Map  -> {MAP_FILE}')
    rospy.loginfo(f'Pose -> {POSE_FILE}')
    rospy.loginfo(f'Base was at ({t.x:.3f}, {t.y:.3f}, {t.z:.3f}) in "{WORLD_FRAME}"')


if __name__ == '__main__':
    main()
