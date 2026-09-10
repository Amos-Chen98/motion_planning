#!/usr/bin/env python3
"""Black-box checks of the installed server and the empty-map adapter."""
import math
import struct
import threading
import time
import unittest

import rospy
import rostest
import tf2_ros
from geometry_msgs.msg import TransformStamped
from octomap_msgs.msg import Octomap
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker, MarkerArray


def leaves(message):
    data = bytes(v & 255 for v in message.data)
    result = []
    cursor = 0

    def visit(center, size, depth):
        nonlocal cursor
        value, mask = struct.unpack_from('=fB', data, cursor)
        cursor += 5
        if not mask:
            result.append((center, size, depth, value))
        for child in range(8):
            if mask & (1 << child):
                visit(tuple(center[a] + (size / 4 if child & (1 << a) else -size / 4)
                            for a in range(3)), size / 2, depth + 1)
    if data:
        visit((0., 0., 0.), message.resolution * 65536, 0)
    if cursor != len(data):
        raise AssertionError('Incomplete full-map decode')
    return result


class OfficialServerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node('official_server_test')
        cls.lock = threading.Lock()
        cls.maps = {key: [] for key in ('mapper_test', 'limited_test')}
        cls.markers = {key: [] for key in cls.maps}
        cls.subs = []
        cls.pubs = {}
        cls.resets = {}
        for key in cls.maps:
            def receive(message, key=key):
                with cls.lock:
                    cls.maps[key].append(message)
            def markers(message, key=key):
                with cls.lock:
                    cls.markers[key].append(message)
            cls.subs += [rospy.Subscriber('/' + key + '/octomap/full', Octomap, receive),
                         rospy.Subscriber('/' + key + '/octomap/occupied_cells', MarkerArray, markers)]
            cls.pubs[key] = rospy.Publisher('/' + key + '/cloud', PointCloud2, queue_size=5)
            rospy.wait_for_service('/' + key + '/octomap/reset', timeout=15)
            cls.resets[key] = rospy.ServiceProxy('/' + key + '/octomap/reset', Empty)
        cls.static = tf2_ros.StaticTransformBroadcaster()
        cls.dynamic = tf2_ros.TransformBroadcaster()
        tf = TransformStamped()
        tf.header.frame_id = 'world'
        tf.child_frame_id = 'probe_sensor'
        tf.header.stamp = rospy.Time.now()
        tf.transform.translation.x = .273
        tf.transform.translation.y = -.767
        tf.transform.translation.z = .287
        tf.transform.rotation.w = 1
        cls.static.sendTransform(tf)
        cls.wait(lambda: all(p.get_num_connections() for p in cls.pubs.values()) and
                 all(cls.maps.values()), 15)
        time.sleep(.3)

    @staticmethod
    def wait(predicate, timeout=5):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            value = predicate()
            if value:
                return value
            time.sleep(.01)
        raise AssertionError('Timed out waiting for server output')

    def setUp(self):
        for key in self.maps:
            count = len(self.maps[key])
            self.resets[key]()
            self.wait(lambda: len(self.maps[key]) > count and not self.maps[key][-1].data)

    def scan(self, points, key='mapper_test', frame='probe_sensor', stamp=None):
        stamp = stamp or rospy.Time.now()
        self.pubs[key].publish(point_cloud2.create_cloud_xyz32(Header(stamp=stamp, frame_id=frame), points))
        return self.wait(lambda: next((m for m in reversed(self.maps[key]) if m.header.stamp == stamp), None))

    def probability(self, message, point):
        for center, size, _, value in leaves(message):
            if all(center[a] - size / 2 - 1e-8 <= point[a] < center[a] + size / 2 - 1e-8
                   for a in range(3)):
                return 1 / (1 + math.exp(-value))
        return None

    def test_hit_fusion_and_duplicate_points(self):
        first = self.scan([(1., 0., 0.)] * 20)
        self.assertFalse(first.binary)
        self.assertEqual(first.id, 'OcTree')
        self.assertAlmostEqual(self.probability(first, (1.25, .25, .25)), .7, places=6)
        second = self.scan([(1., 0., 0.)])
        self.assertAlmostEqual(self.probability(second, (1.25, .25, .25)), .49 / .58, places=6)
        self.assertLess(self.probability(second, (.65, .25, .25)), .5)
        self.assertIsNone(self.probability(second, (3.25, 3.25, 3.25)))

    def test_occupied_endpoint_has_priority(self):
        message = self.scan([(1., 0., 0.), (2., 0., 0.)])
        self.assertAlmostEqual(self.probability(message, (1.25, .25, .25)), .7, places=6)
        self.assertAlmostEqual(self.probability(message, (2.25, .25, .25)), .7, places=6)

    def test_outside_endpoint_clears_inside_and_is_mapped(self):
        self.scan([(1., 0., 0.)])
        for _ in range(3):
            message = self.scan([(6., 0., 0.)])
        self.assertLess(self.probability(message, (1.25, .25, .25)), .5)
        self.assertGreater(self.probability(message, (6.25, .25, .25)), .5)

    def test_empty_cloud_preserves_history(self):
        before = self.scan([(1., 0., 0.)])
        after = self.scan([])
        self.assertEqual(before.data, after.data)

    def test_max_range_has_no_occupied_endpoint(self):
        message = self.scan([(6., 0., 0.)], key='limited_test')
        self.assertTrue(leaves(message))
        self.assertTrue(all(value < 0 for _, _, _, value in leaves(message)))
        self.assertLess(self.probability(message, (2.25, .25, .25)), .5)
        self.assertIsNone(self.probability(message, (6.25, .25, .25)))

    def test_reset_clears_latched_full_map_and_markers(self):
        self.scan([(1., 0., 0.)])
        count = len(self.maps['mapper_test'])
        self.resets['mapper_test']()
        self.wait(lambda: len(self.maps['mapper_test']) > count and not self.maps['mapper_test'][-1].data)
        self.wait(lambda: self.markers['mapper_test'] and
                  all(m.action == Marker.DELETE for m in self.markers['mapper_test'][-1].markers))
        late = []
        sub = rospy.Subscriber('/mapper_test/octomap/full', Octomap, late.append)
        self.wait(lambda: late)
        self.assertFalse(late[-1].data)
        sub.unregister()

    def test_marker_geometry_and_stamp_match_full_tree(self):
        # Eight adjacent saturated cells can prune to a larger occupied leaf.
        points = [(x - .25, y - .25, z - .25)
                  for x in (1.25, 1.35) for y in (.45, .55) for z in (.45, .55)]
        for _ in range(12):
            message = self.scan(points)
        array = self.wait(lambda: next((a for a in reversed(self.markers['mapper_test'])
                          if a.markers and a.markers[0].header.stamp == message.header.stamp), None))
        expected = {(round(c[0], 5), round(c[1], 5), round(c[2], 5), round(size, 5))
                    for c, size, _, value in leaves(message) if value >= 0}
        actual = set()
        for marker in array.markers:
            self.assertEqual(marker.header.frame_id, message.header.frame_id)
            if marker.action == Marker.ADD:
                self.assertEqual(marker.type, Marker.CUBE_LIST)
                actual.update((round(p.x, 5), round(p.y, 5), round(p.z, 5), round(marker.scale.x, 5))
                              for p in marker.points)
        self.assertEqual(actual, expected)
        self.assertTrue(any(size > .1 for _, _, _, size in expected))

    def test_historical_sensor_pose(self):
        old = rospy.Time.now() - rospy.Duration(.3)
        transforms = []
        for stamp, x in ((old, .773), (rospy.Time.now(), 2.773)):
            tf = TransformStamped()
            tf.header.frame_id = 'world'
            tf.child_frame_id = 'moving_sensor'
            tf.header.stamp = stamp
            tf.transform.translation.x = x
            tf.transform.translation.y = -.767
            tf.transform.translation.z = .287
            tf.transform.rotation.w = 1
            transforms.append(tf)
        self.dynamic.sendTransform(transforms)
        time.sleep(.15)
        message = self.scan([(1., 0., 0.)], frame='moving_sensor', stamp=old)
        self.assertGreater(self.probability(message, (1.75, .25, .25)), .5)
        self.assertIsNone(self.probability(message, (3.75, .25, .25)))


if __name__ == '__main__':
    rostest.rosrun('octomap_mapper', 'official_server', OfficialServerTest)
