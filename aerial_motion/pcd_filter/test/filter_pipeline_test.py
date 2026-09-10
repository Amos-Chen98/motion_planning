#!/usr/bin/env python3
import math
import threading
import time
import unittest

import rospy
import rostest
import tf2_ros
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TransformStamped
from octomap_msgs.msg import Octomap
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


class FilterPipelineTest(unittest.TestCase):
    @staticmethod
    def wait(predicate, timeout=6):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            value = predicate()
            if value:
                return value
            time.sleep(.01)
        raise AssertionError('Timed out waiting for filter pipeline')

    @staticmethod
    def tf(child, xyz, stamp=None, yaw=0):
        result = TransformStamped()
        result.header.frame_id = 'world'
        result.header.stamp = stamp or rospy.Time.now()
        result.child_frame_id = child
        result.transform.translation.x, result.transform.translation.y, result.transform.translation.z = xyz
        result.transform.rotation.z = math.sin(yaw / 2)
        result.transform.rotation.w = math.cos(yaw / 2)
        return result

    @classmethod
    def setUpClass(cls):
        rospy.init_node('filter_pipeline_test')
        cls.outputs = {key: [] for key in ('two', 'six', 'missing', 'map')}
        cls.diagnostics = []
        cls.subs = []
        for key, topic in [('two', '/pipe/cloud2'), ('six', '/pipe/cloud6'),
                           ('missing', '/pipe/missing'), ('map', '/pipe/octomap/full')]:
            cls.subs.append(rospy.Subscriber(topic, Octomap if key == 'map' else PointCloud2,
                                            lambda m, key=key: cls.outputs[key].append(m)))
        cls.subs.append(rospy.Subscriber('/pipe/missing_output/pcd_self_filter/diagnostics', DiagnosticArray,
                                        cls.diagnostics.append))
        cls.publisher = rospy.Publisher('/pipe/raw', PointCloud2, queue_size=5)
        cls.static = tf2_ros.StaticTransformBroadcaster()
        cls.dynamic = tf2_ros.TransformBroadcaster()
        cls.static.sendTransform([cls.tf('pipe/base', (0, 0, 10)),
                                  cls.tf('pipe/tip', (.8, 0, 10)),
                                  cls.tf('pipe/mesh', (0, 1, 10)),
                                  cls.tf('pipe/imu', (1, 2, .4), yaw=math.pi / 2)])
        cls.wait(lambda: cls.publisher.get_num_connections() == 3, 15)
        # Wait for every filter's TF buffer and downstream subscriber to be ready.
        # A raw-input connection alone does not guarantee TF has propagated.
        def pipeline_ready():
            fields = [PointField(name=n, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                      for i, n in enumerate(('x', 'y', 'z', 'intensity'))]
            cls.publisher.publish(point_cloud2.create_cloud(
                Header(stamp=rospy.Time.now(), frame_id='world'), fields, []))
            return (all(cls.outputs[key] for key in ('two', 'six', 'map')) and
                    any('nonexistent_output' in status.message
                        for message in cls.diagnostics for status in message.status))
        cls.wait(pipeline_ready, 15)

    def cloud(self, points, frame='world', stamp=None):
        fields = [PointField(name=n, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(('x', 'y', 'z', 'intensity'))]
        return point_cloud2.create_cloud(Header(stamp=stamp or rospy.Time.now(), frame_id=frame), fields, points)

    def scan(self, points, frame='world', stamp=None):
        message = self.cloud(points, frame, stamp)
        self.publisher.publish(message)
        results = {}
        for key in ('two', 'six', 'map'):
            results[key] = self.wait(lambda: next((m for m in reversed(self.outputs[key])
                                      if m.header.stamp == message.header.stamp), None))
        return message, results

    def test_neighbor_counts_and_extra_fields(self):
        for count in (2, 3, 6, 7):
            points = [(2 + i * .01, 2., 1., float(i)) for i in range(count)]
            original, result = self.scan(points)
            self.assertEqual(result['two'].width, count if count >= 3 else 0)
            self.assertEqual(result['six'].width, count if count >= 7 else 0)
            self.assertEqual(result['two'].fields, original.fields)
            self.assertEqual(result['two'].header.frame_id, 'pipe/lidar_origin')
            self.assertEqual(result['two'].header.stamp, original.header.stamp)
            if count >= 3:
                rows = list(point_cloud2.read_points(result['two']))
                # world<-IMU is yaw=pi/2; subtract the IMU-local MID360 translation.
                self.assertAlmostEqual(rows[0][0], .011, places=5)
                self.assertAlmostEqual(rows[0][1], -1 + .02329, places=5)
                self.assertAlmostEqual(rows[0][2], .6 - .04412, places=5)
                self.assertEqual([p[3] for p in rows], list(range(count)))

    def test_body_and_noise_removed_points_produce_no_rays(self):
        _, initial = self.scan([(2 + i * .01, 2., 1., float(i)) for i in range(3)])
        for points in ([(4., 2., 1., 99.)], [], [(0., 0., 10., 1.)] * 7):
            _, result = self.scan(points)
            self.assertEqual(result['two'].width, 0)
            self.assertEqual(result['map'].data, initial['map'].data)

    def test_fast_lio_alias_matches_world_cloud(self):
        points = [(2 + i * .01, 2., 1., float(i)) for i in range(3)]
        _, world = self.scan(points)
        imu = [(y - 2, 1 - x, z - .4, intensity) for x, y, z, intensity in points]
        _, alias = self.scan(imu, frame='body')
        first = list(point_cloud2.read_points(world['two']))
        second = list(point_cloud2.read_points(alias['two']))
        for a, b in zip(first, second):
            for x, y in zip(a, b):
                self.assertAlmostEqual(x, y, places=5)

    def test_historical_input_transform(self):
        old = rospy.Time.now() - rospy.Duration(.2)
        self.dynamic.sendTransform([self.tf('moving_input', (2, 2, 1), old),
                                    self.tf('moving_input', (4, 2, 1), rospy.Time.now())])
        time.sleep(.1)
        _, result = self.scan([(i * .01, 0., 0., float(i)) for i in range(3)],
                              frame='moving_input', stamp=old)
        point = next(point_cloud2.read_points(result['two']))
        self.assertAlmostEqual(point[1], -1 + .02329, places=5)

    def test_missing_output_tf_and_zero_stamp_are_rejected(self):
        message, _ = self.scan([(2 + i * .01, 2., 1., 0.) for i in range(3)])
        time.sleep(.3)
        self.assertFalse(self.outputs['missing'])
        self.assertTrue(any('nonexistent_output' in status.message for m in self.diagnostics
                            for status in m.status))
        before = {key: len(value) for key, value in self.outputs.items()}
        message.header.stamp = rospy.Time(0)
        self.publisher.publish(message)
        time.sleep(.3)
        self.assertEqual(before, {key: len(value) for key, value in self.outputs.items()})


if __name__ == '__main__':
    rostest.rosrun('pcd_filter', 'filter_pipeline', FilterPipelineTest)
