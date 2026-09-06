#!/usr/bin/env python3
"""Exercise noise filtering through the public cloud and occupied topics."""

import threading
import time
import unittest

import rospy
import rostest
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header


class NoiseFilterTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.outputs = {}
        self.sequence = 0
        self.subscribers = []
        for name in ("default", "latest", "disabled", "custom"):
            def receive(message, name=name):
                with self.lock:
                    self.outputs[name] = message
            self.subscribers.append(rospy.Subscriber(
                "/noise_" + name + "/voxelmap/occupied", PointCloud2, receive))
        self.publisher = rospy.Publisher("/noise_test/cloud", PointCloud2, queue_size=1)
        self.wait(lambda: self.publisher.get_num_connections() == 4 and
                  all(sub.get_num_connections() > 0 for sub in self.subscribers))

    def tearDown(self):
        self.publisher.unregister()
        for subscriber in self.subscribers:
            subscriber.unregister()

    def wait(self, predicate, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if predicate():
                return
            time.sleep(0.01)
        self.fail("Timed out waiting for mapper connections or output")

    def observe(self, points):
        self.sequence += 1
        stamp = rospy.Time(100 + self.sequence)
        message = point_cloud2.create_cloud_xyz32(Header(stamp=stamp, frame_id="world"), points)
        self.publisher.publish(message)

        def received():
            with self.lock:
                return len(self.outputs) == 4 and all(
                    output.header.stamp == stamp for output in self.outputs.values())

        self.wait(received)
        with self.lock:
            outputs = dict(self.outputs)
        centers = {}
        for name, output in outputs.items():
            self.assertEqual(output.header.frame_id, "world")
            self.assertEqual(output.header.stamp, stamp)
            centers[name] = {tuple(round(value, 5) for value in point)
                             for point in point_cloud2.read_points(
                                 output, field_names=("x", "y", "z"))}
        return centers

    def test_filter_defaults_overrides_and_map_lifetime(self):
        # The default rejects an isolated point and a pair. Lowering the neighbor
        # count admits the pair, while disabling filtering also admits the singleton.
        for _ in range(3):
            centers = self.observe([(1.55, 1.55, 1.05), (-0.95, 0.05, 1.05),
                                    (-0.93, 0.05, 1.05)])
            self.assertEqual(centers["default"], set())
            self.assertEqual(centers["latest"], set())
            self.assertEqual(centers["custom"], {(-0.95, 0.05, 1.05)})
            self.assertEqual(centers["disabled"],
                             {(-0.95, 0.05, 1.05), (1.55, 1.55, 1.05)})

        cluster = [(0.04, 0.05, 1.05), (0.13, 0.05, 1.05), (0.22, 0.05, 1.05)]
        expected = {(0.05, 0.05, 1.05), (0.15, 0.05, 1.05), (0.25, 0.05, 1.05)}
        centers = self.observe(cluster)
        for result in centers.values():
            self.assertEqual(result, expected)

        # A new singleton near the historical cluster cannot use it for support.
        centers = self.observe([(0.31, 0.05, 1.05)])
        self.assertEqual(centers["default"], expected)
        self.assertEqual(centers["latest"], set())
        self.assertEqual(centers["custom"], set())
        self.assertEqual(centers["disabled"], {(0.35, 0.05, 1.05)})

        # Only the larger radius plus the lower threshold accepts this wider pair.
        centers = self.observe([(1.05, 0.05, 1.05), (1.31, 0.05, 1.05)])
        self.assertEqual(centers["default"], expected)
        self.assertEqual(centers["latest"], set())
        self.assertEqual(centers["custom"], {(1.05, 0.05, 1.05), (1.35, 0.05, 1.05)})

        centers = self.observe([])
        self.assertEqual(centers["default"], expected)
        for name in ("latest", "disabled", "custom"):
            self.assertEqual(centers[name], set())


if __name__ == "__main__":
    rospy.init_node("voxel_mapping_noise_filter_test")
    rostest.rosrun("voxel_mapping", "voxel_mapping_noise_filter", NoiseFilterTest)
