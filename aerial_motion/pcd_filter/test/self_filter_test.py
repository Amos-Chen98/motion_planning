#!/usr/bin/env python3
import math
import threading
import time
import unittest
import xml.etree.ElementTree as ET

import rospy
import rostest
import tf2_ros
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TransformStamped
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray


class SelfFilterIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node("pcd_filter_integration_test")
        cls.lock = threading.RLock()
        cls.outputs = {key: {} for key in ("filtered", "map", "direct", "late")}
        cls.diagnostics = {}
        cls.subscribers = []
        for key, topic in (("filtered", "/unit/filtered"), ("map", "/unit/occupied"),
                           ("direct", "/unit/unfiltered_occupied"), ("late", "/late/filtered")):
            def receive(message, key=key):
                with cls.lock:
                    stamp = message.markers[0].header.stamp if isinstance(message, MarkerArray) else message.header.stamp
                    cls.outputs[key][stamp.to_nsec()] = message
            cls.subscribers.append(rospy.Subscriber(topic, MarkerArray if key in ("map", "direct") else PointCloud2, receive))
        for key in ("unit", "late"):
            def diagnostic(message, key=key):
                with cls.lock:
                    if message.status:
                        cls.diagnostics[key] = {v.key: v.value for v in message.status[0].values}
            cls.subscribers.append(rospy.Subscriber(
                "/" + key + "/pcd_self_filter/diagnostics", DiagnosticArray, diagnostic))
        cls.raw = rospy.Publisher("/unit/raw", PointCloud2, queue_size=5)
        cls.direct = rospy.Publisher("/unit/unfiltered_input", PointCloud2, queue_size=5)
        cls.late = rospy.Publisher("/late/raw", PointCloud2, queue_size=5)
        cls.static = tf2_ros.StaticTransformBroadcaster()
        cls.broadcaster = tf2_ros.TransformBroadcaster()
        cls.pose = (0.0, 0.0, 0.0)
        cls.tip_enabled = True
        cls.probe_x = 0.0
        cls.stop = threading.Event()
        cls.static.sendTransform([
            cls.transform("world", "sensor", rospy.Time.now(), (0.3, -0.4, 0), math.pi / 2),
            cls.transform("world", "estimated_body", rospy.Time.now(), (4, 5, 6), math.pi / 3),
            cls.transform("world", "missing_mount_body", rospy.Time.now(), (0, 0, 0)),
            cls.transform("unit/base", "unit/mesh", rospy.Time.now(), (0, 1, 0))])

        def broadcast():
            while not cls.stop.is_set() and not rospy.is_shutdown():
                with cls.lock:
                    pose, include_tip, probe_x = cls.pose, cls.tip_enabled, cls.probe_x
                stamp = rospy.Time.now()
                cls.emit(stamp, *pose, include_tip=include_tip)
                cls.broadcaster.sendTransform(cls.transform(
                    "world", "mapping_probe", stamp, (probe_x, 0, 0)))
                cls.stop.wait(0.01)
        cls.thread = threading.Thread(target=broadcast, daemon=True)
        cls.thread.start()
        deadline = time.monotonic() + 10
        while not all(p.get_num_connections() for p in (cls.raw, cls.direct, cls.late)):
            if time.monotonic() >= deadline:
                raise RuntimeError("Nodes did not subscribe to the test topics")
            time.sleep(0.02)
        # Let the upstream TF watchdog discover the dynamic chain.
        time.sleep(1.2)

    @classmethod
    def tearDownClass(cls):
        cls.stop.set()
        cls.thread.join(timeout=2)

    def setUp(self):
        with self.lock:
            type(self).pose = (0.0, 0.0, 0.0)
            type(self).tip_enabled = True
        time.sleep(0.03)

    @staticmethod
    def transform(parent, child, stamp, xyz, yaw=0.0):
        message = TransformStamped()
        message.header.frame_id = parent
        message.child_frame_id = child
        message.header.stamp = stamp
        message.transform.translation.x, message.transform.translation.y, message.transform.translation.z = xyz
        message.transform.rotation.z = math.sin(yaw / 2)
        message.transform.rotation.w = math.cos(yaw / 2)
        return message

    @classmethod
    def emit(cls, stamp, x=0.0, yaw=0.0, tip_yaw=0.0, include_tip=True):
        transforms = [cls.transform("world", "unit/base", stamp, (x, 0, 1), yaw)]
        if include_tip:
            transforms.append(cls.transform("unit/base", "unit/tip", stamp, (0.8, 0, 0), tip_yaw))
        cls.broadcaster.sendTransform(transforms)

    @staticmethod
    def cloud(points, stamp=None, frame="world"):
        fields = [PointField(name, offset, kind, 1) for name, offset, kind in (
            ("x", 0, PointField.FLOAT32), ("y", 4, PointField.FLOAT32),
            ("z", 8, PointField.FLOAT32), ("intensity", 12, PointField.FLOAT32),
            ("label", 16, PointField.UINT32))]
        return point_cloud2.create_cloud(Header(stamp=stamp or rospy.Time.now(), frame_id=frame),
                                        fields, points)

    def wait(self, predicate, timeout=4.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with self.lock:
                if predicate():
                    return
            time.sleep(0.01)
        self.fail("Timed out; diagnostics=" + str(self.diagnostics))

    def result(self, message, key="filtered", publisher=None):
        (publisher or self.raw).publish(message)
        stamp = message.header.stamp.to_nsec()
        self.wait(lambda: stamp in self.outputs[key])
        return self.outputs[key][stamp]

    @staticmethod
    def rows(message):
        if isinstance(message, MarkerArray):
            return [(p.x - 10, p.y - 10, p.z - 2) for m in message.markers
                    if m.action == m.ADD for p in m.points]
        return list(point_cloud2.read_points(message, skip_nans=True))

    def assert_rejected(self, message, publisher=None, key="filtered", diagnostic="unit"):
        stamp = message.header.stamp.to_nsec()
        before = int(self.diagnostics.get(diagnostic, {}).get("failure_count", "0"))
        (publisher or self.raw).publish(message)
        self.wait(lambda: int(self.diagnostics.get(diagnostic, {}).get("failure_count", "0")) > before)
        self.assertNotIn(stamp, self.outputs[key])

    def test_01_geometry_history_and_extra_fields(self):
        stamp = rospy.Time.now() - rospy.Duration(0.08)
        self.emit(stamp)
        with self.lock:
            type(self).pose = (2.0, math.pi / 2, math.pi / 2)
        self.emit(rospy.Time.now(), *self.pose)
        time.sleep(0.03)
        world_points = [(0, 0, 1, 1, 1), (0.215, 0, 1, 2, 2), (0.5, 0, 1, 3, 3),
                        (1.0, 0, 1, 4, 4), (0.03, 1.03, 1.03, 5, 5),
                        (0.0, 1.05, 1.05, 6, 6),
                        (0.235, 0, 1, 71, 71), (2, 0, 1, 72, 72)]
        # Inverse of the static world<-sensor transform (including yaw).
        sensor_points = [(y + 0.4, 0.3 - x, z, intensity, label)
                         for x, y, z, intensity, label in world_points]
        message = self.cloud(sensor_points, stamp, "sensor")
        output = self.result(message)
        self.assertEqual(output.header.frame_id, "world")
        self.assertEqual(output.header.stamp, stamp)
        self.assertEqual(output.fields, message.fields)
        rows = self.rows(output)
        self.assertEqual([r[4] for r in rows], [71, 72])
        self.assertAlmostEqual(rows[0][0], 0.235, places=5)
        self.assertAlmostEqual(rows[1][3], 72.0)
        # Prefix only the private model; preserve each link's collision count.
        original = ET.fromstring(rospy.get_param("/unit/robot_description"))
        adapted = ET.fromstring(rospy.get_param("/unit/pcd_self_filter/filter_robot_description"))
        for link in original.findall("link"):
            self.assertFalse(link.get("name").startswith("unit/"))
            filtered_link = adapted.find("link[@name='unit/" + link.get("name") + "']")
            self.assertIsNotNone(filtered_link)
            self.assertEqual(len(filtered_link.findall("collision")), len(link.findall("collision")))

    def test_02_articulation_and_whole_body_rotation(self):
        with self.lock:
            type(self).pose = (3.0, math.pi / 2, math.pi / 2)
        time.sleep(0.04)
        stamp = rospy.Time.now()
        # Root yaw=90 degrees, joint yaw=90 degrees: tip center=(2.8,0.8,1).
        self.emit(stamp, *self.pose)
        output = self.result(self.cloud([(3, 0, 1, 1, 1), (2.8, 0.8, 1, 2, 2),
                                         (3.0, 1.0, 1, 99, 99)], stamp))
        self.assertEqual([r[4] for r in self.rows(output)], [99])

    def test_03_missing_and_delayed_tf(self):
        with self.lock:
            type(self).tip_enabled = False
        time.sleep(0.05)
        message = self.cloud([(4.2, 0, 1, 1, 1)])
        self.assert_rejected(message)
        self.assertGreater(int(self.diagnostics["unit"].get("failures/tf", "0")), 0)
        delayed = self.cloud([(4.3, 0, 1, 2, 2)])
        timer = threading.Timer(0.06, lambda: self.emit(delayed.header.stamp + rospy.Duration(0.01)))
        timer.start()
        try:
            self.assertEqual(self.result(delayed).width, 1)
            self.wait(lambda: self.diagnostics["unit"]["last_success_stamp"] ==
                      str(delayed.header.stamp.to_nsec()))
            self.assertNotEqual(self.diagnostics["unit"]["filtered_ratio"], "n/a")
        finally:
            timer.join()
            with self.lock:
                type(self).tip_enabled = True

    def test_04_invalid_empty_and_all_self_clouds(self):
        message = self.cloud([(4.4, 0, 1, 1, 1)])
        message.data = message.data[:-1]
        self.assert_rejected(message)
        unstamped = self.cloud([(4.4, 0, 1, 1, 1)])
        unstamped.header.stamp = rospy.Time(0)
        self.assert_rejected(unstamped)
        self.assertEqual(self.result(self.cloud([])).width, 0)
        self.assertEqual(self.result(self.cloud([(0, 0, 1, 1, 1)])).width, 0)

    def test_05_accumulation_contains_environment_without_body_trails(self):
        for x in (5.0, 6.0):
            with self.lock:
                type(self).pose = (x, 0.0, 0.0)
            time.sleep(0.03)
            message = self.cloud([(x + 0.03, 0.03, 1.03, 1, 1), (8.03, 0.03, 1.03, 2, 2)])
            occupied = self.result(message, "map")
            centers = self.rows(occupied)
            self.assertTrue(any(abs(p[0] - 8.05) < 1e-4 for p in centers))
            for body_x in (5.05, 6.05):
                self.assertFalse(any(abs(p[0] - body_x) < 1e-4 and abs(p[1] - 0.05) < 1e-4 for p in centers))

    def test_06_mapper_uses_cloud_timestamp_without_filter(self):
        old = rospy.Time.now() - rospy.Duration(0.08)
        with self.lock:
            type(self).probe_x = 3.0
        self.broadcaster.sendTransform([
            self.transform("world", "mapping_probe", old, (1, 0, 0)),
            self.transform("world", "mapping_probe", rospy.Time.now(), (3, 0, 0))])
        time.sleep(0.02)
        message = self.cloud([(0.03, 0.03, 1.03, 1, 1)], old, "mapping_probe")
        output = self.result(message, "direct", self.direct)
        centers = self.rows(output)
        self.assertEqual(len(centers), 1)
        self.assertAlmostEqual(centers[0][0], 1.05, places=4)

    def test_07_model_not_ready_then_recovers(self):
        self.assert_rejected(self.cloud([(4.5, 0, 1, 1, 1)]), self.late, "late", "late")
        rospy.set_param("/late/robot_description", rospy.get_param("/unit/robot_description"))
        self.wait(lambda: rospy.has_param("/late/pcd_self_filter/filter_robot_description"))
        time.sleep(1.2)
        message = self.cloud([(4.6, 0, 1, 1, 1)])
        self.assertEqual(self.result(message, "late", self.late).width, 1)

    def test_08_visual_only_links_are_ignored_without_tf(self):
        # No TF is ever published for unit/visual_only. Its URDF visual is at
        # world (0, -1, 1), and must neither remove points nor block the cloud.
        message = self.cloud([(0, -1, 1, 81, 81), (0, 0, 1, 1, 1),
                              (0.3, 0, 1, 82, 82)])
        output = self.result(message)
        # The final point is inside base's visual but outside its collisions.
        self.assertEqual([r[4] for r in self.rows(output)], [81, 82])
        self.assertEqual(output.header.stamp, message.header.stamp)
        self.assertEqual(output.fields, message.fields)

    def test_09_body_alias_uses_physical_mount_during_articulation(self):
        # The estimator's body TF is valid but disagrees with the physical
        # sensor. Incoming XYZ already use sensor-local coordinates.
        for x, yaw, tip_yaw in ((0, 0, 0), (3, math.pi / 2, math.pi / 2)):
            with self.lock:
                type(self).pose = (x, yaw, tip_yaw)
            time.sleep(0.04)
            stamp = rospy.Time.now()
            self.emit(stamp, x, yaw, tip_yaw)
            tip_x = x + 0.8 * math.cos(yaw) + 0.2 * math.cos(yaw + tip_yaw)
            tip_y = 0.8 * math.sin(yaw) + 0.2 * math.sin(yaw + tip_yaw)
            world_points = [(x, 0, 1, 1, 1), (tip_x, tip_y, 1, 2, 2),
                            (8.03, 0.03, 1.03, 99, 99)]
            sensor_points = [(wy + 0.4, 0.3 - wx, wz, intensity, label)
                             for wx, wy, wz, intensity, label in world_points]
            message = self.cloud(sensor_points, stamp, "/estimated_body")
            output = self.result(message)
            rows = self.rows(output)
            self.assertEqual([r[4] for r in rows], [99])
            for actual, expected in zip(rows[0][:3], world_points[-1][:3]):
                self.assertAlmostEqual(actual, expected, places=5)
            self.assertEqual(output.header.stamp, stamp)
            self.assertEqual(output.header.frame_id, "world")
            self.assertEqual(output.fields, message.fields)
            self.assertEqual(message.header.frame_id, "/estimated_body")
            self.wait(lambda: stamp.to_nsec() in self.outputs["map"])
            centers = self.rows(self.outputs["map"][stamp.to_nsec()])
            self.assertFalse(any(abs(p[0] - (x + 0.05)) < 1e-4 and
                                 abs(p[1] - 0.05) < 1e-4 for p in centers))

    def test_10_missing_alias_target_drops_cloud_despite_valid_body_tf(self):
        self.assert_rejected(self.cloud([(4, 0, 1, 1, 1)], frame="missing_mount_body"))


if __name__ == "__main__":
    rostest.rosrun("pcd_filter", "pcd_filter_integration", SelfFilterIntegration)
