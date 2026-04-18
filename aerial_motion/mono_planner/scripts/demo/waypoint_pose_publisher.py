#!/usr/bin/env python3

import math
import os

import rospy
import tf.transformations as tf_trans
import yaml
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

try:
    import rospkg
except ImportError:
    rospkg = None


class WaypointPosePublisher:
    RING_RADIUS_METERS = 0.4
    RING_THICKNESS_METERS = 0.02
    RING_SEGMENTS = 72

    def __init__(self):
        rospy.init_node("waypoint_pose_publisher", anonymous=False)

        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 60.0))
        if self.publish_rate_hz <= 0.0:
            raise ValueError("~publish_rate_hz must be greater than 0.")

        self.frame_id = rospy.get_param("~frame_id", "world")
        self.config_file = rospy.get_param("~config_file", self._get_default_config_path())

        config = self._load_config(self.config_file)
        if isinstance(config, dict):
            self.frame_id = config.get("frame_id", self.frame_id)
            waypoint_specs = config.get("waypoints")
        elif isinstance(config, list):
            waypoint_specs = config
        else:
            raise ValueError("Config file must contain a waypoint list or a dictionary with a 'waypoints' field.")

        self.pose_messages = self._build_pose_messages(waypoint_specs)
        self.publishers = [
            rospy.Publisher(f"/waypoint/pose_{index}", PoseStamped, queue_size=1, latch=True)
            for index in range(len(self.pose_messages))
        ]
        self.ring_marker_publisher = rospy.Publisher("/waypoint/ring_markers", MarkerArray, queue_size=1, latch=True)
        self._publish_ring_markers()

        rospy.loginfo(
            "Loaded %d waypoints from %s, publishing poses at %.1f Hz with frame_id '%s' and ring markers on /waypoint/ring_markers.",
            len(self.pose_messages),
            self.config_file,
            self.publish_rate_hz,
            self.frame_id,
        )

    def _get_default_config_path(self):
        if rospkg is not None:
            try:
                package_path = rospkg.RosPack().get_path("mono_planner")
                return os.path.join(package_path, "config", "demo_waypoints.yaml")
            except rospkg.ResourceNotFound:
                pass

        script_dir = os.path.abspath(os.path.dirname(__file__))
        return os.path.normpath(os.path.join(script_dir, "..", "..", "config", "demo_waypoints.yaml"))

    @staticmethod
    def _load_config(config_file):
        if not os.path.isfile(config_file):
            raise FileNotFoundError("Waypoint config file not found: {}".format(config_file))

        with open(config_file, "r", encoding="utf-8") as file_handle:
            config = yaml.safe_load(file_handle)

        if config is None:
            raise ValueError("Waypoint config file is empty: {}".format(config_file))

        return config

    def _build_pose_messages(self, waypoint_specs):
        if not isinstance(waypoint_specs, list) or not waypoint_specs:
            raise ValueError("'waypoints' must be a non-empty list.")

        pose_messages = []
        for index, waypoint in enumerate(waypoint_specs):
            if not isinstance(waypoint, (list, tuple)) or len(waypoint) != 6:
                raise ValueError(
                    "Waypoint {} must be a list in the form [x, y, z, roll, pitch, yaw].".format(index)
                )

            x, y, z, roll, pitch, yaw = [float(value) for value in waypoint]
            quaternion = tf_trans.quaternion_from_euler(roll, pitch, yaw)

            pose_msg = PoseStamped()
            pose_msg.header.frame_id = self.frame_id
            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            pose_msg.pose.position.z = z
            pose_msg.pose.orientation.x = quaternion[0]
            pose_msg.pose.orientation.y = quaternion[1]
            pose_msg.pose.orientation.z = quaternion[2]
            pose_msg.pose.orientation.w = quaternion[3]
            pose_messages.append(pose_msg)

        return pose_messages

    def _build_ring_markers(self, stamp):
        marker_array = MarkerArray()

        for index, pose_msg in enumerate(self.pose_messages):
            marker = Marker()
            marker.header.frame_id = pose_msg.header.frame_id
            marker.header.stamp = stamp
            marker.ns = "waypoint_rings"
            marker.id = index
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.lifetime = rospy.Duration(0.0)
            marker.pose.position.x = pose_msg.pose.position.x
            marker.pose.position.y = pose_msg.pose.position.y
            marker.pose.position.z = pose_msg.pose.position.z
            marker.pose.orientation.x = pose_msg.pose.orientation.x
            marker.pose.orientation.y = pose_msg.pose.orientation.y
            marker.pose.orientation.z = pose_msg.pose.orientation.z
            marker.pose.orientation.w = pose_msg.pose.orientation.w
            marker.scale.x = self.RING_THICKNESS_METERS
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            marker.points = self._build_ring_points()
            marker_array.markers.append(marker)

        return marker_array

    @classmethod
    def _build_ring_points(cls):
        points = []
        for step in range(cls.RING_SEGMENTS + 1):
            angle = 2.0 * math.pi * float(step) / float(cls.RING_SEGMENTS)
            points.append(Point(x=0.0, y=cls.RING_RADIUS_METERS * math.cos(angle), z=cls.RING_RADIUS_METERS * math.sin(angle)))
        return points

    def _publish_ring_markers(self):
        delete_all_marker = Marker()
        delete_all_marker.action = Marker.DELETEALL
        self.ring_marker_publisher.publish(MarkerArray(markers=[delete_all_marker]))
        self.ring_marker_publisher.publish(self._build_ring_markers(rospy.Time.now()))

    def run(self):
        rate = rospy.Rate(self.publish_rate_hz)

        while not rospy.is_shutdown():
            stamp = rospy.Time.now()
            for publisher, pose_msg in zip(self.publishers, self.pose_messages):
                pose_msg.header.stamp = stamp
                publisher.publish(pose_msg)
            rate.sleep()


if __name__ == "__main__":
    try:
        WaypointPosePublisher().run()
    except Exception as exc:
        rospy.logerr("Failed to start waypoint pose publisher: %s", exc)
        raise
