#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker


class RootStraightTrajectoryPublisher:
    NODE_NAME = "root_straight_trajectory"
    DEFAULT_TOPIC = "root/target_pose"
    DEFAULT_START_POSE_TOPIC = "root/tail_pose"
    DEFAULT_MARKER_TOPIC = "root/trajectory_marker"
    DEFAULT_FRAME_ID = "world"
    DEFAULT_PUBLISH_RATE_HZ = 40.0
    DEFAULT_LINEAR_SPEED = 0.10
    DEFAULT_MARKER_LINE_WIDTH_M = 0.03
    DEFAULT_MARKER_SAMPLE_INTERVAL_M = 0.05
    DEFAULT_RUN_DURATION_SEC = 0.0
    YAW = math.pi
    MARKER_NS = "root_target_trajectory"
    MARKER_ID = 0

    def __init__(self):
        rospy.init_node(self.NODE_NAME, anonymous=False)

        self.topic = rospy.get_param("~topic", self.DEFAULT_TOPIC)
        self.start_pose_topic = rospy.get_param("~start_pose_topic", self.DEFAULT_START_POSE_TOPIC)
        self.marker_topic = rospy.get_param("~marker_topic", self.DEFAULT_MARKER_TOPIC)
        self.frame_id = rospy.get_param("~frame_id", self.DEFAULT_FRAME_ID)
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", self.DEFAULT_PUBLISH_RATE_HZ))
        self.linear_speed = float(rospy.get_param("~linear_speed", self.DEFAULT_LINEAR_SPEED))
        self.marker_line_width_m = float(rospy.get_param("~marker_line_width_m", self.DEFAULT_MARKER_LINE_WIDTH_M))
        self.marker_sample_interval_m = float(
            rospy.get_param("~marker_sample_interval_m", self.DEFAULT_MARKER_SAMPLE_INTERVAL_M)
        )
        self.run_duration_sec = float(rospy.get_param("~run_duration_sec", self.DEFAULT_RUN_DURATION_SEC))

        if not math.isfinite(self.publish_rate_hz) or self.publish_rate_hz <= 0.0:
            raise ValueError("~publish_rate_hz must be a finite value greater than 0.")
        if not math.isfinite(self.linear_speed) or self.linear_speed <= 0.0:
            raise ValueError("~linear_speed must be a finite value greater than 0.")
        if not math.isfinite(self.marker_line_width_m) or self.marker_line_width_m <= 0.0:
            raise ValueError("~marker_line_width_m must be a finite value greater than 0.")
        if not math.isfinite(self.marker_sample_interval_m) or self.marker_sample_interval_m <= 0.0:
            raise ValueError("~marker_sample_interval_m must be a finite value greater than 0.")
        if not math.isfinite(self.run_duration_sec) or self.run_duration_sec < 0.0:
            raise ValueError("~run_duration_sec must be a finite non-negative value.")

        self.publisher = rospy.Publisher(self.topic, PoseStamped, queue_size=1)
        self.marker_publisher = rospy.Publisher(self.marker_topic, Marker, queue_size=1, latch=True)
        self.start_pose_subscriber = rospy.Subscriber(
            self.start_pose_topic,
            PoseStamped,
            self.start_pose_callback,
            queue_size=1,
        )
        self.marker_points = []
        self.start_time = None
        self.start_x = None
        self.start_y = None
        self.start_z = None
        self.has_published_initial_pose = False

        rospy.loginfo(
            "Waiting for start pose on %s before publishing straight root-link FLU trajectory on %s at %.1f Hz with speed %.3f m/s along world -X; blue trajectory marker on %s.",
            rospy.resolve_name(self.start_pose_topic),
            self.publisher.resolved_name,
            self.publish_rate_hz,
            self.linear_speed,
            self.marker_publisher.resolved_name,
        )

    def run(self):
        rate = rospy.Rate(self.publish_rate_hz)
        while not rospy.is_shutdown():
            if not self.is_initialized():
                rospy.loginfo_throttle(
                    2.0,
                    "Waiting for start pose on %s before publishing %s",
                    rospy.resolve_name(self.start_pose_topic),
                    self.publisher.resolved_name,
                )
                rate.sleep()
                continue

            elapsed = (rospy.Time.now() - self.start_time).to_sec()
            if self.run_duration_sec > 0.0 and elapsed >= self.run_duration_sec:
                rospy.loginfo(
                    "Straight root-link trajectory reached run duration %.3f s; shutting down.",
                    self.run_duration_sec,
                )
                rospy.signal_shutdown("run duration reached")
                break

            if not self.has_published_initial_pose:
                elapsed = 0.0

            pose_msg = self.build_pose_message(elapsed)
            self.publisher.publish(pose_msg)
            self.update_marker_points(pose_msg)
            self.marker_publisher.publish(self.build_trajectory_marker(pose_msg.header.stamp))
            self.has_published_initial_pose = True
            rate.sleep()

    def build_pose_message(self, elapsed):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = self.frame_id
        pose_msg.pose.position.x = self.start_x - self.linear_speed * elapsed
        pose_msg.pose.position.y = self.start_y
        pose_msg.pose.position.z = self.start_z

        orientation = self.quaternion_from_yaw(self.YAW)
        pose_msg.pose.orientation.x = orientation[0]
        pose_msg.pose.orientation.y = orientation[1]
        pose_msg.pose.orientation.z = orientation[2]
        pose_msg.pose.orientation.w = orientation[3]
        return pose_msg

    @staticmethod
    def quaternion_from_yaw(yaw):
        half_yaw = 0.5 * yaw
        return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))

    def start_pose_callback(self, msg):
        if self.is_initialized():
            return

        start_x = msg.pose.position.x
        start_y = msg.pose.position.y
        start_z = msg.pose.position.z
        if not self.is_finite_position(start_x, start_y, start_z):
            rospy.logwarn_throttle(
                1.0,
                "Ignoring non-finite start pose from %s",
                rospy.resolve_name(self.start_pose_topic),
            )
            return

        self.start_x = start_x
        self.start_y = start_y
        self.start_z = start_z
        self.start_time = rospy.Time.now()
        self.has_published_initial_pose = False
        self.marker_points = [self.build_point(start_x, start_y, start_z)]
        rospy.loginfo(
            "Initialized straight root-link trajectory from %s at [%.3f, %.3f, %.3f]",
            rospy.resolve_name(self.start_pose_topic),
            self.start_x,
            self.start_y,
            self.start_z,
        )

    def is_initialized(self):
        return self.start_time is not None

    def update_marker_points(self, pose_msg):
        point = self.build_point(pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z)

        if self.marker_points and self.distance(point, self.marker_points[-1]) < self.marker_sample_interval_m:
            return

        self.marker_points.append(point)

    def build_trajectory_marker(self, stamp):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.frame_id
        marker.ns = self.MARKER_NS
        marker.id = self.MARKER_ID
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.marker_line_width_m
        marker.color.r = 0.0
        marker.color.g = 0.25
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.lifetime = rospy.Duration(0.0)
        marker.points = list(self.marker_points)
        return marker

    @staticmethod
    def distance(lhs, rhs):
        return math.sqrt(
            (lhs.x - rhs.x) * (lhs.x - rhs.x)
            + (lhs.y - rhs.y) * (lhs.y - rhs.y)
            + (lhs.z - rhs.z) * (lhs.z - rhs.z)
        )

    @staticmethod
    def build_point(x, y, z):
        point = Point()
        point.x = x
        point.y = y
        point.z = z
        return point

    @staticmethod
    def is_finite_position(x, y, z):
        return math.isfinite(x) and math.isfinite(y) and math.isfinite(z)


if __name__ == "__main__":
    try:
        RootStraightTrajectoryPublisher().run()
    except rospy.ROSInterruptException:
        pass
