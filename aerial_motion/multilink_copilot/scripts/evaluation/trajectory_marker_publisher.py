#!/usr/bin/env python3

import threading

import rospy
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray


class TrajectoryMarkerPublisher(object):
    NODE_NAME = "trajectory_marker_publisher"
    DEFAULT_ROOT_TOPIC = "/dragon/root/pose"
    DEFAULT_LAST_LINK_TOPIC = "/dragon/last_link/tail_pose"
    DEFAULT_MARKER_TOPIC = "trajectory_markers"
    DEFAULT_PUBLISH_RATE_HZ = 10.0
    DEFAULT_LINE_WIDTH_M = 0.03
    LOG_THROTTLE_SEC = 2.0

    ROOT_MARKER_NS = "root_pose_trajectory"
    LAST_LINK_MARKER_NS = "last_link_tail_pose_trajectory"
    ROOT_MARKER_ID = 0
    LAST_LINK_MARKER_ID = 1
    ROOT_MARKER_COLOR_RGBA = (0.1411764705882353, 0.7254901960784313, 0.050980392156862744, 1.0)
    LAST_LINK_MARKER_COLOR_RGBA = (0.6352941176470588, 0.050980392156862744, 0.7254901960784313, 1.0)

    def __init__(self):
        rospy.init_node(self.NODE_NAME, anonymous=False)

        self.root_topic = rospy.get_param("~root_topic", self.DEFAULT_ROOT_TOPIC)
        self.last_link_topic = rospy.get_param("~last_link_topic", self.DEFAULT_LAST_LINK_TOPIC)
        self.marker_topic = rospy.get_param("~marker_topic", self.DEFAULT_MARKER_TOPIC)
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", self.DEFAULT_PUBLISH_RATE_HZ))
        self.line_width_m = float(rospy.get_param("~line_width_m", self.DEFAULT_LINE_WIDTH_M))

        if self.publish_rate_hz <= 0.0:
            raise ValueError("~publish_rate_hz must be greater than 0.")
        if self.line_width_m <= 0.0:
            raise ValueError("~line_width_m must be greater than 0.")

        self.lock = threading.RLock()
        self.reference_frame_id = None
        self.root_points = []
        self.last_link_points = []

        self.marker_publisher = rospy.Publisher(
            self.marker_topic,
            MarkerArray,
            queue_size=1,
            latch=True,
        )
        self.root_subscriber = rospy.Subscriber(
            self.root_topic,
            PoseStamped,
            self.root_pose_callback,
            queue_size=200,
        )
        self.last_link_subscriber = rospy.Subscriber(
            self.last_link_topic,
            PoseStamped,
            self.last_link_pose_callback,
            queue_size=200,
        )
        self.publish_timer = rospy.Timer(
            rospy.Duration.from_sec(1.0 / self.publish_rate_hz),
            self.publish_timer_callback,
        )

        rospy.loginfo(
            "Publishing pose trajectory markers on %s from %s and %s at %.3f Hz with line width %.3f m.",
            self.marker_publisher.resolved_name,
            rospy.resolve_name(self.root_topic),
            rospy.resolve_name(self.last_link_topic),
            self.publish_rate_hz,
            self.line_width_m,
        )

    def root_pose_callback(self, msg):
        self.append_pose_sample(
            msg,
            rospy.resolve_name(self.root_topic),
            self.root_points,
        )

    def last_link_pose_callback(self, msg):
        self.append_pose_sample(
            msg,
            rospy.resolve_name(self.last_link_topic),
            self.last_link_points,
        )

    def append_pose_sample(self, msg, topic_name, point_list):
        if not self.capture_reference_frame(msg.header.frame_id, topic_name):
            return

        point = Point()
        point.x = msg.pose.position.x
        point.y = msg.pose.position.y
        point.z = msg.pose.position.z

        with self.lock:
            point_list.append(point)

    def capture_reference_frame(self, frame_id, topic_name):
        if not frame_id:
            rospy.logwarn_throttle(
                self.LOG_THROTTLE_SEC,
                "Discarding %s sample with empty frame_id.",
                topic_name,
            )
            return False

        with self.lock:
            if self.reference_frame_id is None:
                self.reference_frame_id = frame_id
                rospy.loginfo(
                    "Captured trajectory marker reference frame '%s' from %s.",
                    self.reference_frame_id,
                    topic_name,
                )
                return True

            expected_frame_id = self.reference_frame_id

        if frame_id != expected_frame_id:
            rospy.logwarn_throttle(
                self.LOG_THROTTLE_SEC,
                "Discarding %s sample with frame_id '%s'; expected '%s'.",
                topic_name,
                frame_id,
                expected_frame_id,
            )
            return False

        return True

    def publish_timer_callback(self, _event):
        self.publish_markers()

    def publish_markers(self):
        with self.lock:
            if self.reference_frame_id is None:
                return

            reference_frame_id = self.reference_frame_id
            root_points = list(self.root_points)
            last_link_points = list(self.last_link_points)

        marker_array = MarkerArray()
        marker_array.markers.append(
            self.build_line_strip_marker(
                reference_frame_id,
                self.ROOT_MARKER_NS,
                self.ROOT_MARKER_ID,
                root_points,
                self.ROOT_MARKER_COLOR_RGBA,
            )
        )
        marker_array.markers.append(
            self.build_line_strip_marker(
                reference_frame_id,
                self.LAST_LINK_MARKER_NS,
                self.LAST_LINK_MARKER_ID,
                last_link_points,
                self.LAST_LINK_MARKER_COLOR_RGBA,
            )
        )
        self.marker_publisher.publish(marker_array)

    def build_line_strip_marker(self, frame_id, namespace, marker_id, points, color):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time(0)
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.line_width_m
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.lifetime = rospy.Duration(0)
        marker.points = points
        return marker

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        TrajectoryMarkerPublisher().run()
    except rospy.ROSInterruptException:
        pass
