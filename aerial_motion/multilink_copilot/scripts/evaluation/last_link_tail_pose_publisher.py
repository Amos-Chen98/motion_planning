#!/usr/bin/env python3

import math

import rospy
import tf
import tf.transformations as tf_trans
from geometry_msgs.msg import PoseStamped


class LastLinkTailPosePublisher:
    NODE_NAME = "last_link_tail_pose_publisher"
    ROOT_POSE_TOPIC = "root/pose"
    LAST_LINK_TAIL_POSE_TOPIC = "last_link/tail_pose"
    TF_LOOKUP_TIMEOUT_SEC = 0.05
    LOG_THROTTLE_SEC = 2.0

    def __init__(self):
        rospy.init_node(self.NODE_NAME, anonymous=False)

        self.root_frame = rospy.get_param("~root_frame", "dragon/root")
        self.last_link_frame = rospy.get_param("~last_link_frame", "dragon/link4")
        self.tail_offset_x = float(rospy.get_param("~tail_offset_x", 0.455))
        self.tf_lookup_timeout = rospy.Duration.from_sec(self.TF_LOOKUP_TIMEOUT_SEC)

        self.tf_listener = tf.TransformListener()
        self.tail_pose_publisher = rospy.Publisher(self.LAST_LINK_TAIL_POSE_TOPIC, PoseStamped, queue_size=1)
        self.root_pose_subscriber = rospy.Subscriber(
            self.ROOT_POSE_TOPIC,
            PoseStamped,
            self.root_pose_callback,
            queue_size=1,
        )

        rospy.loginfo(
            "Publishing %s from %s using TF %s -> %s with tail_offset_x=%.3f m",
            self.tail_pose_publisher.resolved_name,
            rospy.resolve_name(self.ROOT_POSE_TOPIC),
            self.root_frame,
            self.last_link_frame,
            self.tail_offset_x,
        )

    def root_pose_callback(self, msg):
        relative_transform = self.lookup_root_to_last_link_transform(msg.header.stamp)
        if relative_transform is None:
            return

        try:
            world_last_link_position, world_last_link_orientation = self.compose_pose_with_relative_transform(
                self.position_from_pose(msg),
                self.orientation_from_pose(msg),
                relative_transform[0],
                relative_transform[1],
            )
            tail_offset_world = self.rotate_vector(world_last_link_orientation, (self.tail_offset_x, 0.0, 0.0))
        except ValueError as exc:
            rospy.logwarn_throttle(self.LOG_THROTTLE_SEC, "Skipping last-link tail pose publish: %s", exc)
            return

        tail_pose_msg = PoseStamped()
        tail_pose_msg.header.stamp = msg.header.stamp
        tail_pose_msg.header.frame_id = msg.header.frame_id
        tail_pose_msg.pose.position.x = world_last_link_position[0] + tail_offset_world[0]
        tail_pose_msg.pose.position.y = world_last_link_position[1] + tail_offset_world[1]
        tail_pose_msg.pose.position.z = world_last_link_position[2] + tail_offset_world[2]
        tail_pose_msg.pose.orientation.x = world_last_link_orientation[0]
        tail_pose_msg.pose.orientation.y = world_last_link_orientation[1]
        tail_pose_msg.pose.orientation.z = world_last_link_orientation[2]
        tail_pose_msg.pose.orientation.w = world_last_link_orientation[3]
        self.tail_pose_publisher.publish(tail_pose_msg)

    def lookup_root_to_last_link_transform(self, stamp):
        lookup_time = stamp if stamp != rospy.Time() else rospy.Time(0)

        try:
            self.tf_listener.waitForTransform(
                self.root_frame,
                self.last_link_frame,
                lookup_time,
                self.tf_lookup_timeout,
            )
            return self.tf_listener.lookupTransform(self.root_frame, self.last_link_frame, lookup_time)
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as exact_exc:
            try:
                self.tf_listener.waitForTransform(
                    self.root_frame,
                    self.last_link_frame,
                    rospy.Time(0),
                    self.tf_lookup_timeout,
                )
                rospy.logdebug_throttle(
                    self.LOG_THROTTLE_SEC,
                    "Falling back to latest TF for %s -> %s at stamp %.6f",
                    self.root_frame,
                    self.last_link_frame,
                    stamp.to_sec(),
                )
                return self.tf_listener.lookupTransform(self.root_frame, self.last_link_frame, rospy.Time(0))
            except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as latest_exc:
                rospy.logwarn_throttle(
                    self.LOG_THROTTLE_SEC,
                    "Failed to resolve TF %s -> %s at stamp %.6f and latest TF; skipping sample (exact: %s, latest: %s)",
                    self.root_frame,
                    self.last_link_frame,
                    stamp.to_sec(),
                    exact_exc,
                    latest_exc,
                )
                return None

    def compose_pose_with_relative_transform(
        self,
        parent_position,
        parent_orientation,
        relative_position,
        relative_orientation,
    ):
        normalized_parent_orientation = self.normalize_quaternion(parent_orientation)
        normalized_relative_orientation = self.normalize_quaternion(relative_orientation)
        world_child_orientation = self.normalize_quaternion(
            tf_trans.quaternion_multiply(normalized_parent_orientation, normalized_relative_orientation)
        )
        rotated_relative_position = self.rotate_vector(normalized_parent_orientation, relative_position)
        world_child_position = (
            parent_position[0] + rotated_relative_position[0],
            parent_position[1] + rotated_relative_position[1],
            parent_position[2] + rotated_relative_position[2],
        )
        return world_child_position, world_child_orientation

    def rotate_vector(self, quaternion, vector):
        normalized_quaternion = self.normalize_quaternion(quaternion)
        vector_quaternion = (vector[0], vector[1], vector[2], 0.0)
        rotated = tf_trans.quaternion_multiply(
            tf_trans.quaternion_multiply(normalized_quaternion, vector_quaternion),
            tf_trans.quaternion_conjugate(normalized_quaternion),
        )
        return (rotated[0], rotated[1], rotated[2])

    def normalize_quaternion(self, quaternion):
        norm = math.sqrt(sum(component * component for component in quaternion))
        if norm <= 1e-12:
            raise ValueError("Received zero-norm quaternion while computing last-link tail pose.")
        return tuple(component / norm for component in quaternion)

    @staticmethod
    def position_from_pose(msg):
        return (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        )

    @staticmethod
    def orientation_from_pose(msg):
        return (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        LastLinkTailPosePublisher().run()
    except rospy.ROSInterruptException:
        pass
