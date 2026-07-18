#!/usr/bin/env python3
"""Ideal point-robot model driven directly by FlightNav position commands."""

import copy
import math
import threading

import rospy
import tf.transformations as tf_trans
import tf2_ros
from aerial_robot_msgs.msg import FlightNav
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


POSITION_MODES = (FlightNav.POS_MODE, FlightNav.POS_VEL_MODE)


def is_finite(*values):
    return all(math.isfinite(value) for value in values)


class PointRobotModel:
    def __init__(self):
        self.world_frame_id = rospy.get_param("~world_frame_id", "world")
        self.cog_frame_id = rospy.get_param("~cog_frame_id", "quadrotor/cog")
        self.publish_tf = bool(rospy.get_param("~publish_tf", True))
        self.publish_rate = float(rospy.get_param("~publish_rate", 40.0))

        spawn_x = float(rospy.get_param("~spawn_x", 0.0))
        spawn_y = float(rospy.get_param("~spawn_y", 0.0))
        spawn_z = float(rospy.get_param("~spawn_z", 1.0))
        spawn_yaw = float(rospy.get_param("~spawn_yaw", 0.0))

        self.lock = threading.Lock()
        self.odom = Odometry()
        self.odom.header.frame_id = self.world_frame_id
        self.odom.child_frame_id = self.cog_frame_id
        self.odom.pose.pose.position.x = spawn_x
        self.odom.pose.pose.position.y = spawn_y
        self.odom.pose.pose.position.z = spawn_z
        quaternion = tf_trans.quaternion_from_euler(0.0, 0.0, spawn_yaw)
        self.set_quaternion(self.odom.pose.pose.orientation, quaternion)

        self.odom_pub = rospy.Publisher("uav/cog/odom", Odometry, queue_size=10)
        self.nav_sub = rospy.Subscriber(
            "uav/nav", FlightNav, self.nav_callback, queue_size=1, tcp_nodelay=True
        )
        self.tf_broadcaster = tf2_ros.TransformBroadcaster() if self.publish_tf else None

        publish_rate = self.publish_rate if self.publish_rate > 0.0 else 40.0
        self.timer = rospy.Timer(
            rospy.Duration.from_sec(1.0 / publish_rate), self.timer_callback
        )

        rospy.loginfo(
            "Point geometric model follows %s and publishes %s.",
            rospy.resolve_name("uav/nav"),
            rospy.resolve_name("uav/cog/odom"),
        )

    @staticmethod
    def set_quaternion(message, quaternion):
        message.x = quaternion[0]
        message.y = quaternion[1]
        message.z = quaternion[2]
        message.w = quaternion[3]

    def nav_callback(self, message):
        active_values = []
        if message.pos_xy_nav_mode in POSITION_MODES:
            active_values.extend((message.target_pos_x, message.target_pos_y))
        if message.pos_z_nav_mode in POSITION_MODES:
            active_values.append(message.target_pos_z)
        if message.yaw_nav_mode in POSITION_MODES:
            active_values.append(message.target_yaw)

        if not is_finite(*active_values):
            rospy.logwarn_throttle(1.0, "Ignoring FlightNav with non-finite target values.")
            return

        changed = False
        with self.lock:
            if message.pos_xy_nav_mode in POSITION_MODES:
                self.odom.pose.pose.position.x = message.target_pos_x
                self.odom.pose.pose.position.y = message.target_pos_y
                changed = True

            if message.pos_z_nav_mode in POSITION_MODES:
                self.odom.pose.pose.position.z = message.target_pos_z
                changed = True

            if message.yaw_nav_mode in POSITION_MODES:
                quaternion = tf_trans.quaternion_from_euler(0.0, 0.0, message.target_yaw)
                self.set_quaternion(self.odom.pose.pose.orientation, quaternion)
                changed = True

            if changed:
                self.odom.twist.twist.linear.x = 0.0
                self.odom.twist.twist.linear.y = 0.0
                self.odom.twist.twist.linear.z = 0.0
                self.odom.twist.twist.angular.x = 0.0
                self.odom.twist.twist.angular.y = 0.0
                self.odom.twist.twist.angular.z = 0.0

        if changed:
            self.publish_state()

    def timer_callback(self, _event):
        self.publish_state()

    def publish_state(self):
        with self.lock:
            odom = copy.deepcopy(self.odom)

        odom.header.stamp = rospy.Time.now()
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is None:
            return

        transform = TransformStamped()
        transform.header.stamp = odom.header.stamp
        transform.header.frame_id = self.world_frame_id
        transform.child_frame_id = self.cog_frame_id
        transform.transform.translation.x = odom.pose.pose.position.x
        transform.transform.translation.y = odom.pose.pose.position.y
        transform.transform.translation.z = odom.pose.pose.position.z
        transform.transform.rotation = copy.deepcopy(odom.pose.pose.orientation)
        self.tf_broadcaster.sendTransform(transform)


if __name__ == "__main__":
    rospy.init_node("point_robot_model")
    PointRobotModel()
    rospy.spin()
