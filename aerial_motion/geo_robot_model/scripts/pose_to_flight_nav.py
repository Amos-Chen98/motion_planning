#!/usr/bin/env python3
"""Bridge PoseStamped position commands to aerial_robot FlightNav commands."""

import rospy
import tf.transformations as tf_trans
from aerial_robot_msgs.msg import FlightNav
from geometry_msgs.msg import PoseStamped


class PoseToFlightNav:
    def __init__(self):
        self.publish_yaw_command = rospy.get_param("~publish_yaw_command", False)
        self.publisher = rospy.Publisher("flight_nav", FlightNav, queue_size=10)
        self.subscriber = rospy.Subscriber(
            "pose_command", PoseStamped, self.pose_callback, tcp_nodelay=True
        )

        rospy.loginfo(
            "Pose-to-FlightNav bridge follows %s and publishes %s.",
            rospy.resolve_name("pose_command"),
            rospy.resolve_name("flight_nav"),
        )

    def pose_callback(self, message):
        nav = FlightNav()
        nav.header.stamp = rospy.Time.now()
        nav.header.frame_id = message.header.frame_id
        nav.control_frame = FlightNav.WORLD_FRAME
        nav.target = FlightNav.COG

        nav.pos_xy_nav_mode = FlightNav.POS_MODE
        nav.target_pos_x = message.pose.position.x
        nav.target_pos_y = message.pose.position.y

        nav.pos_z_nav_mode = FlightNav.POS_MODE
        nav.target_pos_z = message.pose.position.z

        nav.roll_nav_mode = FlightNav.NO_NAVIGATION
        nav.pitch_nav_mode = FlightNav.NO_NAVIGATION

        if self.publish_yaw_command:
            quaternion = message.pose.orientation
            _, _, yaw = tf_trans.euler_from_quaternion(
                [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
            )
            nav.yaw_nav_mode = FlightNav.POS_MODE
            nav.target_yaw = yaw
        else:
            nav.yaw_nav_mode = FlightNav.NO_NAVIGATION

        self.publisher.publish(nav)


if __name__ == "__main__":
    rospy.init_node("pose_to_flight_nav")
    PoseToFlightNav()
    rospy.spin()
