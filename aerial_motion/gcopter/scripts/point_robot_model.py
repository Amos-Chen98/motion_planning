#! /usr/bin/env python3
"""Point-robot model for the standalone GCOPTER demo."""

import copy
import math
import threading

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class PointRobotModel:
    def __init__(self):
        self.frame_id = rospy.get_param('~frame_id', 'odom')
        self.child_frame_id = rospy.get_param('~child_frame_id', 'base_link')
        self.publish_rate = float(rospy.get_param('~publish_rate', 40.0))

        spawn_x = float(rospy.get_param('~spawn_x', 0.0))
        spawn_y = float(rospy.get_param('~spawn_y', 0.0))
        spawn_z = float(rospy.get_param('~spawn_z', 1.0))
        spawn_yaw = float(rospy.get_param('~spawn_yaw', 0.0))

        self.lock = threading.Lock()
        self.odom = Odometry()
        self.odom.header.frame_id = self.frame_id
        self.odom.child_frame_id = self.child_frame_id
        self.odom.pose.pose.position.x = spawn_x
        self.odom.pose.pose.position.y = spawn_y
        self.odom.pose.pose.position.z = spawn_z
        self.odom.pose.pose.orientation.x = 0.0
        self.odom.pose.pose.orientation.y = 0.0
        self.odom.pose.pose.orientation.z = math.sin(0.5 * spawn_yaw)
        self.odom.pose.pose.orientation.w = math.cos(0.5 * spawn_yaw)

        self.pub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.sub = rospy.Subscriber('target_pose', PoseStamped, self.target_callback,
                                    tcp_nodelay=True)

        period = 1.0 / self.publish_rate if self.publish_rate > 0.0 else 1.0 / 40.0
        self.timer = rospy.Timer(rospy.Duration.from_sec(period), self.timer_callback)

        rospy.loginfo(
            "Point robot model publishing %s and following %s.",
            rospy.resolve_name('odom'),
            rospy.resolve_name('target_pose'),
        )

    def target_callback(self, msg):
        with self.lock:
            self.odom.header.frame_id = self.frame_id
            self.odom.child_frame_id = self.child_frame_id
            self.odom.pose.pose.position = copy.deepcopy(msg.pose.position)
            self.odom.pose.pose.orientation = copy.deepcopy(msg.pose.orientation)
            odom = copy.deepcopy(self.odom)

        self.publish(odom)

    def timer_callback(self, _event):
        with self.lock:
            odom = copy.deepcopy(self.odom)

        self.publish(odom)

    def publish(self, odom):
        odom.header.stamp = rospy.Time.now()
        self.pub.publish(odom)


if __name__ == '__main__':
    rospy.init_node('point_robot_model')
    PointRobotModel()
    rospy.spin()
