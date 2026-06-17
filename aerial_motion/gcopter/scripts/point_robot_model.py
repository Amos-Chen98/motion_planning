#! /usr/bin/env python3
"""Point-robot model for the standalone GCOPTER demo."""

import copy
import math
import threading

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry


def yaw_quaternion(yaw):
    return [0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw)]


def set_quaternion(msg_quat, q):
    msg_quat.x = q[0]
    msg_quat.y = q[1]
    msg_quat.z = q[2]
    msg_quat.w = q[3]


class PointRobotModel:
    def __init__(self):
        self.world_frame_id = rospy.get_param('~world_frame_id', 'world')
        self.camera_init_frame_id = rospy.get_param(
            '~camera_init_frame_id',
            rospy.get_param('~frame_id', 'camera_init'),
        )
        self.child_frame_id = rospy.get_param('~child_frame_id', 'base_link')
        self.publish_tf = bool(rospy.get_param('~publish_tf', False))
        self.publish_rate = float(rospy.get_param('~publish_rate', 40.0))

        spawn_x = float(rospy.get_param('~spawn_x', 0.0))
        spawn_y = float(rospy.get_param('~spawn_y', 0.0))
        spawn_z = float(rospy.get_param('~spawn_z', 1.0))
        spawn_yaw = float(rospy.get_param('~spawn_yaw', 0.0))
        self.world_camera_translation = [spawn_x, spawn_y, spawn_z]
        self.world_camera_quaternion = yaw_quaternion(spawn_yaw)

        self.lock = threading.Lock()
        self.odom = Odometry()
        self.odom.header.frame_id = self.world_frame_id
        self.odom.child_frame_id = self.child_frame_id
        self.odom.pose.pose.position.x = spawn_x
        self.odom.pose.pose.position.y = spawn_y
        self.odom.pose.pose.position.z = spawn_z
        set_quaternion(self.odom.pose.pose.orientation, self.world_camera_quaternion)

        self.pub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.sub = rospy.Subscriber('target_pose', PoseStamped, self.target_callback,
                                    tcp_nodelay=True)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster() if self.publish_tf else None

        period = 1.0 / self.publish_rate if self.publish_rate > 0.0 else 1.0 / 40.0
        self.timer = rospy.Timer(rospy.Duration.from_sec(period), self.timer_callback)

        rospy.loginfo(
            "Point robot model publishing %s and following %s.",
            rospy.resolve_name('odom'),
            rospy.resolve_name('target_pose'),
        )

    def target_callback(self, msg):
        with self.lock:
            self.odom.header.frame_id = self.world_frame_id
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
        self.publish_transforms(odom)

    def publish_transforms(self, odom):
        if self.tf_broadcaster is None:
            return

        transforms = []
        stamp = odom.header.stamp

        world_camera = TransformStamped()
        world_camera.header.stamp = stamp
        world_camera.header.frame_id = self.world_frame_id
        world_camera.child_frame_id = self.camera_init_frame_id
        world_camera.transform.translation.x = self.world_camera_translation[0]
        world_camera.transform.translation.y = self.world_camera_translation[1]
        world_camera.transform.translation.z = self.world_camera_translation[2]
        set_quaternion(world_camera.transform.rotation, self.world_camera_quaternion)
        transforms.append(world_camera)

        world_body = TransformStamped()
        world_body.header.stamp = stamp
        world_body.header.frame_id = self.world_frame_id
        world_body.child_frame_id = odom.child_frame_id
        world_body.transform.translation.x = odom.pose.pose.position.x
        world_body.transform.translation.y = odom.pose.pose.position.y
        world_body.transform.translation.z = odom.pose.pose.position.z
        world_body.transform.rotation = copy.deepcopy(odom.pose.pose.orientation)
        transforms.append(world_body)

        self.tf_broadcaster.sendTransform(transforms)


if __name__ == '__main__':
    rospy.init_node('point_robot_model')
    PointRobotModel()
    rospy.spin()
