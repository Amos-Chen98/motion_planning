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
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom')
        self.camera_init_frame_id = rospy.get_param(
            '~camera_init_frame_id',
            rospy.get_param('~frame_id', 'camera_init'),
        )
        self.publish_tf = bool(rospy.get_param('~publish_tf', False))
        self.publish_rate = float(rospy.get_param('~publish_rate', 40.0))

        spawn_x = float(rospy.get_param('~spawn_x', 0.0))
        spawn_y = float(rospy.get_param('~spawn_y', 0.0))
        spawn_z = float(rospy.get_param('~spawn_z', 1.0))
        spawn_yaw = float(rospy.get_param('~spawn_yaw', 0.0))
        # Fixed world -> camera_init anchor (the LIO origin), placed at the spawn pose.
        self.world_camera_init_translation = [spawn_x, spawn_y, spawn_z]
        self.world_camera_init_quaternion = yaw_quaternion(spawn_yaw)

        self.lock = threading.Lock()
        self.odom = Odometry()
        self.odom.header.frame_id = self.world_frame_id
        self.odom.child_frame_id = self.odom_frame_id
        self.odom.pose.pose.position.x = spawn_x
        self.odom.pose.pose.position.y = spawn_y
        self.odom.pose.pose.position.z = spawn_z
        set_quaternion(self.odom.pose.pose.orientation, yaw_quaternion(spawn_yaw))

        self.pub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.sub = rospy.Subscriber('target_pose', PoseStamped, self.target_callback,
                                    tcp_nodelay=True)

        self.tf_broadcaster = None
        self.static_tf_broadcaster = None
        if self.publish_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster()
            self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
            self.publish_static_transforms()

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
            self.odom.child_frame_id = self.odom_frame_id
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
        self.publish_world_to_odom(odom)

    def publish_world_to_odom(self, odom):
        """Real-time world -> odom transform carrying the current robot pose."""
        if self.tf_broadcaster is None:
            return

        world_odom = TransformStamped()
        world_odom.header.stamp = odom.header.stamp
        world_odom.header.frame_id = self.world_frame_id
        world_odom.child_frame_id = self.odom_frame_id
        world_odom.transform.translation.x = odom.pose.pose.position.x
        world_odom.transform.translation.y = odom.pose.pose.position.y
        world_odom.transform.translation.z = odom.pose.pose.position.z
        world_odom.transform.rotation = copy.deepcopy(odom.pose.pose.orientation)
        self.tf_broadcaster.sendTransform(world_odom)

    def publish_static_transforms(self):
        """Publish only the fixed world -> camera_init reference frame.

        The robot and lidar frames are deliberately not attached here. Their TF chain is
        world -> odom_frame_id (dynamic) -> lidar_imu_frame_id (fixed sensor extrinsic).
        Giving the lidar frame another static parent would freeze or corrupt its world pose.
        """
        stamp = rospy.Time.now()

        world_camera_init = TransformStamped()
        world_camera_init.header.stamp = stamp
        world_camera_init.header.frame_id = self.world_frame_id
        world_camera_init.child_frame_id = self.camera_init_frame_id
        world_camera_init.transform.translation.x = self.world_camera_init_translation[0]
        world_camera_init.transform.translation.y = self.world_camera_init_translation[1]
        world_camera_init.transform.translation.z = self.world_camera_init_translation[2]
        set_quaternion(world_camera_init.transform.rotation, self.world_camera_init_quaternion)

        self.static_tf_broadcaster.sendTransform(world_camera_init)


if __name__ == '__main__':
    rospy.init_node('point_robot_model')
    PointRobotModel()
    rospy.spin()
