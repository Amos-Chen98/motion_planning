#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import QuaternionStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf.transformations import euler_from_quaternion, quaternion_from_euler


JOINT_CONTROL_TOPIC = "/dragon/joints_ctrl"
ATTITUDE_CONTROL_TOPIC = "/dragon/final_target_baselink_rot"
BASELINK_ODOM_TOPIC = "/dragon/uav/baselink/odom"
TARGET_JOINT_POSITIONS = [
    0.0,
    math.pi / 2.0,
    0.0,
    math.pi / 2.0,
    0.0,
    math.pi / 2.0,
]


def level_body(attitude_control_pub):
    try:
        odom = rospy.wait_for_message(BASELINK_ODOM_TOPIC, Odometry, timeout=5.0)
    except rospy.ROSException as exc:
        rospy.logwarn("Skipping body leveling: could not read %s: %s", BASELINK_ODOM_TOPIC, exc)
        return

    orientation = odom.pose.pose.orientation
    quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
    if not all(math.isfinite(value) for value in quaternion) or math.hypot(*quaternion) < 1e-6:
        rospy.logwarn("Skipping body leveling: invalid orientation from %s", BASELINK_ODOM_TOPIC)
        return

    yaw = euler_from_quaternion(quaternion)[2]
    desire_attitude = QuaternionStamped()
    desire_attitude.header.stamp = rospy.Time.now()
    desire_attitude.header.frame_id = odom.header.frame_id
    # Keep the measured world heading; DragonNavigator interpolates to level.
    (
        desire_attitude.quaternion.x,
        desire_attitude.quaternion.y,
        desire_attitude.quaternion.z,
        desire_attitude.quaternion.w,
    ) = quaternion_from_euler(0.0, 0.0, yaw)
    attitude_control_pub.publish(desire_attitude)
    rospy.loginfo("Published level body target to %s: roll=0, pitch=0, yaw=%.3f rad", ATTITUDE_CONTROL_TOPIC, yaw)


def main():
    rospy.init_node("reset_dragon_joint")

    joint_control_pub = rospy.Publisher(JOINT_CONTROL_TOPIC, JointState, queue_size=10)
    attitude_control_pub = rospy.Publisher(ATTITUDE_CONTROL_TOPIC, QuaternionStamped, queue_size=10)

    desire_joint = JointState()
    desire_joint.position = TARGET_JOINT_POSITIONS

    rospy.sleep(0.5)
    joint_control_pub.publish(desire_joint)
    rospy.loginfo("Published dragon joint target to %s: %s", JOINT_CONTROL_TOPIC, TARGET_JOINT_POSITIONS)
    level_body(attitude_control_pub)
    rospy.sleep(0.1)


if __name__ == "__main__":
    main()
