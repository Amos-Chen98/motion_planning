#!/usr/bin/env python3

import math

import rospy
from sensor_msgs.msg import JointState


JOINT_CONTROL_TOPIC = "/dragon/joints_ctrl"
TARGET_JOINT_POSITIONS = [
    0.0,
    math.pi / 2.0,
    0.0,
    math.pi / 2.0,
    0.0,
    math.pi / 2.0,
]


def main():
    rospy.init_node("reset_dragon_joint")

    joint_control_pub = rospy.Publisher(JOINT_CONTROL_TOPIC, JointState, queue_size=10)

    desire_joint = JointState()
    desire_joint.position = TARGET_JOINT_POSITIONS

    rospy.sleep(0.5)
    joint_control_pub.publish(desire_joint)
    rospy.loginfo("Published dragon joint target to %s: %s", JOINT_CONTROL_TOPIC, TARGET_JOINT_POSITIONS)
    rospy.sleep(0.1)


if __name__ == "__main__":
    main()
