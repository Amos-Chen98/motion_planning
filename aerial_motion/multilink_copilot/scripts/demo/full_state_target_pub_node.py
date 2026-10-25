#!/usr/bin/env python3

import copy
import re
import time

import rospy
from aerial_robot_msgs.msg import FullStateTarget
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState


ROBOT_NS = "/dragon"


def main():
    # Keep Python's Ctrl+C behavior while waiting for terminal input.
    rospy.init_node("full_state_target_pub", disable_signals=True)
    publisher = rospy.Publisher(ROBOT_NS + "/full_state_target", FullStateTarget, queue_size=1)

    rospy.loginfo("Waiting for initial %s/root/pose and %s/joint_states.", ROBOT_NS, ROBOT_NS)
    initial_pose = rospy.wait_for_message(ROBOT_NS + "/root/pose", PoseStamped)
    joint_names = []
    while not rospy.is_shutdown() and not joint_names:
        joints = rospy.wait_for_message(ROBOT_NS + "/joint_states", JointState)
        joint_names = [name for name in joints.name if re.fullmatch(r"joint\d+_(yaw|pitch)", name)]

    rospy.loginfo("Waiting for a %s/full_state_target subscriber.", ROBOT_NS)
    while not rospy.is_shutdown() and publisher.get_num_connections() == 0:
        time.sleep(0.1)

    for index, (offset_x, yaw) in enumerate(((0.0, 1.4), (0.2, 1.4), (0.0, 1.57)), 1):
        if rospy.is_shutdown():
            return
        input("Press Enter to send target %d/3 (x offset %.1f m, yaw %.2f rad): " % (index, offset_x, yaw))
        if rospy.is_shutdown():
            return

        target = FullStateTarget()
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = initial_pose.header.frame_id
        target.root_state.header = copy.deepcopy(target.header)
        target.root_state.pose.pose = copy.deepcopy(initial_pose.pose)
        target.root_state.pose.pose.position.x += offset_x
        target.joint_state.header = copy.deepcopy(target.header)
        target.joint_state.name = joint_names
        target.joint_state.position = [yaw if name.endswith("_yaw") else 0.0 for name in joint_names]
        target.joint_state.velocity = [0.0] * len(joint_names)
        publisher.publish(target)
        rospy.loginfo("Sent target %d/3.", index)

    time.sleep(0.2)


if __name__ == "__main__":
    try:
        main()
    except (rospy.ROSInterruptException, KeyboardInterrupt, EOFError):
        pass
    finally:
        rospy.signal_shutdown("demo finished")
