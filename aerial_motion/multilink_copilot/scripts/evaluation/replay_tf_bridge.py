#!/usr/bin/env python3

import rospy
from tf2_msgs.msg import TFMessage


def normalize_frame(frame_id):
    return frame_id[1:] if frame_id.startswith("/") else frame_id


class ReplayTfBridge:
    def __init__(self):
        robot_ns = normalize_frame(rospy.get_param("~robot_ns", "dragon")).strip("/")
        self.world_frame = normalize_frame(rospy.get_param("~world_frame", "world"))
        self.root_frame = normalize_frame(rospy.get_param("~root_frame", "{}/root".format(robot_ns)))

        bag_tf_topic = rospy.get_param("~bag_tf_topic", "/bag_tf")

        self.tf_pub = rospy.Publisher("/tf", TFMessage, queue_size=100)

        rospy.Subscriber(bag_tf_topic, TFMessage, self.handle_tf, queue_size=100)

        rospy.loginfo(
            "Replay TF bridge forwarding %s -> %s from %s",
            self.world_frame,
            self.root_frame,
            bag_tf_topic,
        )

    def is_root_transform(self, transform):
        return (
            normalize_frame(transform.header.frame_id) == self.world_frame
            and normalize_frame(transform.child_frame_id) == self.root_frame
        )

    def handle_tf(self, msg):
        root_transforms = [transform for transform in msg.transforms if self.is_root_transform(transform)]
        if root_transforms:
            self.tf_pub.publish(TFMessage(root_transforms))


if __name__ == "__main__":
    rospy.init_node("replay_tf_bridge")
    ReplayTfBridge()
    rospy.spin()
