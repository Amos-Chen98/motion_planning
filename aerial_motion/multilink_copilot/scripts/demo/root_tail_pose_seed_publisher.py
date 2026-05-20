#!/usr/bin/env python3

import rospy
import tf.transformations as tf_trans
from geometry_msgs.msg import PoseStamped


class RootTailPoseSeedPublisher:
    NODE_NAME = "root_tail_pose_seed_publisher"

    def __init__(self):
        rospy.init_node(self.NODE_NAME, anonymous=False)

        self.topic = rospy.get_param("~topic", "root/tail_pose")
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 10.0))
        if self.publish_rate_hz <= 0.0:
            raise ValueError("~publish_rate_hz must be greater than 0.")

        self.pose_msg = PoseStamped()
        self.pose_msg.header.frame_id = self.frame_id
        self.pose_msg.pose.position.x = float(rospy.get_param("~x", 0.0))
        self.pose_msg.pose.position.y = float(rospy.get_param("~y", 0.0))
        self.pose_msg.pose.position.z = float(rospy.get_param("~z", 1.0))

        roll = float(rospy.get_param("~roll", 0.0))
        pitch = float(rospy.get_param("~pitch", 0.0))
        yaw = float(rospy.get_param("~yaw", 0.0))
        quaternion = tf_trans.quaternion_from_euler(roll, pitch, yaw)
        self.pose_msg.pose.orientation.x = quaternion[0]
        self.pose_msg.pose.orientation.y = quaternion[1]
        self.pose_msg.pose.orientation.z = quaternion[2]
        self.pose_msg.pose.orientation.w = quaternion[3]

        self.publisher = rospy.Publisher(self.topic, PoseStamped, queue_size=1, latch=True)
        rospy.loginfo(
            "Publishing root-link tail seed on %s in frame '%s' at %.1f Hz.",
            self.publisher.resolved_name,
            self.frame_id,
            self.publish_rate_hz,
        )

    def run(self):
        rate = rospy.Rate(self.publish_rate_hz)
        while not rospy.is_shutdown():
            self.pose_msg.header.stamp = rospy.Time.now()
            self.publisher.publish(self.pose_msg)
            rate.sleep()


if __name__ == "__main__":
    try:
        RootTailPoseSeedPublisher().run()
    except rospy.ROSInterruptException:
        pass
