#! /usr/bin/env python3
"""Feed the Dragon root-link state into a planner as a FLU odometry message.

The planners treat the root link as a free-flying body. They expect the
planning start in a ``nav_msgs/Odometry`` topic and treat the body +X axis as
the forward direction (Forward-Left-Up convention). Dragon publishes the
root-link state as ``geometry_msgs/PoseStamped`` on ``root/pose`` in the LINK
frame, whose +X axis points along the link toward ``joint1`` (i.e. the opposite
of the head forward direction).

This bridge therefore performs the two conversions the planners need:

1. FLU-ification: rotate the LINK-frame orientation by 180 degrees about the
   local/body Z axis so the reported heading matches the FLU convention used by
   the root planner and, downstream, by the copilot planner launched with
   ``target_pose_frame_type:=FLU``. This mirrors the inverse of the
   ``convertFluPoseToLinkFrame`` transform inside ``multilink_copilot``.
2. Topic-format processing: wrap the FLU pose into ``nav_msgs/Odometry`` (the
   message type the planners subscribe to on their ``odom`` input).

The translation is unchanged because both frames share the same origin (the
root link). Velocity is left at zero: the planners only consume the odometry
position and heading for the planning start.
"""

import rospy
import tf.transformations as tf_trans

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

# 180-degree rotation about the local Z axis expressed as [x, y, z, w].
# LINK = FLU * Rz(pi); since Rz(pi) is its own inverse, the same product also
# maps LINK -> FLU.
_LINK_FLU_Z_FLIP = [0.0, 0.0, 1.0, 0.0]


class RootPoseToFluOdom:
    def __init__(self):
        self.child_frame_id = rospy.get_param('~child_frame_id', 'dragon/root')

        self.pub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.sub = rospy.Subscriber('root/pose', PoseStamped, self.callback,
                                    tcp_nodelay=True)

    def callback(self, msg):
        q_link = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ]
        # Post-multiply to apply the flip in the body frame (LINK -> FLU).
        q_flu = tf_trans.quaternion_multiply(q_link, _LINK_FLU_Z_FLIP)

        odom = Odometry()
        odom.header = msg.header
        odom.child_frame_id = self.child_frame_id

        odom.pose.pose.position = msg.pose.position
        odom.pose.pose.orientation.x = q_flu[0]
        odom.pose.pose.orientation.y = q_flu[1]
        odom.pose.pose.orientation.z = q_flu[2]
        odom.pose.pose.orientation.w = q_flu[3]

        self.pub.publish(odom)


if __name__ == '__main__':
    rospy.init_node('root_pose_to_flu_odom')
    RootPoseToFluOdom()
    rospy.spin()
