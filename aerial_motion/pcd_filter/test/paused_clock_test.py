#!/usr/bin/env python3
import unittest

import rospy
import rostest
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus


class PausedClockStartup(unittest.TestCase):
    def test_initializes_before_bag_clock_starts(self):
        rospy.init_node("pcd_filter_paused_clock_test")
        self.assertEqual(rospy.Time.now(), rospy.Time(0))
        message = rospy.wait_for_message(
            "/paused/pcd_self_filter/diagnostics", DiagnosticArray, timeout=5.0)
        self.assertEqual(len(message.status), 1)
        self.assertEqual(message.status[0].level, DiagnosticStatus.WARN)
        self.assertEqual(message.status[0].message, "Waiting for first valid observation")
        self.assertEqual(rospy.Time.now(), rospy.Time(0))


if __name__ == "__main__":
    rostest.rosrun("pcd_filter", "pcd_filter_paused_clock", PausedClockStartup)
