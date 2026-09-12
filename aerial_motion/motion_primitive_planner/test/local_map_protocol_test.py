#!/usr/bin/env python3
"""Exercise the planner's real callback and map worker with standalone snapshots."""
import copy
import time
import unittest
import rospy
import rostest
from std_msgs.msg import Header
from rog_map_msgs.msg import LocalMap

class LocalMapProtocolTest(unittest.TestCase):
    def test_versions_invalid_messages_and_restart(self):
        ready = []
        sub = rospy.Subscriber('/dragon/planning/map_ready', Header, ready.append, queue_size=100)
        pub = rospy.Publisher('/dragon/rog_map/local_map', LocalMap, queue_size=1)
        deadline = time.monotonic()+15
        while not pub.get_num_connections() and time.monotonic()<deadline:
            time.sleep(.02)
        self.assertTrue(pub.get_num_connections())
        time.sleep(.2)
        self.assertFalse(ready)  # No synthetic empty scene is published on startup.
        message = LocalMap()
        message.header.frame_id = 'world'
        message.epoch = 100
        message.version = 1
        message.resolution = .1
        message.size = [81,81,81]
        message.inflation_steps = 2
        message.occupied_bits = [0]*((81**3+7)//8)
        message.inflated_bits = list(message.occupied_bits)
        serial = 0
        def send(m, accept):
            nonlocal serial
            serial += 1
            m.header.stamp = rospy.Time(serial)
            count = len(ready)
            pub.publish(m)
            deadline = time.monotonic()+(3 if accept else .15)
            while len(ready)==count and time.monotonic()<deadline:
                time.sleep(.005)
            if accept:
                self.assertEqual(len(ready),count+1)
                self.assertEqual(ready[-1].stamp,m.header.stamp)
            else:
                self.assertEqual(len(ready),count)
        send(message,True)
        send(message,False)  # Duplicate.
        message.version = 10
        send(message,True)  # Intermediate snapshots may be lost.
        message.version = 9
        send(message,False)
        bad = copy.deepcopy(message)
        bad.epoch = 1000
        bad.occupied_bits.pop()
        send(bad,False)
        message.version = 11
        send(message,True)  # Malformed future epochs cannot poison reception.
        message.epoch = 101
        message.version = 1
        send(message,True)  # A restarted mapper begins version 1.
        message.epoch = 100
        message.version = 999
        send(message,False)
        message.epoch = 101
        message.version = 2
        send(message,True)

if __name__ == '__main__':
    rospy.init_node('local_map_protocol_test')
    rostest.rosrun('motion_primitive_planner','local_map_protocol_test',LocalMapProtocolTest)
