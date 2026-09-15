#!/usr/bin/env python3
"""Manually started, one-shot flight-state guard for the whole-body planner."""

import sys
import threading
from xmlrpc.client import ServerProxy, Transport

import rosgraph
import rospy
from std_msgs.msg import UInt8


HOVER_STATE = 5
RPC_TIMEOUT = 3.0  # Wall-clock seconds per master/node request.


class TimeoutTransport(Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = RPC_TIMEOUT
        return connection

    def request(self, host, handler, request_body, verbose=False):
        # shutdown has side effects: do not retry after a lost response.
        return self.single_request(host, handler, request_body, verbose)


def shutdown_planner(planner_node, state):
    """Request ROS shutdown; a successful reply is not proof of process exit."""
    caller = rospy.get_name()
    with ServerProxy(rosgraph.get_master_uri(), transport=TimeoutTransport()) as master:
        code, message, uri = master.lookupNode(caller, planner_node)
    if code != 1:
        raise RuntimeError("Cannot find {}: {}".format(planner_node, message))

    reason = "Flight state {} != HOVER_STATE (5)".format(state)
    with ServerProxy(uri, transport=TimeoutTransport()) as planner:
        code, message, _ = planner.shutdown(caller, reason)
    if code != 1:
        raise RuntimeError("Shutdown rejected by {}: {}".format(planner_node, message))


class StopMonitor:
    def __init__(self):
        self.planner_node = rospy.resolve_name(rospy.get_param(
            "~planner_node", "whole_body_motion_primitive_planner"))
        self.triggered = threading.Event()
        self.lock = threading.Lock()
        self.state = None
        self.subscriber = rospy.Subscriber(
            "flight_state", UInt8, self.state_callback, queue_size=1, tcp_nodelay=True)

    def state_callback(self, message):
        if message.data == HOVER_STATE:
            return
        with self.lock:
            if not self.triggered.is_set():
                self.state = message.data
                self.triggered.set()

    def run(self):
        rospy.loginfo("Stop monitor armed: %s; target=%s; HOVER_STATE=5.",
                      rospy.resolve_name("flight_state"), self.planner_node)
        while not rospy.is_shutdown():
            if self.triggered.wait(0.1):
                break
        if rospy.is_shutdown():
            return 0

        rospy.logwarn("Flight state %d != 5; requesting ROS shutdown of %s.",
                      self.state, self.planner_node)
        try:
            shutdown_planner(self.planner_node, self.state)
        except Exception as error:
            rospy.logerr("Stop request for %s failed: %s. Check the target and "
                         "restart this monitor manually.", self.planner_node, error)
            return 1
        rospy.loginfo("%s accepted shutdown for flight state %d; monitor exiting. "
                      "The planner may still be cleaning up its planning threads.",
                      self.planner_node, self.state)
        return 0


def main():
    rospy.init_node("whole_body_planner_stop_monitor")
    try:
        return StopMonitor().run()
    finally:
        rospy.signal_shutdown("Stop monitor finished")


if __name__ == "__main__":
    sys.exit(main())
