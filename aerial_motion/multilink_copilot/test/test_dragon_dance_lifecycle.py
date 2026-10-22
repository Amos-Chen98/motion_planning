#!/usr/bin/env python3
"""Supervisor checks with real ROS messages and a deterministic, masterless transport."""

import copy
from contextlib import ExitStack
import math
import os
import sys
import threading
import unittest
from unittest import mock

import rospy
import yaml
from aerial_robot_msgs.msg import FlightNav, FullStateTarget
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt8

PACKAGE = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(PACKAGE, "scripts", "demo"))
import dragon_dance_node as dance


class Endpoint:
    def __init__(self, name, *_args, **kwargs):
        self.resolved_name = "/dragon/" + name.lstrip("~")
        self.messages = []
        self.connections = 1
        self.latched = kwargs.get("latch", False)

    def publish(self, message):
        self.messages.append(copy.deepcopy(message))

    def get_num_connections(self):
        return self.connections


class DanceLifecycleTest(unittest.TestCase):
    def setUp(self):
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        self.ros = 100.0
        self.wall = 1000.0
        self.params = {"keyboard_enabled": False}
        with open(os.path.join(PACKAGE, "config", "dragon_dance.yaml")) as stream:
            self.defaults = yaml.safe_load(stream)
        self.robot = "<robot name='test'>" + "".join(
            '<joint name="%s"><origin xyz="0.54 0 0"/><limit lower="-%s" upper="%s"/></joint>'
            % (name, math.pi, math.pi) for name in self.defaults["shutdown_joint_names"]
        ) + "</robot>"
        self.eager_callbacks = False
        self.endpoints = {}
        self.patch(dance.rospy, "init_node")
        self.patch(dance.rospy, "get_param", side_effect=self.get_param)
        self.patch(dance.rospkg.RosPack, "get_path", return_value=PACKAGE)
        self.patch(dance.rospy.Time, "now", side_effect=lambda: rospy.Time.from_sec(self.ros))
        self.patch(dance.time, "monotonic", side_effect=lambda: self.wall)
        self.patch(dance.rospy, "Publisher", side_effect=self.publisher)
        self.patch(dance.rospy, "Subscriber", side_effect=self.subscriber)
        self.patch(dance.rospy, "Service", side_effect=Endpoint)
        self.patch(dance.rospy, "resolve_name", side_effect=self.resolve_name)
        self.patch(dance.rospy, "on_shutdown")
        self.patch(dance.rospy, "is_shutdown", return_value=False)
        for method in ("loginfo", "logwarn", "logerr", "loginfo_throttle"):
            self.patch(dance.rospy, method)

    def patch(self, owner, name, **kwargs):
        return self.stack.enter_context(mock.patch.object(owner, name, **kwargs))

    @staticmethod
    def resolve_name(name):
        return ("/dragon/dragon_dance/" + name[1:]) if name.startswith("~") else "/dragon/" + name

    def get_param(self, name, default=None):
        return self.params if name == "~" else self.robot if name == "robot_description" else default

    def publisher(self, name, *args, **kwargs):
        endpoint = Endpoint(name, *args, **kwargs)
        self.endpoints[name] = endpoint
        return endpoint

    def subscriber(self, name, _message_type, callback, **kwargs):
        if self.eager_callbacks and name == "root/pose":
            callback(self.pose())
        return Endpoint(name, **kwargs)

    def make_node(self, **overrides):
        self.params.update(overrides)
        return dance.DragonDancePublisher()

    def pose(self, position=(0.0, 0.0, 1.0), rpy=(0.0, 0.0, 0.0)):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.from_sec(self.ros)
        msg.header.frame_id = "world"
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = position
        # A helper instance is unnecessary for the quaternion conversion.
        cr, cp, cy = (math.cos(value / 2) for value in rpy)
        sr, sp, sy = (math.sin(value / 2) for value in rpy)
        q = msg.pose.orientation
        q.x, q.y = sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy
        q.z, q.w = cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy
        return msg

    def feed(self, node, hover=True, goal=False, joints=None, rpy=None, velocity=0.0, yaw_rate=0.0, baselink_rpy=None):
        pose = copy.deepcopy(node.startup_transition_goal) if goal else self.pose()
        pose.header.stamp = rospy.Time.from_sec(self.ros)
        if rpy is not None:
            pose.pose.orientation = self.pose(rpy=rpy).pose.orientation
        node.root_pose_callback(pose)
        odom = Odometry(header=copy.deepcopy(pose.header))
        odom.pose.pose = copy.deepcopy(pose.pose)
        odom.twist.twist.linear.x = velocity
        odom.twist.twist.angular.z = yaw_rate
        node.cog_odom_callback(odom)
        baselink_odom = copy.deepcopy(odom)
        if baselink_rpy is not None:
            baselink_odom.pose.pose.orientation = self.pose(rpy=baselink_rpy).pose.orientation
        node.baselink_odom_callback(baselink_odom)
        joint = JointState()
        joint.header.stamp = rospy.Time.from_sec(self.ros)
        joint.name = list(node.SHUTDOWN_JOINT_NAMES)
        joint.position = list(joints) if joints is not None else [0.0] * len(joint.name)
        node.joint_state_callback(joint)
        node.flight_state_callback(UInt8(data=5 if hover else 0))

    def advance(self, node, seconds, refresh=True, ros_seconds=None, **feedback):
        self.wall += seconds
        self.ros += seconds if ros_seconds is None else ros_seconds
        if refresh:
            self.feed(node, **feedback)
        with node.lock:
            node.tick()

    def enter(self, node=None):
        node = node or self.make_node()
        self.feed(node)
        node.tick()
        self.assertEqual(node.state, "ENTRY")
        return node

    def wait_planner(self, node=None):
        node = self.enter(node)
        self.advance(node, node.entry.duration + 0.01, goal=True)
        self.assertEqual(node.state, "WAIT_PLANNER")
        return node

    def planner_message(self, node, source_stamp=None):
        msg = FullStateTarget()
        msg.header.stamp = rospy.Time.from_sec(self.ros)
        msg.root_state.header.stamp = rospy.Time.from_sec(self.ros if source_stamp is None else source_stamp)
        msg.root_state.header.frame_id = "world"
        msg.root_state.pose.pose = copy.deepcopy(node.startup_transition_goal.pose)
        msg.joint_state.name = list(node.SHUTDOWN_JOINT_NAMES)
        msg.joint_state.position = [0.0] * len(msg.joint_state.name)
        return msg

    def dancing(self):
        node = self.wait_planner()
        node.planner_callback(self.planner_message(node))
        self.assertEqual(node.state, "DANCING")
        return node

    def test_callbacks_can_arrive_during_subscription(self):
        self.eager_callbacks = True
        node = self.make_node()
        self.assertIsNotNone(node.latest_root_pose)
        self.assertTrue(node.status_publisher.latched)
        self.assertIn("WAIT_INPUTS", node.status_publisher.messages[-1].data)

    def test_missing_inputs_and_model_wait_indefinitely_without_commands(self):
        model, self.robot = self.robot, None
        node = self.make_node()
        node.tick()
        self.assertIn("missing robot_description", node.detail)
        for _ in range(3):
            self.advance(node, 3600.0, refresh=False)
            self.assertEqual(node.state, "WAIT_INPUTS")
            self.assertFalse(node.nav_publisher.messages)
        self.robot = model
        self.advance(node, 0.0)
        self.assertEqual(node.state, "ENTRY")

    def test_waiting_for_takeoff_has_no_timeout(self):
        node = self.make_node()
        self.feed(node, hover=False)
        node.tick()
        for _ in range(5):
            self.advance(node, 3600.0, hover=False)
            self.assertEqual(node.state, "WAIT_HOVER")
        self.assertFalse(node.motion_started)
        self.assertFalse(node.nav_publisher.messages)
        self.advance(node, 0.0)
        self.assertEqual(node.state, "ENTRY")

    def test_entry_starts_on_first_qualifying_speed_measurement(self):
        for axis in ("velocity", "yaw_rate"):
            for speed in (-0.31, -0.27, -0.04, 0.04, 0.27, 0.31):
                with self.subTest(axis=axis, speed=speed):
                    node = self.make_node()
                    self.feed(node, **{axis: speed})
                    node.tick()
                    self.advance(node, 3600.0, **{axis: speed})
                    self.assertEqual(node.state, "WAIT_HOVER")
                    self.assertFalse(node.nav_publisher.messages)
                    self.advance(node, 0.0, **{axis: math.copysign(0.03, speed)})
                    self.assertEqual(node.state, "ENTRY")
                    msg = node.nav_publisher.messages[-1]
                    self.assertEqual(msg.pos_xy_nav_mode, FlightNav.POS_VEL_MODE)
                    self.assertEqual(msg.pos_z_nav_mode, FlightNav.POS_VEL_MODE)
                    self.assertEqual(msg.yaw_nav_mode, FlightNav.POS_VEL_MODE)
                    self.assertAlmostEqual(msg.header.stamp.to_sec(), self.ros)

    def test_entry_uses_current_velocity_without_filter_delay(self):
        node = self.make_node()
        self.feed(node, velocity=0.08, yaw_rate=0.08)
        node.tick()
        self.assertEqual(node.state, "WAIT_HOVER")
        self.advance(node, 0.0, velocity=-0.02, yaw_rate=0.025)
        self.assertEqual(node.state, "ENTRY")
        self.assertAlmostEqual(node.entry.sample(0.0)[1][0], -0.02)
        self.assertAlmostEqual(node.entry.sample(0.0)[3], 0.025)
        self.assertAlmostEqual(node.nav_publisher.messages[-1].target_vel_x, -0.02)


    def test_entry_requires_reference_end_and_immediate_measured_convergence(self):
        node = self.enter()
        self.advance(node, node.entry.duration / 2, goal=True)
        self.assertEqual(node.state, "ENTRY")
        remaining = node.entry.duration - (self.ros - node.state_ros)
        self.advance(node, remaining + 0.01)
        self.assertEqual(node.state, "ENTRY")
        self.advance(node, 0.0, goal=True)
        self.assertEqual(node.state, "WAIT_PLANNER")
        self.assertFalse(node.full_state_publisher.messages)
        self.assertTrue(node.root_target_publisher.messages)

    def test_entry_completion_uses_pose_tolerance_without_startup_rates(self):
        node = self.enter()
        hover = dict(goal=True, velocity=0.04, yaw_rate=0.04)
        self.advance(node, node.entry.duration / 2, **hover)
        self.assertTrue(node.entry_converged())
        self.assertFalse(node.is_stationary())
        self.assertEqual(node.state, "ENTRY")
        remaining = node.entry.duration - (self.ros - node.state_ros)
        self.advance(node, remaining + 0.01, **hover)
        self.assertFalse(node.is_stationary())
        self.assertEqual(node.state, "WAIT_PLANNER")

    def test_entry_holds_endpoint_indefinitely_until_pose_converges(self):
        node = self.enter()
        for _ in range(3):
            self.advance(node, node.entry.duration + 3600.0)
            self.assertEqual(node.state, "ENTRY")
            target = node.nav_publisher.messages[-1]
            self.assertAlmostEqual(target.target_pos_x, node.entry.goal_position[0])
            self.assertEqual(target.target_vel_x, 0.0)
            self.assertFalse(node.root_target_publisher.messages)
        self.advance(node, 0.0, goal=True)
        self.assertEqual(node.state, "WAIT_PLANNER")

    def test_slow_clock_entry_has_no_wall_deadline(self):
        node = self.enter()
        self.advance(node, 3600.0, ros_seconds=node.entry.duration / 2, goal=True)
        self.assertEqual(node.state, "ENTRY")
        self.advance(node, 3600.0, ros_seconds=node.entry.duration / 2 + 0.01, goal=True)
        self.assertEqual(node.state, "WAIT_PLANNER")

    def test_flu_conversion_rotates_about_local_z_with_nonzero_pitch(self):
        node = self.make_node()
        node.resolve_robot_model()
        pitch, yaw = 0.4, 0.7
        tail = self.pose(position=(1.0, 2.0, 3.0), rpy=(0.0, pitch, yaw))
        root = node.convert_flu_tail_pose_to_root_pose(tail)
        rotation = node.orientation_to_tuple(root.pose.orientation)
        tail_forward = (math.cos(yaw) * math.cos(pitch), math.sin(yaw) * math.cos(pitch), -math.sin(pitch))
        tail_up = (math.cos(yaw) * math.sin(pitch), math.sin(yaw) * math.sin(pitch), math.cos(pitch))
        for actual, expected in zip(node.rotate_vector(rotation, (1.0, 0.0, 0.0)), tail_forward):
            self.assertAlmostEqual(actual, -expected)
        for actual, expected in zip(node.rotate_vector(rotation, (0.0, 0.0, 1.0)), tail_up):
            self.assertAlmostEqual(actual, expected)
        for actual, origin, forward in zip(node.position_to_tuple(root.pose.position), (1.0, 2.0, 3.0), tail_forward):
            self.assertAlmostEqual(actual, origin + node.root_link_length * forward)

    def test_already_at_entry_skips_trajectory_immediately(self):
        node = self.make_node()
        node.resolve_robot_model()
        node.startup_transition_goal = node.convert_flu_tail_pose_to_root_pose(node.startup_tail_pose_flu)
        self.feed(node, goal=True)
        node.tick()
        self.assertEqual(node.state, "WAIT_PLANNER")
        self.assertIsNone(node.entry)

    def test_planner_checks_original_stamp_and_message_validity(self):
        node = self.wait_planner()
        for stamp in (node.planning_started_ros - 0.1, self.ros + 1.0):
            node.planner_callback(self.planner_message(node, source_stamp=stamp))
            self.assertEqual(node.state, "WAIT_PLANNER")
        invalid = self.planner_message(node)
        invalid.joint_state.position.pop()
        node.planner_callback(invalid)
        invalid = self.planner_message(node)
        invalid.root_state.twist.twist.linear.x = float("nan")
        node.planner_callback(invalid)
        self.assertFalse(node.full_state_publisher.messages)
        node.planner_callback(self.planner_message(node))
        self.assertEqual(node.state, "DANCING")
        self.assertEqual(len(node.full_state_publisher.messages), 1)

    def test_planner_output_gap_does_not_exit_or_repeat_cached_output(self):
        node = self.dancing()
        cached = self.planner_message(node)
        for _ in range(3):
            self.advance(node, 3600.0, goal=True)
            cached.header.stamp = rospy.Time.from_sec(self.ros)
            node.planner_callback(cached)
            self.assertEqual(node.state, "DANCING")
            self.assertEqual(len(node.full_state_publisher.messages), 1)
        node.planner_callback(self.planner_message(node))
        self.assertEqual(len(node.full_state_publisher.messages), 2)

    def test_nonincreasing_planner_source_stamps_are_not_forwarded(self):
        node = self.dancing()
        self.advance(node, 0.2, goal=True)
        node.planner_callback(self.planner_message(node))
        accepted_stamp = node.last_planner_source_stamp
        self.assertEqual(len(node.full_state_publisher.messages), 2)
        for source_stamp in (accepted_stamp, accepted_stamp - 0.05):
            self.advance(node, 0.1, goal=True)
            node.planner_callback(self.planner_message(node, source_stamp=source_stamp))
            self.assertEqual(len(node.full_state_publisher.messages), 2)
            self.assertEqual(node.last_planner_source_stamp, accepted_stamp)
        node.planner_callback(self.planner_message(node))
        self.assertEqual(len(node.full_state_publisher.messages), 3)
        self.assertGreater(node.last_planner_source_stamp, accepted_stamp)

    def test_extra_feedback_joints_allowed_but_planner_joint_count_is_exact(self):
        node = self.wait_planner()
        feedback = copy.deepcopy(node.latest_joint_state)
        feedback.name.append("gimbal1_yaw")
        feedback.position.append(0.0)
        node.joint_state_callback(feedback)
        self.assertIn("gimbal1_yaw", node.latest_joint_state.name)
        self.assertFalse(node.input_problems(self.ros))
        output = self.planner_message(node)
        output.joint_state = copy.deepcopy(feedback)
        node.planner_callback(output)
        self.assertEqual(node.state, "WAIT_PLANNER")
        self.assertFalse(node.full_state_publisher.messages)
        self.assertIsNone(node.last_planner_source_stamp)
        node.planner_callback(self.planner_message(node))
        self.assertEqual(node.state, "DANCING")
        self.assertEqual(len(node.full_state_publisher.messages), 1)

    def test_first_planner_output_waits_indefinitely_and_accepts_delayed_result(self):
        node = self.wait_planner()
        requested_stamp = self.ros
        for _ in range(3):
            self.advance(node, 3600.0, goal=True)
            self.assertEqual(node.state, "WAIT_PLANNER")
            self.assertFalse(node.full_state_publisher.messages)
            self.assertEqual(node.nav_publisher.messages[-1].target_vel_x, 0.0)
        node.planner_callback(self.planner_message(node, source_stamp=requested_stamp))
        self.assertEqual(node.state, "DANCING")

    def test_two_cycles_preserve_entry_pose_and_continue_forwarding(self):
        node = self.dancing()
        for _ in range(int(2 * node.total_duration)):
            self.advance(node, 1.0, goal=True)
            node.planner_callback(self.planner_message(node))
        # Move just beyond the boundary to avoid ROS nanosecond quantization.
        self.advance(node, 1e-6, goal=True)
        self.assertEqual(node.state, "DANCING")
        self.assertEqual(node.completed_cycles, 2)
        final = node.root_target_publisher.messages[-1].pose
        entry = node.startup_tail_pose_flu.pose
        for component in ("x", "y", "z"):
            self.assertAlmostEqual(getattr(final.position, component), getattr(entry.position, component), places=6)
        self.assertEqual(len(node.full_state_publisher.messages), int(2 * node.total_duration) + 1)

    def test_stops_before_motion_and_repeated_stops_are_idempotent(self):
        node = self.make_node()
        self.assertTrue(node.stop_callback(None).success)
        self.assertEqual(node.state, "DONE")
        self.assertTrue(node.stop_callback(None).success)
        self.assertFalse(node.cog_target_publisher.messages)

    def test_stop_during_entry_cancels_nav_and_starts_recovery(self):
        node = self.enter()
        before = len(node.nav_publisher.messages)
        self.assertTrue(node.stop_callback(None).success)
        self.assertEqual(node.state, "STOP_WAIT")
        self.advance(node, 0.0)
        self.assertEqual(node.state, "SHAPING")
        shape_started = node.state_wall
        self.assertTrue(node.stop_callback(None).success)
        self.assertEqual(node.state_wall, shape_started)
        self.assertEqual(len(node.nav_publisher.messages), before)
        self.assertEqual(len(node.cog_target_publisher.messages), 1)

    def test_stop_while_waiting_for_planner_never_forwards_first_output(self):
        node = self.wait_planner()
        self.assertTrue(node.stop_callback(None).success)
        node.planner_callback(self.planner_message(node))
        self.assertEqual(node.state, "STOP_WAIT")
        self.assertFalse(node.full_state_publisher.messages)

    def test_stop_gates_pending_concurrent_planner_callbacks(self):
        node = self.dancing()
        message = self.planner_message(node)
        started = threading.Event()

        def queued_callback():
            started.set()
            node.planner_callback(message)

        with node.lock:
            thread = threading.Thread(target=queued_callback)
            thread.start()
            self.assertTrue(started.wait(1.0))
            self.assertTrue(node.stop_callback(None).success)
        thread.join(1.0)
        self.assertFalse(thread.is_alive())
        node.planner_callback(message)
        self.assertEqual(node.state, "STOP_WAIT")
        self.assertEqual(len(node.full_state_publisher.messages), 1)

    def test_hover_loss_closes_forwarding_immediately(self):
        node = self.dancing()
        node.flight_state_callback(UInt8(data=4))
        self.assertEqual(node.state, "ERROR")
        node.planner_callback(self.planner_message(node))
        self.assertEqual(len(node.full_state_publisher.messages), 1)
        self.assertFalse(node.stop_callback(None).success)

    def test_feedback_age_alone_does_not_exit(self):
        node = self.enter()
        self.advance(node, 3600.0, refresh=False)
        self.assertEqual(node.state, "ENTRY")
        self.assertFalse(node.input_problems(self.ros))
        self.advance(node, 0.0, goal=True)
        self.assertEqual(node.state, "WAIT_PLANNER")

    def test_explicit_invalid_feedback_stops_commands(self):
        node = self.enter()
        before = len(node.nav_publisher.messages)
        invalid = self.pose(position=(float("nan"), 0.0, 1.0))
        node.root_pose_callback(invalid)
        node.tick()
        self.assertEqual(node.state, "ERROR")
        self.assertIn("invalid", node.detail)
        self.assertEqual(len(node.nav_publisher.messages), before)

    def test_paused_clock_waits_indefinitely_and_stop_stays_available(self):
        node = self.enter()
        before = node.nav_publisher.messages[-1]
        for _ in range(3):
            self.advance(node, 3600.0, ros_seconds=0.0)
            self.assertEqual(node.state, "ENTRY")
            self.assertEqual(node.nav_publisher.messages[-1], before)
        self.assertTrue(node.stop_callback(None).success)
        self.assertEqual(node.state, "STOP_WAIT")

    def test_backwards_clock_is_terminal(self):
        node = self.enter()
        self.advance(node, 0.1, ros_seconds=-0.1)
        self.assertEqual(node.state, "ERROR")
        self.assertIn("ROS time moved backwards", node.detail)

    def test_connection_loss_stops_active_entry(self):
        node = self.enter()
        node.nav_publisher.connections = 0
        self.advance(node, 0.1)
        self.assertEqual(node.state, "ERROR")
        self.assertIn("no subscriber", node.detail)

    def test_joint_recovery_waits_indefinitely_then_levels_on_first_in_tolerance_update(self):
        node = self.enter()
        node.stop_callback(None)
        self.advance(node, 0.0)
        self.assertEqual(node.state, "SHAPING")
        for _ in range(3):
            self.advance(node, 3600.0)
            self.assertEqual(node.state, "SHAPING")
            self.assertFalse(node.rotation_target_publisher.messages)
        outside = [target - 1.01 * node.shutdown_joint_tolerance for target in node.SHUTDOWN_JOINT_POSITIONS]
        self.advance(node, 3600.0, joints=outside)
        self.assertEqual(node.state, "SHAPING")
        inside = [target - 0.99 * node.shutdown_joint_tolerance for target in node.SHUTDOWN_JOINT_POSITIONS]
        self.advance(node, 0.0, joints=inside)
        self.assertEqual(node.state, "LEVELING")
        self.assertTrue(node.rotation_target_publisher.messages)

    def test_stop_wait_uses_measured_speed_and_no_fixed_duration(self):
        node = self.enter()
        self.feed(node, velocity=0.04)
        node.stop_callback(None)
        for feedback in ({"velocity": 0.04}, {"yaw_rate": 0.04}):
            self.advance(node, 3600.0, **feedback)
            self.assertEqual(node.state, "STOP_WAIT")
            self.assertFalse(node.joint_target_publisher.messages)
        self.advance(node, 0.0)
        self.assertEqual(node.state, "SHAPING")
        self.assertTrue(node.joint_target_publisher.messages)


    def test_leveling_waits_for_reference_and_feedback_without_deadline_or_dwell(self):
        node = self.enter()
        node.stop_callback(None)
        recovery = dict(joints=node.SHUTDOWN_JOINT_POSITIONS, rpy=(0.3, -0.2, 0.7))
        self.advance(node, 0.0, **recovery)
        self.assertEqual(node.state, "LEVELING")
        self.assertEqual(len(node.cog_target_publisher.messages), 2)
        self.assertTrue(node.stop_callback(None).success)
        # Pose tolerance cannot complete leveling before its reference finishes.
        aligned = dict(joints=node.SHUTDOWN_JOINT_POSITIONS, rpy=(0.0, 0.0, 0.7))
        self.advance(node, node.shutdown_leveling_publish_duration / 2, **aligned)
        self.assertEqual(node.state, "LEVELING")
        for _ in range(3):
            self.advance(node, 3600.0, **recovery)
            self.assertEqual(node.state, "LEVELING")
            self.assertEqual(len(node.cog_target_publisher.messages), 2)
            roll, pitch, yaw = node.rpy_from_orientation(node.rotation_target_publisher.messages[-1].pose.pose.orientation)
            self.assertAlmostEqual(roll, 0.0)
            self.assertAlmostEqual(pitch, 0.0)
            self.assertAlmostEqual(yaw, 0.7)
        self.advance(node, 0.0, **aligned)
        self.assertEqual(node.state, "DONE")
        self.assertIn("Land", node.detail)

    def test_leveling_controls_and_checks_physical_baselink_instead_of_cog(self):
        node = self.enter()
        self.feed(node, joints=node.SHUTDOWN_JOINT_POSITIONS,
                  rpy=(0.0, 0.0, -0.2), baselink_rpy=(0.4, -0.3, 1.2))
        node.stop_callback(None)
        node.tick()
        self.assertEqual(node.state, "LEVELING")
        msg = node.rotation_target_publisher.messages[-1]
        self.assertEqual(msg.header.frame_id, "baselink")
        for actual, expected in zip(node.rpy_from_orientation(msg.pose.pose.orientation), (0.4, -0.3, 1.2)):
            self.assertAlmostEqual(actual, expected)
        # Position hold still uses the CoG heading, independently of body yaw.
        self.assertAlmostEqual(node.yaw_from_orientation(node.cog_target_publisher.messages[-1].pose.orientation), -0.2)
        midpoint = node.build_shutdown_leveling_rotation_target(0.5 * node.shutdown_leveling_publish_duration)
        for actual, expected in zip(node.rpy_from_orientation(midpoint.pose.pose.orientation), (0.2, -0.15, 1.2)):
            self.assertAlmostEqual(actual, expected)
        self.advance(node, node.shutdown_leveling_publish_duration + 0.01,
                     joints=node.SHUTDOWN_JOINT_POSITIONS, rpy=(0.0, 0.0, 0.0), baselink_rpy=(0.4, -0.3, 1.2))
        self.assertEqual(node.state, "LEVELING", "A level CoG must not hide a tilted body")
        endpoint = node.rotation_target_publisher.messages[-1]
        for actual, expected in zip(node.rpy_from_orientation(endpoint.pose.pose.orientation), (0.0, 0.0, 1.2)):
            self.assertAlmostEqual(actual, expected)
        # Only physical-body feedback determines the leveling error.
        self.advance(node, 0.0, joints=node.SHUTDOWN_JOINT_POSITIONS,
                     rpy=(0.25, -0.2, 0.0), baselink_rpy=(0.0, 0.0, 1.2))
        self.assertEqual(node.state, "DONE")

    def test_leveling_maintains_and_rechecks_joint_shape_until_complete(self):
        node = self.enter()
        node.stop_callback(None)
        self.advance(node, 0.0, joints=node.SHUTDOWN_JOINT_POSITIONS)
        self.assertEqual(node.state, "LEVELING")
        displaced = list(node.SHUTDOWN_JOINT_POSITIONS)
        displaced[0] = 0.2
        count = len(node.joint_target_publisher.messages)
        self.advance(node, 3600.0, joints=displaced)
        self.assertEqual(node.state, "LEVELING")
        self.assertGreater(len(node.joint_target_publisher.messages), count)
        self.assertEqual(list(node.joint_target_publisher.messages[-1].position), node.SHUTDOWN_JOINT_POSITIONS)
        self.advance(node, 0.0, joints=node.SHUTDOWN_JOINT_POSITIONS)
        self.assertEqual(node.state, "DONE")

    def test_baselink_feedback_is_required_and_invalid_attitude_stops_commands(self):
        node = self.make_node()
        self.feed(node)
        node.received.pop(node.BASELINK_ODOM_TOPIC)
        node.latest_baselink_pose = None
        self.advance(node, 3600.0, refresh=False)
        self.assertEqual(node.state, "WAIT_INPUTS")
        self.assertIn("uav/baselink/odom", node.detail)
        self.assertFalse(node.nav_publisher.messages)
        self.advance(node, 0.0)
        self.assertEqual(node.state, "ENTRY")
        count = len(node.nav_publisher.messages)
        invalid = Odometry(header=self.pose().header)
        node.baselink_odom_callback(invalid)
        node.tick()
        self.assertEqual(node.state, "ERROR")
        self.assertIn("uav/baselink/odom", node.detail)
        self.assertEqual(len(node.nav_publisher.messages), count)

    def test_nonplanar_shutdown_shape_is_rejected(self):
        positions = list(self.defaults["shutdown_joint_positions"])
        positions[0] = 0.1
        with self.assertRaisesRegex(ValueError, "pitch joints to zero"):
            self.make_node(shutdown_joint_positions=positions)
        self.assertFalse(self.endpoints)

    def test_hover_loss_still_aborts_recovery(self):
        node = self.enter()
        node.stop_callback(None)
        before = len(node.cog_target_publisher.messages)
        self.advance(node, 0.1, hover=False)
        self.assertEqual(node.state, "ERROR")
        self.assertEqual(len(node.cog_target_publisher.messages), before)

    def test_paused_clock_recovery_has_no_wall_deadline_and_publishes_each_hold_once(self):
        node = self.enter()
        node.stop_callback(None)
        self.assertEqual(len(node.cog_target_publisher.messages), 1)
        self.advance(node, 0.0, joints=node.SHUTDOWN_JOINT_POSITIONS)
        self.assertEqual(node.state, "LEVELING")
        for _ in range(3):
            self.advance(node, 3600.0, ros_seconds=0.0, joints=node.SHUTDOWN_JOINT_POSITIONS)
            self.assertEqual(node.state, "LEVELING")
            self.assertEqual(len(node.cog_target_publisher.messages), 2)
        self.advance(node, node.shutdown_leveling_publish_duration + 0.01, joints=node.SHUTDOWN_JOINT_POSITIONS)
        self.assertEqual(node.state, "DONE")

    def test_no_tty_and_q_share_the_stop_service_path(self):
        stream = mock.Mock()
        stream.isatty.return_value = False
        self.patch(dance.sys, "stdin", new=stream)
        node = self.make_node(keyboard_enabled=True)
        self.assertFalse(node.keyboard_enabled)
        warnings = " ".join(str(call) for call in dance.rospy.logwarn.call_args_list)
        self.assertIn("/dragon/dragon_dance/stop", warnings)
        node.keyboard_enabled = True
        stream.read.return_value = "q"
        self.patch(dance.select, "select", return_value=([stream], [], []))
        self.assertEqual(node.run(), 0)
        self.assertEqual(node.state, "DONE")

    def test_terminal_is_restored_once_on_shutdown(self):
        stream = mock.Mock()
        stream.isatty.return_value = True
        stream.fileno.return_value = 12
        self.patch(dance.sys, "stdin", new=stream)
        settings = [1, 2, 3]
        self.patch(dance.termios, "tcgetattr", return_value=settings)
        self.patch(dance.tty, "setcbreak")
        restore = self.patch(dance.termios, "tcsetattr")
        node = self.make_node(keyboard_enabled=True)
        self.assertTrue(node.keyboard_enabled)
        node.on_ros_shutdown()
        node.on_ros_shutdown()
        restore.assert_called_once_with(12, dance.termios.TCSADRAIN, settings)
        self.assertFalse(node.keyboard_enabled)

    def test_keyboard_eof_restores_terminal_and_keeps_service_available(self):
        node = self.make_node()
        stream = mock.Mock()
        stream.read.return_value = ""
        self.patch(dance.sys, "stdin", new=stream)
        self.patch(dance.select, "select", return_value=([stream], [], []))
        restore = self.patch(dance.termios, "tcsetattr")
        node.keyboard_enabled, node.stdin_fd, node.stdin_settings = True, 12, [1, 2, 3]
        self.assertFalse(node.key_pressed())
        self.assertFalse(node.keyboard_enabled)
        restore.assert_called_once()
        self.assertTrue(node.stop_callback(None).success)

    def test_invalid_config_is_rejected_before_registering_interfaces(self):
        for key, value in (("entry_max_linear_speed", 0.0), ("entry_max_yaw_rate", float("nan")),
                           ("publish_rate_hz", True), ("z_cycles", 0.5),
                           ("shutdown_joint_positions", []), ("frame_id", "map")):
            with self.subTest(key=key):
                self.params = {"keyboard_enabled": False, key: value}
                with self.assertRaisesRegex(ValueError, key):
                    self.make_node()
                self.assertFalse(self.endpoints)

    def test_unknown_config_keys_are_rejected_before_interface_registration(self):
        for name in ("entry_max_linear_spped", "tick", "state"):
            with self.subTest(name=name):
                self.params = {"keyboard_enabled": False, name: 0.3}
                with self.assertRaisesRegex(ValueError, "Unknown demo parameters: " + name):
                    self.make_node()
                self.assertFalse(self.endpoints)

    def test_removed_timing_parameters_are_absent_and_ignored_with_warning(self):
        for name in dance.REMOVED_TIMING_PARAMETERS:
            self.assertNotIn(name, self.defaults)
        node = self.make_node(**{name: 0.001 for name in dance.REMOVED_TIMING_PARAMETERS})
        for name in dance.REMOVED_TIMING_PARAMETERS:
            self.assertNotIn(name, node.config)
            self.assertFalse(hasattr(node, name))
            self.assertTrue(any(name in str(call) for call in dance.rospy.logwarn.call_args_list))
        self.advance(node, 3600.0, refresh=False)
        self.assertEqual(node.state, "WAIT_INPUTS")
        self.advance(node, 0.0)
        self.assertEqual(node.state, "ENTRY")

    def test_malformed_robot_model_is_reported_before_movement(self):
        self.robot = "<robot><joint"
        node = self.make_node()
        self.feed(node)
        node.tick()
        self.assertEqual(node.state, "ERROR")
        self.assertIn("Invalid robot_description", node.detail)
        self.assertFalse(node.nav_publisher.messages)

    def test_obsolete_duration_warns_and_does_not_override_adaptive_entry(self):
        node = self.make_node(startup_transition_duration=1000.0)
        self.assertFalse(hasattr(node, "startup_transition_duration"))
        self.assertTrue(any("obsolete" in str(call) for call in dance.rospy.logwarn.call_args_list))
        self.enter(node)
        self.assertLess(node.entry.duration, 1000.0)


if __name__ == "__main__":
    unittest.main()
