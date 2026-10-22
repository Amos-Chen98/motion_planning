#!/usr/bin/env python3
"""Supervise the dance demo and serialize all of its robot commands."""

import copy
import math
import os
import select
import sys
import termios
import threading
import time
import tty
from xml.etree import ElementTree

import rospkg
import rospy
import yaml
from aerial_robot_msgs.msg import FlightNav, FullStateTarget
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String, UInt8
from std_srvs.srv import Trigger, TriggerResponse

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from dragon_dance_support import DragonDanceSupportMixin
from dragon_dance_trajectory import AdaptiveEntryTrajectory, yaw_rate_from_body_rates


REMOVED_TIMING_PARAMETERS = (
    "settle_duration", "entry_convergence_timeout", "input_ready_timeout",
    "feedback_timeout", "planner_output_timeout", "shutdown_joint_shape_timeout",
    "shutdown_total_timeout", "shutdown_wait_duration",
)


def validate_config(config):
    """Validate before registering callbacks or opening the terminal."""
    positive = (
        "publish_rate_hz", "total_duration", "ellipse_semi_major", "ellipse_semi_minor",
        "entry_max_linear_speed", "entry_max_yaw_rate", "startup_position_tolerance",
        "startup_yaw_tolerance", "status_period",
        "shutdown_joint_tolerance", "shutdown_cog_hold_duration",
        "shutdown_leveling_publish_duration", "shutdown_leveling_tolerance",
    )
    nonnegative = ("z_amplitude",)
    for name in positive + nonnegative + ("ellipse_center_x", "ellipse_center_y", "z_offset"):
        value = config[name]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError("%s must be a finite number" % name)
        if name in positive and value <= 0:
            raise ValueError("%s must be positive" % name)
        if name in nonnegative and value < 0:
            raise ValueError("%s must be nonnegative" % name)
    if config["total_duration"] * config["publish_rate_hz"] < 2:
        raise ValueError("total_duration must span at least two publication periods")
    cycles = config["z_cycles"]
    if isinstance(cycles, bool) or not isinstance(cycles, (int, float)) or not math.isfinite(cycles) or cycles < 0 or int(cycles) != cycles:
        raise ValueError("z_cycles must be a nonnegative integer for a continuous loop")
    if not isinstance(config["frame_id"], str) or config["frame_id"].lstrip("/") != "world":
        raise ValueError("frame_id must be world (the Dragon navigation world frame)")
    if not isinstance(config["keyboard_enabled"], bool):
        raise ValueError("keyboard_enabled must be a boolean")
    names, positions = config["shutdown_joint_names"], config["shutdown_joint_positions"]
    if not isinstance(names, list) or not names or not all(isinstance(n, str) and n for n in names) or len(set(names)) != len(names):
        raise ValueError("shutdown_joint_names must be a nonempty list of unique names")
    if not isinstance(positions, list) or len(names) != len(positions):
        raise ValueError("shutdown_joint_positions must match shutdown_joint_names")
    if any(isinstance(p, bool) or not isinstance(p, (int, float)) or not math.isfinite(p) for p in positions):
        raise ValueError("shutdown_joint_positions must contain finite numbers")
    if any(name.endswith("_pitch") and abs(position) > 1e-9 for name, position in zip(names, positions)):
        raise ValueError("shutdown_joint_positions must set pitch joints to zero for whole-body leveling")
    if "root_link_length" in config:
        length = config["root_link_length"]
        if isinstance(length, bool) or not isinstance(length, (int, float)) or not math.isfinite(length) or length <= 0:
            raise ValueError("root_link_length must be positive and finite")
    return config


class DragonDancePublisher(DragonDanceSupportMixin):
    ROOT_TARGET_TOPIC = "root/target_pose"
    COG_TARGET_TOPIC = "target_pose"
    ROTATION_TARGET_TOPIC = "target_rotation_motion"
    JOINT_CONTROL_TOPIC = "joints_ctrl"
    ROOT_POSE_TOPIC = "root/pose"
    COG_ODOM_TOPIC = "uav/cog/odom"
    BASELINK_ODOM_TOPIC = "uav/baselink/odom"
    JOINT_STATE_TOPIC = "joint_states"
    FLIGHT_STATE_TOPIC = "flight_state"
    ROTATION_FRAME_ID = "baselink"
    TAIL_FLU_TO_ROOT_YAW_OFFSET = math.pi
    HOVER_STATE = 5
    TERMINAL = ("DONE", "ERROR")
    PREPARING = ("WAIT_INPUTS", "WAIT_HOVER")
    STOPPING = ("STOP_WAIT", "SHAPING", "LEVELING")

    def __init__(self):
        rospy.init_node("dragon_dance", anonymous=False)
        package = rospkg.RosPack().get_path("multilink_copilot")
        with open(os.path.join(package, "config", "dragon_dance.yaml")) as stream:
            config = yaml.safe_load(stream)
        overrides = dict(rospy.get_param("~", {}))
        for name in REMOVED_TIMING_PARAMETERS:
            if name in overrides:
                overrides.pop(name)
                rospy.logwarn("~%s has been removed and is ignored; stages advance on their completion conditions", name)
        unknown = set(overrides) - set(config) - {"root_link_length", "startup_transition_duration"}
        if unknown:
            raise ValueError("Unknown demo parameters: " + ", ".join(sorted(unknown)))
        config.update(overrides)
        self.config = validate_config(config)
        if "startup_transition_duration" in config:
            rospy.logwarn("~startup_transition_duration is obsolete and ignored; use entry_max_linear_speed and entry_max_yaw_rate")
        for name, value in config.items():
            if name != "startup_transition_duration":
                setattr(self, name, value)
        # Geometry helpers share these names with the original demo.
        for name in ("frame_id", "publish_rate_hz", "total_duration", "ellipse_center_x", "ellipse_center_y",
                     "ellipse_semi_major", "ellipse_semi_minor", "z_offset", "z_amplitude", "z_cycles",
                     "shutdown_joint_names", "shutdown_joint_positions"):
            setattr(self, name.upper(), config[name])

        # Callback-visible state must exist before any subscription/service is registered.
        self.lock = threading.RLock()
        self.state = "WAIT_INPUTS"
        self.detail = "Waiting for robot inputs and connections"
        self.state_wall = time.monotonic()
        self.state_ros = rospy.Time.now().to_sec()
        self.last_ros = self.state_ros
        self.last_status_wall = -math.inf
        self.latest_root_pose = None
        self.latest_cog_pose = None
        self.latest_cog_odom = None
        self.latest_baselink_pose = None
        self.latest_joint_state = None
        self.latest_flight_state = None
        self.received = {}
        self.invalid_inputs = {}
        self.model_ready = False
        self.root_link_length = None
        self.entry = None
        self.startup_transition_goal = None
        self.planning_started_ros = None
        self.last_planner_source_stamp = None
        self.dance_start_ros = None
        self.completed_cycles = 0
        self.motion_started = False
        self.shutdown_leveling_start_rpy = None
        self.shutdown_hold_cog_target = None
        self.stdin_fd = None
        self.stdin_settings = None
        self.keyboard_enabled = False
        self.exiting = False
        self.startup_tail_pose_flu = self.build_tail_pose_message(0.0)
        self.shutdown_joint_target = self.build_shutdown_joint_target()

        self.root_target_publisher = rospy.Publisher(self.ROOT_TARGET_TOPIC, PoseStamped, queue_size=1)
        self.cog_target_publisher = rospy.Publisher(self.COG_TARGET_TOPIC, PoseStamped, queue_size=1)
        self.nav_publisher = rospy.Publisher("uav/nav", FlightNav, queue_size=1)
        self.rotation_target_publisher = rospy.Publisher(self.ROTATION_TARGET_TOPIC, Odometry, queue_size=1)
        self.joint_target_publisher = rospy.Publisher(self.JOINT_CONTROL_TOPIC, JointState, queue_size=1)
        self.full_state_publisher = rospy.Publisher("full_state_target", FullStateTarget, queue_size=1)
        self.status_publisher = rospy.Publisher("~status", String, queue_size=1, latch=True)
        self.subscribers = [
            rospy.Subscriber(self.ROOT_POSE_TOPIC, PoseStamped, self.root_pose_callback, queue_size=1),
            rospy.Subscriber(self.COG_ODOM_TOPIC, Odometry, self.cog_odom_callback, queue_size=1),
            rospy.Subscriber(self.BASELINK_ODOM_TOPIC, Odometry, self.baselink_odom_callback, queue_size=1),
            rospy.Subscriber(self.JOINT_STATE_TOPIC, JointState, self.joint_state_callback, queue_size=1),
            rospy.Subscriber(self.FLIGHT_STATE_TOPIC, UInt8, self.flight_state_callback, queue_size=1),
        ]
        self.planner_subscriber = rospy.Subscriber("dragon_dance/planned_full_state_target", FullStateTarget,
                                                  self.planner_callback, queue_size=1)
        self.stop_service = rospy.Service("~stop", Trigger, self.stop_callback)
        self.resolved_cog_target_topic = rospy.resolve_name(self.COG_TARGET_TOPIC)
        self.resolved_rotation_target_topic = rospy.resolve_name(self.ROTATION_TARGET_TOPIC)
        rospy.on_shutdown(self.on_ros_shutdown)
        self.keyboard_enabled = self.configure_keyboard_input() if config["keyboard_enabled"] else False
        rospy.loginfo("[Dance] Normal stop: q or rosservice call %s '{}'. Ctrl+C terminates without completing recovery.",
                      rospy.resolve_name("~stop"))
        self.report(force=True)

    def transition(self, state, detail, wall=None, ros=None):
        self.state = state
        self.detail = detail
        self.state_wall = time.monotonic() if wall is None else wall
        self.state_ros = rospy.Time.now().to_sec() if ros is None else ros
        self.report(force=True, wall=self.state_wall)

    def report(self, force=False, wall=None):
        wall = time.monotonic() if wall is None else wall
        if not force and wall - self.last_status_wall < self.status_period:
            return
        message = "[%s] %s" % (self.state, self.detail)
        self.status_publisher.publish(String(data=message))
        if force or self.state != "DANCING":
            (rospy.logerr if self.state == "ERROR" else rospy.loginfo)("[Dance] %s", message)
        self.last_status_wall = wall

    def fail(self, reason, wall=None, ros=None):
        if self.state not in self.TERMINAL:
            self.transition("ERROR", reason + "; automatic commands stopped. Land/reset through bringup before restarting the demo.", wall, ros)

    def run(self):
        try:
            while not rospy.is_shutdown():
                with self.lock:
                    if self.key_pressed():
                        self.request_stop("q")
                    self.tick()
                    if self.state in self.TERMINAL:
                        break
                # Wall sleep keeps keyboard/service handling responsive if /clock pauses.
                time.sleep(1.0 / self.publish_rate_hz)
            return 1 if self.state == "ERROR" else 0
        finally:
            self.on_ros_shutdown()

    def tick(self, ros=None, wall=None):
        ros = rospy.Time.now().to_sec() if ros is None else ros
        wall = time.monotonic() if wall is None else wall
        if self.state in self.TERMINAL or self.exiting:
            return
        if ros < self.last_ros:
            self.fail("ROS time moved backwards", wall, ros)
            return
        self.last_ros = ros
        if self.state == "WAIT_INPUTS":
            try:
                if not self.model_ready:
                    self.model_ready = self.resolve_robot_model()
            except (ValueError, ElementTree.ParseError) as exc:
                self.fail("Invalid robot_description: %s" % exc, wall, ros)
                return
            reasons = self.input_problems(ros) + self.connection_problems()
            if not self.model_ready:
                reasons.append("missing robot_description")
            if ros <= 0:
                reasons.append("waiting for ROS time")
            if reasons:
                self.detail = "; ".join(reasons)
                self.report(wall=wall)
                return
            self.transition("WAIT_HOVER", "Use bringup to take off; waiting for stable HOVER", wall, ros)

        reasons = self.input_problems(ros) + self.connection_problems()
        if reasons:
            self.fail("; ".join(reasons), wall, ros)
            return
        if self.state == "WAIT_HOVER":
            stable = self.latest_flight_state == self.HOVER_STATE and self.is_stationary()
            if stable:
                self.start_entry(ros, wall)
            else:
                self.detail = "Waiting for stable HOVER (flight_state=%s); release motion controls" % self.latest_flight_state
        elif self.latest_flight_state != self.HOVER_STATE:
            self.fail("Left HOVER (flight_state=%s)" % self.latest_flight_state, wall, ros)
            return
        elif self.state == "ENTRY":
            elapsed = ros - self.state_ros
            self.publish_entry_sample(min(elapsed, self.entry.duration))
            position_error, yaw_error = self.compute_startup_goal_error()
            self.detail = "Entry %.1f/%.1f s; position error %.3f m, yaw error %.3f rad" % (elapsed, self.entry.duration, position_error, yaw_error)
            arrived = elapsed >= self.entry.duration and self.entry_converged()
            if arrived:
                self.begin_planner(ros, wall)
        elif self.state == "WAIT_PLANNER":
            self.publish_entry_hold()
            self.root_target_publisher.publish(self.build_tail_pose_message(0.0))
        elif self.state == "DANCING":
            elapsed = ros - self.dance_start_ros
            cycles = int(elapsed / self.total_duration)
            if cycles != self.completed_cycles:
                self.completed_cycles = cycles
                rospy.loginfo("[Dance] Completed cycle %d", cycles)
            self.root_target_publisher.publish(self.build_tail_pose_message(elapsed % self.total_duration))
            self.detail = "Cycle %d, %.1f/%.1f s; q or stop service to recover" % (cycles + 1, elapsed % self.total_duration, self.total_duration)
        elif self.state in self.STOPPING:
            self.tick_shutdown(ros, wall)
        self.report(wall=wall)

    def current_velocity(self):
        # Dragon's estimator publishes world linear velocity and CoG/body
        # angular velocity in this Odometry, despite its child_frame_id.
        velocity = self.position_to_tuple(self.latest_cog_odom.twist.twist.linear)
        roll, pitch, _ = self.rpy_from_orientation(self.latest_cog_pose.pose.orientation)
        rate = yaw_rate_from_body_rates(roll, pitch, self.position_to_tuple(self.latest_cog_odom.twist.twist.angular))
        return velocity, rate

    def is_stationary(self):
        velocity, yaw_rate = self.current_velocity()
        # Check the current measurement directly, without filtering or a dwell.
        return (math.hypot(*velocity) <= 0.1 * self.entry_max_linear_speed
                and abs(yaw_rate) <= 0.1 * self.entry_max_yaw_rate)

    def entry_converged(self):
        position_error, yaw_error = self.compute_startup_goal_error()
        return position_error <= self.startup_position_tolerance and yaw_error <= self.startup_yaw_tolerance

    def start_entry(self, ros, wall):
        self.startup_transition_goal = self.build_startup_cog_target(self.latest_root_pose, self.latest_cog_pose)
        if self.entry_converged():
            self.begin_planner(ros, wall)
            return
        velocity, yaw_rate = self.current_velocity()
        self.entry = AdaptiveEntryTrajectory(
            self.position_to_tuple(self.latest_cog_pose.pose.position), velocity,
            self.yaw_from_orientation(self.latest_cog_pose.pose.orientation), yaw_rate,
            self.position_to_tuple(self.startup_transition_goal.pose.position),
            self.yaw_from_orientation(self.startup_transition_goal.pose.orientation),
            self.entry_max_linear_speed, self.entry_max_yaw_rate, self.publish_rate_hz)
        self.motion_started = True
        self.transition("ENTRY", "distance=%.3f m, yaw=%.3f rad, duration=%.2f s, limiting=%s" %
                        (self.entry.distance, abs(self.entry.yaw_delta), self.entry.duration, self.entry.limiting_factor), wall, ros)
        self.publish_entry_sample(0.0)

    def publish_nav(self, position, velocity, yaw, yaw_rate):
        msg = FlightNav()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.FRAME_ID
        msg.control_frame = FlightNav.WORLD_FRAME
        msg.target = FlightNav.COG
        msg.pos_xy_nav_mode = msg.pos_z_nav_mode = msg.yaw_nav_mode = FlightNav.POS_VEL_MODE
        msg.target_pos_x, msg.target_pos_y, msg.target_pos_z = position
        msg.target_vel_x, msg.target_vel_y, msg.target_vel_z = velocity
        msg.target_yaw, msg.target_omega_z = yaw, yaw_rate
        self.nav_publisher.publish(msg)

    def publish_entry_sample(self, elapsed):
        self.publish_nav(*self.entry.sample(elapsed))

    def publish_entry_hold(self):
        self.publish_nav(self.position_to_tuple(self.startup_transition_goal.pose.position), (0.0, 0.0, 0.0),
                         self.yaw_from_orientation(self.startup_transition_goal.pose.orientation), 0.0)

    def begin_planner(self, ros, wall):
        self.motion_started = True
        self.planning_started_ros = ros
        self.transition("WAIT_PLANNER", "Holding entry pose; waiting for first valid full-state target", wall, ros)
        self.publish_entry_hold()
        self.root_target_publisher.publish(self.build_tail_pose_message(0.0))

    def planner_callback(self, msg):
        # Closing this same lock before recovery makes the ownership handoff atomic.
        with self.lock:
            if self.exiting or self.state not in ("WAIT_PLANNER", "DANCING"):
                return
            ros, wall = rospy.Time.now().to_sec(), time.monotonic()
            problems = self.input_problems(ros)
            if self.latest_flight_state != self.HOVER_STATE:
                problems.append("Left HOVER")
            if ros < self.last_ros:
                problems.append("ROS time moved backwards")
            if problems:
                self.fail("; ".join(problems), wall, ros)
                return
            # The outer stamp is refreshed even when copilot reuses an old input.
            source_stamp = msg.root_state.header.stamp.to_sec()
            if source_stamp < self.planning_started_ros or source_stamp > ros + 0.05:
                return
            if self.last_planner_source_stamp is not None and source_stamp <= self.last_planner_source_stamp:
                return
            if (not self.valid_pose(msg.root_state.pose.pose, msg.root_state.header.frame_id)
                    or not self.valid_joints(msg.joint_state)
                    or len(msg.joint_state.position) != len(self.SHUTDOWN_JOINT_NAMES)):
                return
            twist = msg.root_state.twist.twist
            if not all(math.isfinite(v) for v in self.position_to_tuple(twist.linear) + self.position_to_tuple(twist.angular)):
                return
            if self.state == "WAIT_PLANNER":
                self.dance_start_ros = ros
                self.transition("DANCING", "Planner ready; beginning dance", wall, ros)
            self.last_planner_source_stamp = source_stamp
            self.full_state_publisher.publish(msg)

    def stop_callback(self, _request):
        with self.lock:
            return TriggerResponse(success=self.request_stop("stop service"), message=self.state + ": " + self.detail)

    def request_stop(self, source):
        if self.state == "ERROR" or self.exiting:
            return False
        if self.state in self.STOPPING or self.state == "DONE":
            return True
        ros, wall = rospy.Time.now().to_sec(), time.monotonic()
        if not self.motion_started:
            self.transition("DONE", "Stopped before movement; bringup remains running", wall, ros)
            return True
        # Disable forwarding immediately, before publishing any recovery command.
        self.transition("STOP_WAIT", "Stop requested by %s; holding before joint recovery" % source, wall, ros)
        if self.input_problems(ros) or self.latest_flight_state != self.HOVER_STATE:
            self.fail("Cannot recover without valid feedback and HOVER", wall, ros)
            return False
        self.shutdown_hold_cog_target = self.build_shutdown_hold_cog_target(self.yaw_from_orientation(self.latest_cog_pose.pose.orientation))
        self.publish_shutdown_hold()
        return True

    def publish_shutdown_hold(self):
        goal = self.copy_pose_message(self.shutdown_hold_cog_target)
        goal.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(self.shutdown_cog_hold_duration)
        self.cog_target_publisher.publish(goal)

    def tick_shutdown(self, ros, wall):
        if self.state == "STOP_WAIT":
            velocity, yaw_rate = self.current_velocity()
            self.detail = "Holding before joint recovery: speed %.3f/%.3f m/s, yaw rate %.3f/%.3f rad/s" % (
                math.hypot(*velocity), 0.1 * self.entry_max_linear_speed,
                abs(yaw_rate), 0.1 * self.entry_max_yaw_rate)
            if not self.is_stationary():
                return
            self.transition("SHAPING", "Restoring configured joint shape", wall, ros)
        if self.state == "SHAPING":
            self.publish_shutdown_joint_shape_commands()
            error = self.compute_shutdown_joint_error()
            self.detail = "Joint recovery: max error %.3f rad" % error
            if error <= self.shutdown_joint_tolerance:
                # DragonNavigator levels the physical body through its baselink
                # rotation interface. CoG orientation is a mutable control frame.
                self.shutdown_leveling_start_rpy = self.rpy_from_orientation(self.latest_baselink_pose.pose.orientation)
                self.shutdown_hold_cog_target = self.build_shutdown_hold_cog_target(
                    self.yaw_from_orientation(self.latest_cog_pose.pose.orientation))
                self.transition("LEVELING", "Leveling body; checking baselink attitude and joint shape", wall, ros)
                # Send each hold once. Repeating target_pose would restart the
                # navigator's trajectory and flood its logs at the loop rate.
                self.publish_shutdown_hold()
        if self.state == "LEVELING":
            elapsed = ros - self.state_ros
            # Maintain the planar shape while changing the body attitude, just
            # as DragonNavigator::landingProcess keeps its pitch joints at zero.
            self.publish_shutdown_joint_shape_commands()
            # Includes progress=1; do not report completion before sending the endpoint.
            self.publish_shutdown_leveling_commands(min(elapsed, self.shutdown_leveling_publish_duration))
            roll, pitch, _ = self.rpy_from_orientation(self.latest_baselink_pose.pose.orientation)
            error = max(abs(roll), abs(pitch))
            joint_error = self.compute_shutdown_joint_error()
            self.detail = "Leveling %.1f/%.1f s; baselink tilt %.3f rad, joint error %.3f rad" % (
                elapsed, self.shutdown_leveling_publish_duration, error, joint_error)
            ready = (elapsed >= self.shutdown_leveling_publish_duration
                     and error <= self.shutdown_leveling_tolerance
                     and joint_error <= self.shutdown_joint_tolerance)
            if ready:
                self.transition("DONE", "Recovery complete. Land through the standard bringup interface; bringup remains running.", wall, ros)

    def connection_problems(self):
        publishers = [self.cog_target_publisher, self.rotation_target_publisher, self.joint_target_publisher]
        if self.state not in self.STOPPING:
            publishers += [self.nav_publisher, self.root_target_publisher, self.full_state_publisher]
        problems = ["no subscriber: " + pub.resolved_name for pub in publishers if pub.get_num_connections() == 0]
        if self.state not in self.STOPPING and self.planner_subscriber.get_num_connections() == 0:
            problems.append("no planner output connection")
        return problems

    def input_problems(self, ros):
        problems = []
        for topic in (self.ROOT_POSE_TOPIC, self.COG_ODOM_TOPIC, self.BASELINK_ODOM_TOPIC,
                      self.JOINT_STATE_TOPIC, self.FLIGHT_STATE_TOPIC):
            if topic in self.invalid_inputs:
                problems.append(topic + ": " + self.invalid_inputs[topic])
            elif topic not in self.received:
                problems.append("missing " + rospy.resolve_name(topic))
            else:
                stamp = self.received[topic]
                if stamp is not None and stamp > ros + 0.1:
                    problems.append("future timestamp: " + rospy.resolve_name(topic))
        return problems

    def valid_pose(self, pose, frame):
        values = self.position_to_tuple(pose.position) + (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        norm = sum(v * v for v in values[3:])
        return frame.lstrip("/") == self.FRAME_ID.lstrip("/") and all(math.isfinite(v) for v in values) and abs(norm - 1.0) < 0.01

    def valid_joints(self, msg):
        return (len(msg.name) == len(msg.position) and len(set(msg.name)) == len(msg.name)
                and set(self.SHUTDOWN_JOINT_NAMES).issubset(msg.name)
                and all(math.isfinite(v) for v in msg.position))

    def record_input(self, topic, valid, stamp=None):
        if not valid:
            self.invalid_inputs[topic] = "invalid frame, values or incomplete feedback"
            return False
        self.invalid_inputs.pop(topic, None)
        self.received[topic] = stamp
        return True

    def root_pose_callback(self, msg):
        with self.lock:
            if self.record_input(self.ROOT_POSE_TOPIC, self.valid_pose(msg.pose, msg.header.frame_id), msg.header.stamp.to_sec()):
                self.latest_root_pose = copy.deepcopy(msg)

    def cog_odom_callback(self, msg):
        with self.lock:
            twist = msg.twist.twist
            valid = self.valid_pose(msg.pose.pose, msg.header.frame_id)
            valid = valid and all(math.isfinite(v) for v in self.position_to_tuple(twist.linear) + self.position_to_tuple(twist.angular))
            if valid:
                roll, pitch, _ = self.rpy_from_orientation(msg.pose.pose.orientation)
                try:
                    yaw_rate_from_body_rates(roll, pitch, self.position_to_tuple(twist.angular))
                except ValueError:
                    valid = False
            if self.record_input(self.COG_ODOM_TOPIC, valid, msg.header.stamp.to_sec()):
                self.latest_cog_odom = copy.deepcopy(msg)
                self.latest_cog_pose = PoseStamped(header=copy.deepcopy(msg.header), pose=copy.deepcopy(msg.pose.pose))

    def joint_state_callback(self, msg):
        with self.lock:
            # Some servo drivers omit the header stamp.
            stamp = msg.header.stamp.to_sec() or None
            if self.record_input(self.JOINT_STATE_TOPIC, self.valid_joints(msg), stamp):
                self.latest_joint_state = copy.deepcopy(msg)

    def baselink_odom_callback(self, msg):
        with self.lock:
            if self.record_input(self.BASELINK_ODOM_TOPIC, self.valid_pose(msg.pose.pose, msg.header.frame_id),
                                 msg.header.stamp.to_sec()):
                self.latest_baselink_pose = PoseStamped(header=copy.deepcopy(msg.header), pose=copy.deepcopy(msg.pose.pose))

    def flight_state_callback(self, msg):
        with self.lock:
            # Dragon also has extended states such as PRE_LAND (0x20).
            if self.record_input(self.FLIGHT_STATE_TOPIC, 0 <= msg.data <= 255):
                self.latest_flight_state = msg.data
                if self.motion_started and msg.data != self.HOVER_STATE:
                    self.fail("Left HOVER (flight_state=%s)" % msg.data)

    def resolve_robot_model(self):
        description = rospy.get_param("robot_description", None)
        if description is None:
            return False
        model = ElementTree.fromstring(description)
        joints = {j.get("name"): j for j in model.findall("joint")}
        link_joints = {name for name in joints if name and name.startswith("joint") and name.endswith(("_pitch", "_yaw"))}
        if set(self.SHUTDOWN_JOINT_NAMES) != link_joints:
            raise ValueError("shutdown_joint_names must cover all model pitch/yaw joints")
        for name, position in zip(self.SHUTDOWN_JOINT_NAMES, self.SHUTDOWN_JOINT_POSITIONS):
            limit = joints[name].find("limit")
            if limit is None or not float(limit.get("lower", "-inf")) <= position <= float(limit.get("upper", "inf")):
                raise ValueError("shutdown_joint_positions outside model limit for " + name)
        length = self.config.get("root_link_length")
        if length is None:
            origin = joints.get("joint1_pitch").find("origin") if "joint1_pitch" in joints else None
            if origin is None:
                raise ValueError("missing joint1_pitch origin; set a valid root_link_length")
            xyz = origin.get("xyz", "").split()
            if len(xyz) != 3:
                raise ValueError("joint1_pitch origin must contain three coordinates")
            length = abs(float(xyz[0]))
        if not math.isfinite(length) or length <= 0:
            raise ValueError("root link length must be positive and finite")
        self.root_link_length = length
        rospy.loginfo("[Dance] Validated robot model, root link length %.3f m", length)
        return True

    def configure_keyboard_input(self):
        if not sys.stdin.isatty():
            rospy.logwarn("[Dance] No interactive terminal; use rosservice call %s '{}'", rospy.resolve_name("~stop"))
            return False
        try:
            self.stdin_fd = sys.stdin.fileno()
            self.stdin_settings = termios.tcgetattr(self.stdin_fd)
            tty.setcbreak(self.stdin_fd)
            return True
        except (termios.error, ValueError, OSError) as exc:
            rospy.logwarn("[Dance] Keyboard unavailable: %s; use stop service", exc)
            self.restore_terminal_settings()
            return False

    def restore_terminal_settings(self):
        if self.stdin_fd is not None and self.stdin_settings is not None:
            try:
                termios.tcsetattr(self.stdin_fd, termios.TCSADRAIN, self.stdin_settings)
            except (termios.error, ValueError, OSError):
                pass
        self.stdin_fd = self.stdin_settings = None
        self.keyboard_enabled = False

    def key_pressed(self):
        if not self.keyboard_enabled:
            return False
        try:
            readable, _, _ = select.select([sys.stdin], [], [], 0.0)
            if readable:
                key = sys.stdin.read(1)
                if not key:
                    self.restore_terminal_settings()
                return key.lower() == "q"
        except (OSError, ValueError):
            rospy.logwarn("[Dance] Keyboard input lost; use stop service")
            self.restore_terminal_settings()
        return False

    def on_ros_shutdown(self):
        with self.lock:
            if not self.exiting and self.state not in self.TERMINAL:
                rospy.logwarn("[Dance] Launch terminated; graceful recovery was not completed. Bringup remains independent.")
            self.exiting = True
            self.restore_terminal_settings()


if __name__ == "__main__":
    try:
        sys.exit(DragonDancePublisher().run())
    except (ValueError, KeyError, TypeError, OSError, yaml.YAMLError) as exc:
        rospy.logfatal("[Dance] Cannot run demo: %s", exc)
        sys.exit(1)
    except rospy.ROSInterruptException:
        pass
