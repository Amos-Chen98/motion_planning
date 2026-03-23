#!/usr/bin/env python3

import math
import os
import select
import sys
import termios
import tty
from xml.etree import ElementTree

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt8

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from dragon_dance_support import DragonDanceSupportMixin


class DragonDancePublisher(DragonDanceSupportMixin):
    ROOT_TARGET_TOPIC = "root/target_pose"
    COG_TARGET_TOPIC = "target_pose"
    ROTATION_TARGET_TOPIC = "target_rotation_motion"
    JOINT_CONTROL_TOPIC = "joints_ctrl"
    ROOT_POSE_TOPIC = "root/pose"
    COG_ODOM_TOPIC = "uav/cog/odom"
    JOINT_STATE_TOPIC = "joint_states"
    FLIGHT_STATE_TOPIC = "flight_state"
    FRAME_ID = "world"
    ROTATION_FRAME_ID = "cog"
    HOVER_STATE = 5
    PUBLISH_RATE_HZ = 40.0
    TOTAL_DURATION = 60.0
    STARTUP_TRANSITION_DURATION = 8.0
    STARTUP_POSITION_TOLERANCE = 0.15
    STARTUP_YAW_TOLERANCE = 0.15
    STARTUP_GOAL_RETRY_MARGIN = 2.0
    SHUTDOWN_WAIT_DURATION = 3.0
    SHUTDOWN_JOINT_SHAPE_TIMEOUT = 6.0
    SHUTDOWN_JOINT_TOLERANCE = 0.05
    SHUTDOWN_COG_HOLD_DURATION = 0.5
    SHUTDOWN_LEVELING_PUBLISH_DURATION = 6.0
    TAIL_FLU_TO_ROOT_YAW_OFFSET = math.pi
    ROBOT_DESCRIPTION_PARAM = "robot_description"
    SHUTDOWN_JOINT_NAMES = (
        "joint1_pitch",
        "joint1_yaw",
        "joint2_pitch",
        "joint2_yaw",
        "joint3_pitch",
        "joint3_yaw",
    )
    SHUTDOWN_JOINT_POSITIONS = (
        0.0,
        math.pi / 2.0,
        0.0,
        math.pi / 2.0,
        0.0,
        math.pi / 2.0,
    )

    ELLIPSE_CENTER_X = 0.0
    ELLIPSE_CENTER_Y = -0.7
    ELLIPSE_SEMI_MAJOR = 2.0
    ELLIPSE_SEMI_MINOR = 1.25

    Z_OFFSET = 1.3
    Z_AMPLITUDE = 0.5
    Z_CYCLES = 3.0

    def __init__(self):
        rospy.init_node("dragon_dance", anonymous=False)

        self.root_target_publisher = rospy.Publisher(self.ROOT_TARGET_TOPIC, PoseStamped, queue_size=1)
        self.cog_target_publisher = rospy.Publisher(self.COG_TARGET_TOPIC, PoseStamped, queue_size=1)
        self.rotation_target_publisher = rospy.Publisher(self.ROTATION_TARGET_TOPIC, Odometry, queue_size=1)
        self.joint_target_publisher = rospy.Publisher(self.JOINT_CONTROL_TOPIC, JointState, queue_size=1)
        self.root_pose_subscriber = rospy.Subscriber(
            self.ROOT_POSE_TOPIC,
            PoseStamped,
            self.root_pose_callback,
            queue_size=1,
        )
        self.cog_odom_subscriber = rospy.Subscriber(
            self.COG_ODOM_TOPIC,
            Odometry,
            self.cog_odom_callback,
            queue_size=1,
        )
        self.joint_state_subscriber = rospy.Subscriber(
            self.JOINT_STATE_TOPIC,
            JointState,
            self.joint_state_callback,
            queue_size=1,
        )
        self.flight_state_subscriber = rospy.Subscriber(
            self.FLIGHT_STATE_TOPIC,
            UInt8,
            self.flight_state_callback,
            queue_size=1,
        )
        self.startup_transition_duration = rospy.get_param(
            "~startup_transition_duration",
            self.STARTUP_TRANSITION_DURATION,
        )
        self.startup_position_tolerance = rospy.get_param(
            "~startup_position_tolerance",
            self.STARTUP_POSITION_TOLERANCE,
        )
        self.startup_yaw_tolerance = rospy.get_param(
            "~startup_yaw_tolerance",
            self.STARTUP_YAW_TOLERANCE,
        )
        self.shutdown_wait_duration = rospy.get_param(
            "~shutdown_wait_duration",
            self.SHUTDOWN_WAIT_DURATION,
        )
        self.shutdown_joint_shape_timeout = rospy.get_param(
            "~shutdown_joint_shape_timeout",
            self.SHUTDOWN_JOINT_SHAPE_TIMEOUT,
        )
        self.shutdown_joint_tolerance = rospy.get_param(
            "~shutdown_joint_tolerance",
            self.SHUTDOWN_JOINT_TOLERANCE,
        )
        self.shutdown_cog_hold_duration = rospy.get_param(
            "~shutdown_cog_hold_duration",
            self.SHUTDOWN_COG_HOLD_DURATION,
        )
        self.shutdown_leveling_publish_duration = rospy.get_param(
            "~shutdown_leveling_publish_duration",
            self.SHUTDOWN_LEVELING_PUBLISH_DURATION,
        )
        self.root_link_length = self.resolve_root_link_length()
        self.samples_per_cycle = int(round(self.TOTAL_DURATION * self.PUBLISH_RATE_HZ))
        self.sample_index = 0
        self.completed_cycles = 0
        self.latest_root_pose = None
        self.latest_cog_pose = None
        self.latest_joint_state = None
        self.latest_flight_state = None
        self.startup_transition_complete = False
        self.startup_transition_goal = None
        self.startup_goal_sent_time = None
        self.startup_tail_pose_flu = self.build_tail_pose_message(0.0)
        self.shutdown_requested = False
        self.shutdown_request_time = None
        self.shutdown_joint_shape_started = False
        self.shutdown_joint_shape_complete = False
        self.shutdown_joint_shape_start_time = None
        self.shutdown_leveling_started = False
        self.shutdown_leveling_start_time = None
        self.shutdown_leveling_start_rpy = None
        self.shutdown_joint_target = self.build_shutdown_joint_target()
        self.shutdown_hold_cog_target = None
        self.stdin_fd = None
        self.stdin_settings = None
        self.keyboard_enabled = self.configure_keyboard_input()
        self.resolved_root_pose_topic = rospy.resolve_name(self.ROOT_POSE_TOPIC)
        self.resolved_cog_odom_topic = rospy.resolve_name(self.COG_ODOM_TOPIC)
        self.resolved_cog_target_topic = rospy.resolve_name(self.COG_TARGET_TOPIC)
        self.resolved_rotation_target_topic = rospy.resolve_name(self.ROTATION_TARGET_TOPIC)
        self.resolved_joint_control_topic = rospy.resolve_name(self.JOINT_CONTROL_TOPIC)
        self.resolved_joint_state_topic = rospy.resolve_name(self.JOINT_STATE_TOPIC)
        self.resolved_root_target_topic = rospy.resolve_name(self.ROOT_TARGET_TOPIC)
        self.resolved_flight_state_topic = rospy.resolve_name(self.FLIGHT_STATE_TOPIC)

        rospy.on_shutdown(self.restore_terminal_settings)

        rospy.loginfo(
            "Publishing dragon dance trajectory on %s at %.1f Hz with %d samples per cycle",
            self.root_target_publisher.resolved_name,
            self.PUBLISH_RATE_HZ,
            self.samples_per_cycle,
        )
        rospy.loginfo(
            "Startup transition uses %s -> %s with current root pose from %s and CoG odom from %s",
            self.resolved_cog_target_topic,
            self.resolved_root_target_topic,
            self.resolved_root_pose_topic,
            self.resolved_cog_odom_topic,
        )
        rospy.loginfo(
            "Startup goal targets the dragon-dance entry tail pose in FLU [x=%.2f, y=%.2f, z=%.2f] over %.1f s",
            self.startup_tail_pose_flu.pose.position.x,
            self.startup_tail_pose_flu.pose.position.y,
            self.startup_tail_pose_flu.pose.position.z,
            self.startup_transition_duration,
        )
        rospy.loginfo("Resolved root link length for startup target conversion: %.3f m", self.root_link_length)

    def run(self):
        rate = rospy.Rate(self.PUBLISH_RATE_HZ)
        try:
            while not rospy.is_shutdown():
                if not self.shutdown_requested and self.key_pressed():
                    self.shutdown_requested = True
                    self.shutdown_request_time = rospy.Time.now()
                    rospy.loginfo(
                        "Key press detected, stopping root tail target publishing and waiting %.1f s before leveling CoG via %s",
                        self.shutdown_wait_duration,
                        self.resolved_rotation_target_topic,
                    )

                if self.shutdown_requested:
                    if self.handle_shutdown_sequence():
                        break
                    rate.sleep()
                    continue

                pose_msg = self.build_next_pose_message()
                if pose_msg is not None:
                    self.root_target_publisher.publish(pose_msg)

                rate.sleep()
        finally:
            self.restore_terminal_settings()

    def build_next_pose_message(self):
        if not self.startup_transition_complete:
            self.handle_startup_transition()
            return None

        elapsed = self.sample_index / self.PUBLISH_RATE_HZ
        pose_msg = self.build_tail_pose_message(elapsed)

        self.sample_index += 1
        if self.sample_index >= self.samples_per_cycle:
            self.sample_index = 0
            self.completed_cycles += 1
            rospy.loginfo("Completed trajectory cycle %d, restarting", self.completed_cycles)

        return pose_msg

    def handle_startup_transition(self):
        missing_topics = []
        if self.latest_root_pose is None:
            missing_topics.append(self.resolved_root_pose_topic)
        if self.latest_cog_pose is None:
            missing_topics.append(self.resolved_cog_odom_topic)
        if self.latest_flight_state is None:
            missing_topics.append(self.resolved_flight_state_topic)

        if missing_topics:
            rospy.loginfo_throttle(
                2.0,
                "Waiting for startup inputs: %s",
                ", ".join(missing_topics),
            )
            return

        if self.latest_flight_state != self.HOVER_STATE:
            rospy.loginfo_throttle(
                2.0,
                "Waiting for hover state on %s before sending startup CoG target",
                self.resolved_flight_state_topic,
            )
            return

        if self.cog_target_publisher.get_num_connections() == 0:
            rospy.loginfo_throttle(
                2.0,
                "Waiting for a subscriber on %s before sending the startup CoG target",
                self.resolved_cog_target_topic,
            )
            return

        if self.startup_transition_goal is None:
            self.startup_transition_goal = self.build_startup_cog_target(
                self.latest_root_pose,
                self.latest_cog_pose,
            )
            rospy.loginfo(
                "Prepared startup CoG goal [x=%.2f, y=%.2f, z=%.2f, yaw=%.2f rad] from root pose [x=%.2f, y=%.2f, z=%.2f]",
                self.startup_transition_goal.pose.position.x,
                self.startup_transition_goal.pose.position.y,
                self.startup_transition_goal.pose.position.z,
                self.yaw_from_orientation(self.startup_transition_goal.pose.orientation),
                self.latest_root_pose.pose.position.x,
                self.latest_root_pose.pose.position.y,
                self.latest_root_pose.pose.position.z,
            )
            self.publish_startup_cog_target()
            return

        position_error, yaw_error = self.compute_startup_goal_error()
        if position_error <= self.startup_position_tolerance and yaw_error <= self.startup_yaw_tolerance:
            self.startup_transition_complete = True
            rospy.loginfo(
                "Startup CoG transition completed with position error %.3f m and yaw error %.3f rad; switching to %s",
                position_error,
                yaw_error,
                self.resolved_root_target_topic,
            )
            return

        rospy.loginfo_throttle(
            2.0,
            "Navigating CoG to startup goal: position error %.3f m, yaw error %.3f rad",
            position_error,
            yaw_error,
        )

        if self.startup_goal_sent_time is None:
            return

        elapsed_since_goal = (rospy.Time.now() - self.startup_goal_sent_time).to_sec()
        retry_delay = self.startup_transition_duration + self.STARTUP_GOAL_RETRY_MARGIN
        if elapsed_since_goal >= retry_delay:
            rospy.logwarn(
                "Startup CoG goal not reached after %.1f s, republishing target_pose",
                elapsed_since_goal,
            )
            self.publish_startup_cog_target()

    def handle_shutdown_sequence(self):
        elapsed = (rospy.Time.now() - self.shutdown_request_time).to_sec()
        if elapsed < self.shutdown_wait_duration:
            rospy.loginfo_throttle(
                1.0,
                "Shutdown requested; root tail publishing is paused for %.1f / %.1f s before CoG leveling",
                elapsed,
                self.shutdown_wait_duration,
            )
            return False

        if self.latest_cog_pose is None:
            rospy.loginfo_throttle(
                2.0,
                "Waiting for %s before publishing the shutdown CoG leveling target",
                self.resolved_cog_odom_topic,
            )
            return False

        if self.latest_flight_state != self.HOVER_STATE:
            rospy.loginfo_throttle(
                2.0,
                "Waiting for hover state on %s before publishing the shutdown CoG leveling target",
                self.resolved_flight_state_topic,
            )
            return False

        if self.cog_target_publisher.get_num_connections() == 0:
            rospy.loginfo_throttle(
                2.0,
                "Waiting for a subscriber on %s before publishing the shutdown CoG hold target",
                self.resolved_cog_target_topic,
            )
            return False

        if self.rotation_target_publisher.get_num_connections() == 0:
            rospy.loginfo_throttle(
                2.0,
                "Waiting for a subscriber on %s before publishing the shutdown CoG leveling target",
                self.resolved_rotation_target_topic,
            )
            return False

        if self.joint_target_publisher.get_num_connections() == 0:
            rospy.loginfo_throttle(
                2.0,
                "Waiting for a subscriber on %s before publishing the shutdown joint target",
                self.resolved_joint_control_topic,
            )
            return False

        if not self.shutdown_joint_shape_complete:
            return self.handle_shutdown_joint_shape()

        if not self.shutdown_leveling_started:
            self.shutdown_leveling_start_rpy = self.rpy_from_orientation(self.latest_cog_pose.pose.orientation)
            self.shutdown_hold_cog_target = self.build_shutdown_hold_cog_target(self.shutdown_leveling_start_rpy[2])
            self.shutdown_leveling_started = True
            self.shutdown_leveling_start_time = rospy.Time.now()
            rospy.loginfo(
                "Starting shutdown leveling sequence after joint shaping: holding CoG via %s and interpolating CoG attitude via %s for %.1f s from [roll=%.2f, pitch=%.2f, yaw=%.2f] rad",
                self.resolved_cog_target_topic,
                self.resolved_rotation_target_topic,
                self.shutdown_leveling_publish_duration,
                self.shutdown_leveling_start_rpy[0],
                self.shutdown_leveling_start_rpy[1],
                self.shutdown_leveling_start_rpy[2],
            )

        leveling_elapsed = (rospy.Time.now() - self.shutdown_leveling_start_time).to_sec()
        if leveling_elapsed >= self.shutdown_leveling_publish_duration:
            rospy.loginfo("Shutdown leveling commands published for %.1f s, exiting demo node", leveling_elapsed)
            return True

        self.publish_shutdown_leveling_commands(leveling_elapsed)
        rospy.loginfo_throttle(
            1.0,
            "Publishing shutdown hold/level commands for %.1f / %.1f s",
            leveling_elapsed,
            self.shutdown_leveling_publish_duration,
        )
        return False

    def handle_shutdown_joint_shape(self):
        if not self.shutdown_joint_shape_started:
            self.shutdown_joint_shape_started = True
            self.shutdown_joint_shape_start_time = rospy.Time.now()
            current_yaw = self.yaw_from_orientation(self.latest_cog_pose.pose.orientation)
            self.shutdown_hold_cog_target = self.build_shutdown_hold_cog_target(current_yaw)
            rospy.loginfo(
                "Starting shutdown joint shaping on %s toward %s",
                self.resolved_joint_control_topic,
                list(self.SHUTDOWN_JOINT_POSITIONS),
            )

        shaping_elapsed = (rospy.Time.now() - self.shutdown_joint_shape_start_time).to_sec()
        self.publish_shutdown_joint_shape_commands()

        joint_error = self.compute_shutdown_joint_error()
        if joint_error is not None and joint_error <= self.shutdown_joint_tolerance:
            self.shutdown_joint_shape_complete = True
            rospy.loginfo(
                "Shutdown joint shaping completed with max error %.3f rad; proceeding to CoG leveling",
                joint_error,
            )
            return False

        if shaping_elapsed >= self.shutdown_joint_shape_timeout:
            if joint_error is None:
                rospy.logwarn(
                    "Shutdown joint shaping timed out after %.1f s without usable %s feedback; proceeding to CoG leveling",
                    shaping_elapsed,
                    self.resolved_joint_state_topic,
                )
            else:
                rospy.logwarn(
                    "Shutdown joint shaping timed out after %.1f s with remaining max error %.3f rad; proceeding to CoG leveling",
                    shaping_elapsed,
                    joint_error,
                )
            self.shutdown_joint_shape_complete = True
            return False

        if joint_error is None:
            rospy.loginfo_throttle(
                1.0,
                "Publishing shutdown joint target for %.1f / %.1f s while waiting for %s",
                shaping_elapsed,
                self.shutdown_joint_shape_timeout,
                self.resolved_joint_state_topic,
            )
        else:
            rospy.loginfo_throttle(
                1.0,
                "Publishing shutdown joint target for %.1f / %.1f s with current max joint error %.3f rad",
                shaping_elapsed,
                self.shutdown_joint_shape_timeout,
                joint_error,
            )
        return False

    def resolve_root_link_length(self):
        param_link_length = rospy.get_param("~root_link_length", None)
        if param_link_length is not None:
            return float(param_link_length)

        robot_description_param = rospy.resolve_name(self.ROBOT_DESCRIPTION_PARAM)
        try:
            robot_description = rospy.get_param(robot_description_param)
        except KeyError:
            rospy.logwarn(
                "Could not find %s; using zero root link length for startup target conversion",
                robot_description_param,
            )
            return 0.0

        try:
            robot_description_xml = ElementTree.fromstring(robot_description)
        except ElementTree.ParseError as exc:
            rospy.logwarn(
                "Failed to parse %s (%s); using zero root link length for startup target conversion",
                robot_description_param,
                exc,
            )
            return 0.0

        for joint in robot_description_xml.findall("joint"):
            if joint.get("name") != "joint1_pitch":
                continue

            origin = joint.find("origin")
            if origin is None:
                break

            xyz = origin.get("xyz", "").split()
            if not xyz:
                break

            try:
                return abs(float(xyz[0]))
            except ValueError:
                break

        rospy.logwarn(
            "Failed to resolve joint1_pitch origin from %s; using zero root link length for startup target conversion",
            robot_description_param,
        )
        return 0.0

    def publish_startup_cog_target(self):
        goal_msg = self.copy_pose_message(self.startup_transition_goal)
        now = rospy.Time.now()
        # Navigation uses the goal timestamp to infer a smooth arrival duration.
        goal_msg.header.stamp = now + rospy.Duration.from_sec(self.startup_transition_duration)
        goal_msg.header.frame_id = self.FRAME_ID
        self.cog_target_publisher.publish(goal_msg)
        self.startup_goal_sent_time = now
        rospy.loginfo(
            "Published startup CoG target on %s with requested arrival time %.1f s",
            self.resolved_cog_target_topic,
            self.startup_transition_duration,
        )

    def root_pose_callback(self, msg):
        self.latest_root_pose = self.copy_pose_message(msg)

    def cog_odom_callback(self, msg):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = msg.header.stamp
        pose_msg.header.frame_id = msg.header.frame_id
        pose_msg.pose.position.x = msg.pose.pose.position.x
        pose_msg.pose.position.y = msg.pose.pose.position.y
        pose_msg.pose.position.z = msg.pose.pose.position.z
        pose_msg.pose.orientation.x = msg.pose.pose.orientation.x
        pose_msg.pose.orientation.y = msg.pose.pose.orientation.y
        pose_msg.pose.orientation.z = msg.pose.pose.orientation.z
        pose_msg.pose.orientation.w = msg.pose.pose.orientation.w
        self.latest_cog_pose = pose_msg

    def joint_state_callback(self, msg):
        copied_joint_state = JointState()
        copied_joint_state.header = msg.header
        copied_joint_state.name = list(msg.name)
        copied_joint_state.position = list(msg.position)
        copied_joint_state.velocity = list(msg.velocity)
        copied_joint_state.effort = list(msg.effort)
        self.latest_joint_state = copied_joint_state

    def flight_state_callback(self, msg):
        self.latest_flight_state = msg.data

    def configure_keyboard_input(self):
        if not sys.stdin.isatty():
            rospy.logwarn("No interactive terminal detected, repeating until the node is stopped externally")
            return False

        try:
            self.stdin_fd = sys.stdin.fileno()
            self.stdin_settings = termios.tcgetattr(self.stdin_fd)
            tty.setcbreak(self.stdin_fd)
        except (termios.error, ValueError, OSError) as exc:
            rospy.logwarn("Failed to enable keyboard stop input: %s", exc)
            self.stdin_fd = None
            self.stdin_settings = None
            return False

        rospy.loginfo("Press any key in this terminal to stop root tail publishing and trigger the shutdown leveling sequence")
        return True

    def restore_terminal_settings(self):
        if self.stdin_fd is None or self.stdin_settings is None:
            return

        termios.tcsetattr(self.stdin_fd, termios.TCSADRAIN, self.stdin_settings)
        self.stdin_fd = None
        self.stdin_settings = None

    def key_pressed(self):
        if not self.keyboard_enabled:
            return False

        readable, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not readable:
            return False

        sys.stdin.read(1)
        return True


if __name__ == "__main__":
    try:
        DragonDancePublisher().run()
    except rospy.ROSInterruptException:
        pass
