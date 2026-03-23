#!/usr/bin/env python3

import math
import select
import sys
import termios
import tty
from xml.etree import ElementTree

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8


class DragonDancePublisher:
    ROOT_TARGET_TOPIC = "root/target_pose"
    COG_TARGET_TOPIC = "target_pose"
    ROOT_POSE_TOPIC = "root/pose"
    COG_ODOM_TOPIC = "uav/cog/odom"
    FLIGHT_STATE_TOPIC = "flight_state"
    FRAME_ID = "world"
    HOVER_STATE = 5
    PUBLISH_RATE_HZ = 40.0
    TOTAL_DURATION = 60.0
    STARTUP_TRANSITION_DURATION = 8.0
    STARTUP_POSITION_TOLERANCE = 0.15
    STARTUP_YAW_TOLERANCE = 0.15
    STARTUP_GOAL_RETRY_MARGIN = 2.0
    TAIL_FLU_TO_ROOT_YAW_OFFSET = math.pi
    ROBOT_DESCRIPTION_PARAM = "robot_description"

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
        self.root_link_length = self.resolve_root_link_length()
        self.samples_per_cycle = int(round(self.TOTAL_DURATION * self.PUBLISH_RATE_HZ))
        self.sample_index = 0
        self.completed_cycles = 0
        self.latest_root_pose = None
        self.latest_cog_pose = None
        self.latest_flight_state = None
        self.startup_transition_complete = False
        self.startup_transition_goal = None
        self.startup_goal_sent_time = None
        self.startup_tail_pose_flu = self.build_tail_pose_message(0.0)
        self.stdin_fd = None
        self.stdin_settings = None
        self.keyboard_enabled = self.configure_keyboard_input()
        self.resolved_root_pose_topic = rospy.resolve_name(self.ROOT_POSE_TOPIC)
        self.resolved_cog_odom_topic = rospy.resolve_name(self.COG_ODOM_TOPIC)
        self.resolved_cog_target_topic = rospy.resolve_name(self.COG_TARGET_TOPIC)
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
                if self.key_pressed():
                    rospy.loginfo("Key press detected, stopping trajectory publisher")
                    break

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

    def build_tail_pose_message(self, elapsed):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = self.FRAME_ID

        angular_speed = 2.0 * math.pi / self.TOTAL_DURATION
        z_angular_speed = 2.0 * math.pi * self.Z_CYCLES / self.TOTAL_DURATION
        theta = (math.pi / 2.0) - angular_speed * elapsed

        pose_msg.pose.position.x = self.ELLIPSE_CENTER_X + self.ELLIPSE_SEMI_MAJOR * math.cos(theta)
        pose_msg.pose.position.y = self.ELLIPSE_CENTER_Y + self.ELLIPSE_SEMI_MINOR * math.sin(theta)

        z_phase = z_angular_speed * elapsed - (math.pi / 2.0)
        pose_msg.pose.position.z = self.Z_OFFSET + self.Z_AMPLITUDE * math.sin(z_phase)

        velocity_x = self.ELLIPSE_SEMI_MAJOR * angular_speed * math.sin(theta)
        velocity_y = -self.ELLIPSE_SEMI_MINOR * angular_speed * math.cos(theta)
        velocity_z = self.Z_AMPLITUDE * z_angular_speed * math.cos(z_phase)

        horizontal_speed = math.hypot(velocity_x, velocity_y)
        yaw = math.atan2(velocity_y, velocity_x)
        # FLU uses +X forward, +Y left, +Z up, so following the velocity vector
        # requires the link-frame pitch sign to be flipped here.
        pitch = -math.atan2(velocity_z, horizontal_speed)

        half_pitch = 0.5 * pitch
        half_yaw = 0.5 * yaw
        pose_msg.pose.orientation.x = -math.sin(half_pitch) * math.sin(half_yaw)
        pose_msg.pose.orientation.y = math.sin(half_pitch) * math.cos(half_yaw)
        pose_msg.pose.orientation.z = math.cos(half_pitch) * math.sin(half_yaw)
        pose_msg.pose.orientation.w = math.cos(half_pitch) * math.cos(half_yaw)

        return pose_msg

    def build_startup_cog_target(self, root_pose, cog_pose):
        startup_root_pose = self.convert_flu_tail_pose_to_root_pose(self.startup_tail_pose_flu)
        startup_goal = PoseStamped()
        startup_goal.header.stamp = rospy.Time.now()
        startup_goal.header.frame_id = self.FRAME_ID

        # Preserve the current root-to-CoG transform while projecting the FLU tail
        # entry pose onto a CoG target.
        root_orientation = self.orientation_to_tuple(root_pose.pose.orientation)
        cog_orientation = self.orientation_to_tuple(cog_pose.pose.orientation)
        root_to_cog_orientation = self.quaternion_multiply(
            self.quaternion_inverse(root_orientation),
            cog_orientation,
        )

        root_position = self.position_to_tuple(root_pose.pose.position)
        cog_position = self.position_to_tuple(cog_pose.pose.position)
        root_to_cog_translation_world = self.subtract_vectors(cog_position, root_position)
        root_to_cog_translation_root = self.rotate_vector(
            self.quaternion_inverse(root_orientation),
            root_to_cog_translation_world,
        )

        startup_root_orientation = self.orientation_to_tuple(startup_root_pose.pose.orientation)
        startup_root_position = self.position_to_tuple(startup_root_pose.pose.position)
        rotated_translation = self.rotate_vector(startup_root_orientation, root_to_cog_translation_root)
        startup_goal.pose.position.x = startup_root_position[0] + rotated_translation[0]
        startup_goal.pose.position.y = startup_root_position[1] + rotated_translation[1]
        startup_goal.pose.position.z = startup_root_position[2] + rotated_translation[2]

        startup_cog_orientation = self.quaternion_multiply(
            startup_root_orientation,
            root_to_cog_orientation,
        )
        startup_goal.pose.orientation.x = startup_cog_orientation[0]
        startup_goal.pose.orientation.y = startup_cog_orientation[1]
        startup_goal.pose.orientation.z = startup_cog_orientation[2]
        startup_goal.pose.orientation.w = startup_cog_orientation[3]

        return startup_goal

    def convert_flu_tail_pose_to_root_pose(self, tail_pose_flu):
        root_pose = PoseStamped()
        root_pose.header.stamp = rospy.Time.now()
        root_pose.header.frame_id = self.FRAME_ID

        tail_orientation_flu = self.orientation_to_tuple(tail_pose_flu.pose.orientation)
        root_orientation = self.quaternion_multiply(
            self.quaternion_from_yaw(self.TAIL_FLU_TO_ROOT_YAW_OFFSET),
            tail_orientation_flu,
        )
        tail_position = self.position_to_tuple(tail_pose_flu.pose.position)
        link_direction = self.rotate_vector(root_orientation, (1.0, 0.0, 0.0))

        root_pose.pose.position.x = tail_position[0] - self.root_link_length * link_direction[0]
        root_pose.pose.position.y = tail_position[1] - self.root_link_length * link_direction[1]
        root_pose.pose.position.z = tail_position[2] - self.root_link_length * link_direction[2]
        root_pose.pose.orientation.x = root_orientation[0]
        root_pose.pose.orientation.y = root_orientation[1]
        root_pose.pose.orientation.z = root_orientation[2]
        root_pose.pose.orientation.w = root_orientation[3]

        return root_pose

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

    def compute_startup_goal_error(self):
        goal_position = self.position_to_tuple(self.startup_transition_goal.pose.position)
        current_position = self.position_to_tuple(self.latest_cog_pose.pose.position)
        position_error = math.sqrt(
            sum(
                (goal_component - current_component) * (goal_component - current_component)
                for goal_component, current_component in zip(goal_position, current_position)
            )
        )

        goal_yaw = self.yaw_from_orientation(self.startup_transition_goal.pose.orientation)
        current_yaw = self.yaw_from_orientation(self.latest_cog_pose.pose.orientation)
        yaw_error = abs(self.normalize_angle(goal_yaw - current_yaw))

        return position_error, yaw_error

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

    def flight_state_callback(self, msg):
        self.latest_flight_state = msg.data

    def copy_pose_message(self, pose_msg):
        copied_pose = PoseStamped()
        copied_pose.header.stamp = pose_msg.header.stamp
        copied_pose.header.frame_id = pose_msg.header.frame_id
        copied_pose.pose.position.x = pose_msg.pose.position.x
        copied_pose.pose.position.y = pose_msg.pose.position.y
        copied_pose.pose.position.z = pose_msg.pose.position.z
        copied_pose.pose.orientation.x = pose_msg.pose.orientation.x
        copied_pose.pose.orientation.y = pose_msg.pose.orientation.y
        copied_pose.pose.orientation.z = pose_msg.pose.orientation.z
        copied_pose.pose.orientation.w = pose_msg.pose.orientation.w
        return copied_pose

    def position_to_tuple(self, position):
        return (position.x, position.y, position.z)

    def orientation_to_tuple(self, orientation):
        return self.normalize_quaternion(
            (
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            )
        )

    def normalize_quaternion(self, quaternion):
        norm = math.sqrt(sum(component * component for component in quaternion))
        if norm < 1e-8:
            return (0.0, 0.0, 0.0, 1.0)

        return tuple(component / norm for component in quaternion)

    def quaternion_inverse(self, quaternion):
        normalized_quaternion = self.normalize_quaternion(quaternion)
        return (
            -normalized_quaternion[0],
            -normalized_quaternion[1],
            -normalized_quaternion[2],
            normalized_quaternion[3],
        )

    def quaternion_multiply(self, lhs, rhs):
        lhs = self.normalize_quaternion(lhs)
        rhs = self.normalize_quaternion(rhs)
        return self.normalize_quaternion(self.quaternion_multiply_raw(lhs, rhs))

    def quaternion_multiply_raw(self, lhs, rhs):
        return (
            lhs[3] * rhs[0] + lhs[0] * rhs[3] + lhs[1] * rhs[2] - lhs[2] * rhs[1],
            lhs[3] * rhs[1] - lhs[0] * rhs[2] + lhs[1] * rhs[3] + lhs[2] * rhs[0],
            lhs[3] * rhs[2] + lhs[0] * rhs[1] - lhs[1] * rhs[0] + lhs[2] * rhs[3],
            lhs[3] * rhs[3] - lhs[0] * rhs[0] - lhs[1] * rhs[1] - lhs[2] * rhs[2],
        )

    def rotate_vector(self, quaternion, vector):
        normalized_quaternion = self.normalize_quaternion(quaternion)
        vector_quaternion = (vector[0], vector[1], vector[2], 0.0)
        rotated_vector = self.quaternion_multiply_raw(
            self.quaternion_multiply_raw(normalized_quaternion, vector_quaternion),
            self.quaternion_inverse(quaternion),
        )
        return rotated_vector[:3]

    def subtract_vectors(self, lhs, rhs):
        return tuple(lhs_component - rhs_component for lhs_component, rhs_component in zip(lhs, rhs))

    def quaternion_from_yaw(self, yaw):
        half_yaw = 0.5 * yaw
        return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))

    def yaw_from_orientation(self, orientation):
        quaternion = self.orientation_to_tuple(orientation)
        siny_cosp = 2.0 * (quaternion[3] * quaternion[2] + quaternion[0] * quaternion[1])
        cosy_cosp = 1.0 - 2.0 * (quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2])
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

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

        rospy.loginfo("Press any key in this terminal to stop the repeating trajectory publisher")
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
