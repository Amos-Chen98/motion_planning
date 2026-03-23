#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState


class DragonDanceSupportMixin:
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

    def build_shutdown_hold_cog_target(self, yaw):
        goal_msg = PoseStamped()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = self.FRAME_ID
        goal_msg.pose.position.x = self.latest_cog_pose.pose.position.x
        goal_msg.pose.position.y = self.latest_cog_pose.pose.position.y
        goal_msg.pose.position.z = self.latest_cog_pose.pose.position.z

        hold_orientation = self.quaternion_from_yaw(yaw)
        goal_msg.pose.orientation.x = hold_orientation[0]
        goal_msg.pose.orientation.y = hold_orientation[1]
        goal_msg.pose.orientation.z = hold_orientation[2]
        goal_msg.pose.orientation.w = hold_orientation[3]
        return goal_msg

    def build_shutdown_joint_target(self):
        goal_msg = JointState()
        goal_msg.name = list(self.SHUTDOWN_JOINT_NAMES)
        goal_msg.position = list(self.SHUTDOWN_JOINT_POSITIONS)
        return goal_msg

    def build_shutdown_leveling_rotation_target(self, elapsed):
        goal_msg = Odometry()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = self.ROTATION_FRAME_ID

        progress = 1.0
        if self.shutdown_leveling_publish_duration > 1e-6:
            progress = max(0.0, min(1.0, elapsed / self.shutdown_leveling_publish_duration))

        # Smoothstep avoids commanding a sudden orientation jump at either end.
        blend = progress * progress * (3.0 - 2.0 * progress)
        start_roll, start_pitch, start_yaw = self.shutdown_leveling_start_rpy
        target_roll = (1.0 - blend) * start_roll
        target_pitch = (1.0 - blend) * start_pitch
        target_yaw = start_yaw

        level_orientation = self.quaternion_from_rpy(target_roll, target_pitch, target_yaw)
        goal_msg.pose.pose.orientation.x = level_orientation[0]
        goal_msg.pose.pose.orientation.y = level_orientation[1]
        goal_msg.pose.pose.orientation.z = level_orientation[2]
        goal_msg.pose.pose.orientation.w = level_orientation[3]
        goal_msg.twist.twist.angular.x = 0.0
        goal_msg.twist.twist.angular.y = 0.0
        goal_msg.twist.twist.angular.z = 0.0
        return goal_msg

    def publish_shutdown_leveling_commands(self, elapsed):
        hold_goal = self.copy_pose_message(self.shutdown_hold_cog_target)
        hold_goal.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(self.shutdown_cog_hold_duration)
        hold_goal.header.frame_id = self.FRAME_ID
        self.cog_target_publisher.publish(hold_goal)

        rotation_goal = self.build_shutdown_leveling_rotation_target(elapsed)
        self.rotation_target_publisher.publish(rotation_goal)

        rospy.loginfo_throttle(
            1.0,
            "Published shutdown CoG hold target on %s and interpolated leveling target on %s at [x=%.2f, y=%.2f, z=%.2f] with target [roll=%.2f, pitch=%.2f, yaw=%.2f] rad",
            self.resolved_cog_target_topic,
            self.resolved_rotation_target_topic,
            hold_goal.pose.position.x,
            hold_goal.pose.position.y,
            hold_goal.pose.position.z,
            *self.rpy_from_orientation(rotation_goal.pose.pose.orientation),
        )

    def publish_shutdown_joint_shape_commands(self):
        hold_goal = self.copy_pose_message(self.shutdown_hold_cog_target)
        hold_goal.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(self.shutdown_cog_hold_duration)
        hold_goal.header.frame_id = self.FRAME_ID
        self.cog_target_publisher.publish(hold_goal)

        joint_goal = self.copy_joint_state_message(self.shutdown_joint_target)
        joint_goal.header.stamp = rospy.Time.now()
        self.joint_target_publisher.publish(joint_goal)

    def compute_shutdown_joint_error(self):
        if self.latest_joint_state is None:
            return None

        joint_positions = {
            name: position
            for name, position in zip(self.latest_joint_state.name, self.latest_joint_state.position)
        }
        if not all(joint_name in joint_positions for joint_name in self.SHUTDOWN_JOINT_NAMES):
            return None

        return max(
            abs(joint_positions[joint_name] - joint_target)
            for joint_name, joint_target in zip(self.SHUTDOWN_JOINT_NAMES, self.SHUTDOWN_JOINT_POSITIONS)
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

    def copy_joint_state_message(self, joint_msg):
        copied_joint = JointState()
        copied_joint.header = joint_msg.header
        copied_joint.name = list(joint_msg.name)
        copied_joint.position = list(joint_msg.position)
        copied_joint.velocity = list(joint_msg.velocity)
        copied_joint.effort = list(joint_msg.effort)
        return copied_joint

    def copy_odometry_message(self, odom_msg):
        copied_odom = Odometry()
        copied_odom.header.stamp = odom_msg.header.stamp
        copied_odom.header.frame_id = odom_msg.header.frame_id
        copied_odom.child_frame_id = odom_msg.child_frame_id
        copied_odom.pose.pose.position.x = odom_msg.pose.pose.position.x
        copied_odom.pose.pose.position.y = odom_msg.pose.pose.position.y
        copied_odom.pose.pose.position.z = odom_msg.pose.pose.position.z
        copied_odom.pose.pose.orientation.x = odom_msg.pose.pose.orientation.x
        copied_odom.pose.pose.orientation.y = odom_msg.pose.pose.orientation.y
        copied_odom.pose.pose.orientation.z = odom_msg.pose.pose.orientation.z
        copied_odom.pose.pose.orientation.w = odom_msg.pose.pose.orientation.w
        copied_odom.twist.twist.linear.x = odom_msg.twist.twist.linear.x
        copied_odom.twist.twist.linear.y = odom_msg.twist.twist.linear.y
        copied_odom.twist.twist.linear.z = odom_msg.twist.twist.linear.z
        copied_odom.twist.twist.angular.x = odom_msg.twist.twist.angular.x
        copied_odom.twist.twist.angular.y = odom_msg.twist.twist.angular.y
        copied_odom.twist.twist.angular.z = odom_msg.twist.twist.angular.z
        return copied_odom

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

    def quaternion_from_rpy(self, roll, pitch, yaw):
        half_roll = 0.5 * roll
        half_pitch = 0.5 * pitch
        half_yaw = 0.5 * yaw

        sin_roll = math.sin(half_roll)
        cos_roll = math.cos(half_roll)
        sin_pitch = math.sin(half_pitch)
        cos_pitch = math.cos(half_pitch)
        sin_yaw = math.sin(half_yaw)
        cos_yaw = math.cos(half_yaw)

        return self.normalize_quaternion(
            (
                sin_roll * cos_pitch * cos_yaw - cos_roll * sin_pitch * sin_yaw,
                cos_roll * sin_pitch * cos_yaw + sin_roll * cos_pitch * sin_yaw,
                cos_roll * cos_pitch * sin_yaw - sin_roll * sin_pitch * cos_yaw,
                cos_roll * cos_pitch * cos_yaw + sin_roll * sin_pitch * sin_yaw,
            )
        )

    def rpy_from_orientation(self, orientation):
        quaternion = self.orientation_to_tuple(orientation)
        sinr_cosp = 2.0 * (quaternion[3] * quaternion[0] + quaternion[1] * quaternion[2])
        cosr_cosp = 1.0 - 2.0 * (quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1])
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (quaternion[3] * quaternion[1] - quaternion[2] * quaternion[0])
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)

        siny_cosp = 2.0 * (quaternion[3] * quaternion[2] + quaternion[0] * quaternion[1])
        cosy_cosp = 1.0 - 2.0 * (quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2])
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return (roll, pitch, yaw)

    def yaw_from_orientation(self, orientation):
        return self.rpy_from_orientation(orientation)[2]

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))
