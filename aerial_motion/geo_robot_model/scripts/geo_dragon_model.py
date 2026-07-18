#!/usr/bin/env python3
"""Ideal four-link DRAGON model with instantaneous geometric state updates."""

import copy
import math
import threading

import numpy as np
import rospy
import tf.transformations as tf_trans
import tf2_ros
from aerial_robot_msgs.msg import FlightNav, FullStateTarget
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray


JOINT_NAMES = (
    "joint1_pitch",
    "joint1_yaw",
    "joint2_pitch",
    "joint2_yaw",
    "joint3_pitch",
    "joint3_yaw",
)
POSITION_MODES = (FlightNav.POS_MODE, FlightNav.POS_VEL_MODE)


def all_finite(values):
    return all(math.isfinite(float(value)) for value in values)


def vector3_param(name, default):
    """Read a finite three-element ROS parameter with a safe fallback."""
    try:
        vector = np.asarray(rospy.get_param(name, default), dtype=float)
    except (TypeError, ValueError):
        vector = np.asarray([], dtype=float)
    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        rospy.logwarn("Invalid %s; using %s.", name, default)
        return np.asarray(default, dtype=float)
    return vector


def normalized_quaternion(message):
    quaternion = np.array(
        [message.x, message.y, message.z, message.w], dtype=float
    )
    norm = np.linalg.norm(quaternion)
    if not np.all(np.isfinite(quaternion)) or norm < 1.0e-9:
        return None
    return quaternion / norm


def rotation_y(angle):
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return np.array(
        [[cosine, 0.0, sine], [0.0, 1.0, 0.0], [-sine, 0.0, cosine]],
        dtype=float,
    )


def rotation_z(angle):
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return np.array(
        [[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]],
        dtype=float,
    )


def quaternion_rotating_z_to(direction):
    direction = np.asarray(direction, dtype=float)
    norm = np.linalg.norm(direction)
    if norm < 1.0e-9:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)

    target = direction / norm
    source = np.array([0.0, 0.0, 1.0], dtype=float)
    dot = float(np.dot(source, target))
    if dot < -1.0 + 1.0e-9:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)

    quaternion = np.concatenate((np.cross(source, target), [1.0 + dot]))
    return quaternion / np.linalg.norm(quaternion)


class DragonGeoRobotModel:
    def __init__(self):
        self.world_frame_id = rospy.get_param("~world_frame_id", "world")
        self.cog_frame_id = rospy.get_param("~cog_frame_id", "dragon/cog")
        self.root_frame_id = rospy.get_param("~root_frame_id", "dragon/root")
        self.link2_frame_id = rospy.get_param("~link2_frame_id", "dragon/link2")
        self.livox_frame_id = rospy.get_param(
            "~livox_frame_id", "dragon/livox_mid360"
        )
        self.lidar_frame_id = rospy.get_param(
            "~lidar_frame_id", "dragon/lidar_imu"
        )
        self.publish_tf = bool(rospy.get_param("~publish_tf", True))
        self.publish_rate = float(rospy.get_param("~publish_rate", 40.0))
        self.link_length = float(rospy.get_param("~link_length", 0.5255))
        self.link_diameter = float(rospy.get_param("~link_diameter", 0.04))
        self.joint_diameter = float(rospy.get_param("~joint_diameter", 0.06))
        self.link_body_length = float(rospy.get_param("~link_body_length", 0.474))
        self.inter_joint_length = float(
            rospy.get_param("~inter_joint_length", 0.0515)
        )
        self.link2_to_livox_xyz = vector3_param(
            "~link2_to_livox_mid360_xyz", [0.132, 0.0, 0.05592]
        )
        self.link2_to_livox_rpy = vector3_param(
            "~link2_to_livox_mid360_rpy", [0.0, 0.0, 0.0]
        )
        self.livox_to_lidar_xyz = vector3_param(
            "~livox_mid360_to_lidar_imu_xyz", [0.011, 0.02329, -0.02304]
        )
        self.livox_to_lidar_rpy = vector3_param(
            "~livox_mid360_to_lidar_imu_rpy", [0.0, 0.0, 0.0]
        )

        if self.link_length <= 0.0:
            rospy.logwarn("Invalid link_length %.6f; using 0.5255 m.", self.link_length)
            self.link_length = 0.5255
        if self.link_body_length <= 0.0:
            rospy.logwarn(
                "Invalid link_body_length %.6f; using 0.474 m.",
                self.link_body_length,
            )
            self.link_body_length = 0.474
        if self.inter_joint_length < 0.0:
            rospy.logwarn(
                "Invalid inter_joint_length %.6f; using 0.0515 m.",
                self.inter_joint_length,
            )
            self.inter_joint_length = 0.0515

        spawn_x = float(rospy.get_param("~spawn_x", 0.0))
        spawn_y = float(rospy.get_param("~spawn_y", 0.0))
        spawn_z = float(rospy.get_param("~spawn_z", 1.0))
        spawn_yaw = float(rospy.get_param("~spawn_yaw", 0.0))
        initial_joints = rospy.get_param(
            "~initial_joints", [0.0, math.pi / 2.0] * 3
        )
        if len(initial_joints) != len(JOINT_NAMES) or not all_finite(initial_joints):
            rospy.logwarn("Invalid initial_joints; using the square folded pose.")
            initial_joints = [0.0, math.pi / 2.0] * 3

        self.lock = threading.Lock()
        self.root_position = np.array([spawn_x, spawn_y, spawn_z], dtype=float)
        self.root_orientation = np.asarray(
            tf_trans.quaternion_from_euler(0.0, 0.0, spawn_yaw), dtype=float
        )
        self.root_twist = Twist()
        self.joint_positions = np.asarray(initial_joints, dtype=float)

        self.odom_pub = rospy.Publisher("uav/cog/odom", Odometry, queue_size=10)
        self.joint_state_pub = rospy.Publisher(
            "joint_states", JointState, queue_size=10
        )
        self.root_pose_pub = rospy.Publisher(
            "root/pose", PoseStamped, queue_size=10
        )
        self.root_tail_pose_pub = rospy.Publisher(
            "root/tail_pose", PoseStamped, queue_size=10
        )
        self.marker_pub = rospy.Publisher(
            "robot_markers", MarkerArray, queue_size=1
        )

        self.nav_sub = rospy.Subscriber(
            "uav/nav", FlightNav, self.nav_callback, queue_size=1, tcp_nodelay=True
        )
        self.full_state_sub = rospy.Subscriber(
            "full_state_target",
            FullStateTarget,
            self.full_state_callback,
            queue_size=1,
            tcp_nodelay=True,
        )
        self.joint_control_sub = rospy.Subscriber(
            "joints_ctrl",
            JointState,
            self.joint_control_callback,
            queue_size=1,
            tcp_nodelay=True,
        )
        self.root_target_sub = rospy.Subscriber(
            "root/target_pose",
            PoseStamped,
            self.root_target_callback,
            queue_size=1,
            tcp_nodelay=True,
        )

        self.tf_broadcaster = tf2_ros.TransformBroadcaster() if self.publish_tf else None
        self.static_tf_broadcaster = (
            tf2_ros.StaticTransformBroadcaster() if self.publish_tf else None
        )
        if self.static_tf_broadcaster is not None:
            self.publish_lidar_mount_transforms()
        publish_rate = self.publish_rate if self.publish_rate > 0.0 else 40.0
        self.timer = rospy.Timer(
            rospy.Duration.from_sec(1.0 / publish_rate), self.timer_callback
        )

        rospy.loginfo(
            "DRAGON geometric model is ready with %.4f m link spacing, "
            "non-actuating root/target_pose input, and the Livox mounted on %s.",
            self.link_length,
            self.link2_frame_id,
        )

    @staticmethod
    def set_point(message, vector):
        message.x = float(vector[0])
        message.y = float(vector[1])
        message.z = float(vector[2])

    @staticmethod
    def set_quaternion(message, quaternion):
        message.x = float(quaternion[0])
        message.y = float(quaternion[1])
        message.z = float(quaternion[2])
        message.w = float(quaternion[3])

    @staticmethod
    def pose_state(message):
        position = np.array(
            [message.position.x, message.position.y, message.position.z], dtype=float
        )
        quaternion = normalized_quaternion(message.orientation)
        if not np.all(np.isfinite(position)) or quaternion is None:
            return None
        return position, quaternion

    @staticmethod
    def twist_is_finite(message):
        return all_finite(
            (
                message.linear.x,
                message.linear.y,
                message.linear.z,
                message.angular.x,
                message.angular.y,
                message.angular.z,
            )
        )

    def complete_joint_vector(self, message, source_name):
        if len(message.position) != len(JOINT_NAMES):
            rospy.logwarn_throttle(
                1.0,
                "Ignoring %s with %d joint positions; expected %d.",
                source_name,
                len(message.position),
                len(JOINT_NAMES),
            )
            return None
        if not all_finite(message.position):
            rospy.logwarn_throttle(
                1.0, "Ignoring %s with non-finite joint positions.", source_name
            )
            return None

        if not message.name:
            return np.asarray(message.position, dtype=float)

        if len(message.name) != len(JOINT_NAMES) or set(message.name) != set(
            JOINT_NAMES
        ):
            rospy.logwarn_throttle(
                1.0,
                "Ignoring %s without one value for every DRAGON link joint.",
                source_name,
            )
            return None

        index_by_name = {name: index for index, name in enumerate(message.name)}
        return np.asarray(
            [message.position[index_by_name[name]] for name in JOINT_NAMES], dtype=float
        )

    def nav_callback(self, message):
        active_values = []
        if message.pos_xy_nav_mode in POSITION_MODES:
            active_values.extend((message.target_pos_x, message.target_pos_y))
        if message.pos_z_nav_mode in POSITION_MODES:
            active_values.append(message.target_pos_z)
        if message.yaw_nav_mode in POSITION_MODES:
            active_values.append(message.target_yaw)
        if not all_finite(active_values):
            rospy.logwarn_throttle(1.0, "Ignoring FlightNav with non-finite targets.")
            return

        changed = False
        with self.lock:
            candidate_position = self.root_position.copy()
            candidate_orientation = self.root_orientation.copy()

            if message.yaw_nav_mode in POSITION_MODES:
                roll, pitch, _ = tf_trans.euler_from_quaternion(candidate_orientation)
                candidate_orientation = np.asarray(
                    tf_trans.quaternion_from_euler(roll, pitch, message.target_yaw),
                    dtype=float,
                )
                changed = True

            if (
                message.pos_xy_nav_mode in POSITION_MODES
                or message.pos_z_nav_mode in POSITION_MODES
            ):
                _, geometric_cog = self.compute_geometry(
                    candidate_position, candidate_orientation, self.joint_positions
                )
                if message.pos_xy_nav_mode in POSITION_MODES:
                    candidate_position[0] += message.target_pos_x - geometric_cog[0]
                    candidate_position[1] += message.target_pos_y - geometric_cog[1]
                if message.pos_z_nav_mode in POSITION_MODES:
                    candidate_position[2] += message.target_pos_z - geometric_cog[2]
                changed = True

            if changed:
                self.root_position = candidate_position
                self.root_orientation = candidate_orientation
                self.root_twist = Twist()

        if changed:
            self.publish_state()

    def full_state_callback(self, message):
        pose_state = self.pose_state(message.root_state.pose.pose)
        joint_positions = self.complete_joint_vector(
            message.joint_state, "full_state_target"
        )
        if pose_state is None:
            rospy.logwarn_throttle(
                1.0, "Ignoring full_state_target with an invalid root pose."
            )
            return
        if joint_positions is None:
            return
        if not self.twist_is_finite(message.root_state.twist.twist):
            rospy.logwarn_throttle(
                1.0, "Ignoring full_state_target with a non-finite root twist."
            )
            return

        with self.lock:
            self.root_position, self.root_orientation = pose_state
            self.root_twist = copy.deepcopy(message.root_state.twist.twist)
            self.joint_positions = joint_positions
        self.publish_state()

    def joint_control_callback(self, message):
        if not message.name:
            joint_positions = self.complete_joint_vector(message, "joints_ctrl")
            if joint_positions is None:
                return
            with self.lock:
                self.joint_positions = joint_positions
            self.publish_state()
            return

        if len(message.name) != len(message.position) or not all_finite(
            message.position
        ):
            rospy.logwarn_throttle(
                1.0, "Ignoring invalid named joints_ctrl command."
            )
            return

        recognized = []
        seen_names = set()
        for name, position in zip(message.name, message.position):
            if name not in JOINT_NAMES:
                rospy.logwarn_throttle(
                    1.0, "Ignoring unknown joints_ctrl entry '%s'.", name
                )
                continue
            if name in seen_names:
                rospy.logwarn_throttle(
                    1.0, "Ignoring joints_ctrl with duplicate entry '%s'.", name
                )
                return
            seen_names.add(name)
            recognized.append((JOINT_NAMES.index(name), float(position)))

        if not recognized:
            return

        with self.lock:
            updated_positions = self.joint_positions.copy()
            for index, position in recognized:
                updated_positions[index] = position
            self.joint_positions = updated_positions
        self.publish_state()

    def root_target_callback(self, _message):
        # Keep the planner-facing topic connected without bypassing the
        # whole-body command generated on full_state_target.
        return

    def compute_geometry(self, root_position, root_orientation, joint_positions):
        rotation = tf_trans.quaternion_matrix(root_orientation)[:3, :3]
        position = np.asarray(root_position, dtype=float).copy()
        endpoints = [position.copy()]

        for link_index in range(4):
            position = position + self.link_length * rotation[:, 0]
            endpoints.append(position.copy())
            if link_index < 3:
                pitch = float(joint_positions[2 * link_index])
                yaw = float(joint_positions[2 * link_index + 1])
                rotation = rotation @ rotation_y(pitch) @ rotation_z(yaw)

        endpoints = np.asarray(endpoints, dtype=float)
        link_midpoints = 0.5 * (endpoints[:-1] + endpoints[1:])
        return endpoints, np.mean(link_midpoints, axis=0)

    def compute_link2_pose(self, root_position, root_orientation, joint_positions):
        """Compute the v1_5 URDF link2 pose, including the articulated joint gap."""
        link1_rotation = tf_trans.quaternion_matrix(root_orientation)[:3, :3]
        pitch_rotation = link1_rotation @ rotation_y(float(joint_positions[0]))
        link2_position = (
            np.asarray(root_position, dtype=float)
            + self.link_body_length * link1_rotation[:, 0]
            + self.inter_joint_length * pitch_rotation[:, 0]
        )
        link2_rotation = pitch_rotation @ rotation_z(float(joint_positions[1]))
        return link2_position, link2_rotation

    def timer_callback(self, _event):
        self.publish_state()

    def publish_state(self):
        with self.lock:
            root_position = self.root_position.copy()
            root_orientation = self.root_orientation.copy()
            root_twist = copy.deepcopy(self.root_twist)
            joint_positions = self.joint_positions.copy()

        endpoints, geometric_cog = self.compute_geometry(
            root_position, root_orientation, joint_positions
        )
        link2_position, link2_rotation = self.compute_link2_pose(
            root_position, root_orientation, joint_positions
        )
        stamp = rospy.Time.now()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.world_frame_id
        odom.child_frame_id = self.cog_frame_id
        self.set_point(odom.pose.pose.position, geometric_cog)
        self.set_quaternion(odom.pose.pose.orientation, root_orientation)
        odom.twist.twist = root_twist
        self.odom_pub.publish(odom)

        joint_state = JointState()
        joint_state.header.stamp = stamp
        joint_state.header.frame_id = self.root_frame_id
        joint_state.name = list(JOINT_NAMES)
        joint_state.position = joint_positions.tolist()
        joint_state.velocity = [0.0] * len(JOINT_NAMES)
        joint_state.effort = [0.0] * len(JOINT_NAMES)
        self.joint_state_pub.publish(joint_state)

        root_pose = PoseStamped()
        root_pose.header.stamp = stamp
        root_pose.header.frame_id = self.world_frame_id
        self.set_point(root_pose.pose.position, root_position)
        self.set_quaternion(root_pose.pose.orientation, root_orientation)
        self.root_pose_pub.publish(root_pose)

        root_tail_pose = PoseStamped()
        root_tail_pose.header.stamp = stamp
        root_tail_pose.header.frame_id = self.world_frame_id
        self.set_point(root_tail_pose.pose.position, endpoints[1])
        self.set_quaternion(root_tail_pose.pose.orientation, root_orientation)
        self.root_tail_pose_pub.publish(root_tail_pose)

        self.marker_pub.publish(self.build_markers(stamp, endpoints))
        self.publish_transforms(
            stamp,
            root_position,
            root_orientation,
            link2_position,
            link2_rotation,
            geometric_cog,
        )

    def build_markers(self, stamp, endpoints):
        marker_array = MarkerArray()
        for link_index in range(4):
            start = endpoints[link_index]
            end = endpoints[link_index + 1]
            segment = end - start

            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = self.world_frame_id
            marker.ns = "dragon_links"
            marker.id = link_index
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            self.set_point(marker.pose.position, 0.5 * (start + end))
            self.set_quaternion(
                marker.pose.orientation, quaternion_rotating_z_to(segment)
            )
            marker.scale.x = self.link_diameter
            marker.scale.y = self.link_diameter
            marker.scale.z = float(np.linalg.norm(segment))
            if link_index == 0:
                marker.color.r = 0.95
                marker.color.g = 0.35
                marker.color.b = 0.10
            else:
                marker.color.r = 0.15
                marker.color.g = 0.45
                marker.color.b = 0.90
            marker.color.a = 0.95
            marker_array.markers.append(marker)

        for joint_index, endpoint in enumerate(endpoints):
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = self.world_frame_id
            marker.ns = "dragon_joints"
            marker.id = joint_index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            self.set_point(marker.pose.position, endpoint)
            marker.pose.orientation.w = 1.0
            marker.scale.x = self.joint_diameter
            marker.scale.y = self.joint_diameter
            marker.scale.z = self.joint_diameter
            marker.color.r = 0.15
            marker.color.g = 0.15
            marker.color.b = 0.15
            marker.color.a = 1.0
            marker_array.markers.append(marker)

        return marker_array

    def make_fixed_transform(self, parent_frame, child_frame, xyz, rpy, stamp):
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        self.set_point(transform.transform.translation, xyz)
        self.set_quaternion(
            transform.transform.rotation, tf_trans.quaternion_from_euler(*rpy)
        )
        return transform

    def publish_lidar_mount_transforms(self):
        stamp = rospy.Time.now()
        link2_to_livox = self.make_fixed_transform(
            self.link2_frame_id,
            self.livox_frame_id,
            self.link2_to_livox_xyz,
            self.link2_to_livox_rpy,
            stamp,
        )
        livox_to_lidar = self.make_fixed_transform(
            self.livox_frame_id,
            self.lidar_frame_id,
            self.livox_to_lidar_xyz,
            self.livox_to_lidar_rpy,
            stamp,
        )
        self.static_tf_broadcaster.sendTransform([link2_to_livox, livox_to_lidar])

    def publish_transforms(
        self,
        stamp,
        root_position,
        root_orientation,
        link2_position,
        link2_rotation,
        geometric_cog,
    ):
        if self.tf_broadcaster is None:
            return

        root_transform = TransformStamped()
        root_transform.header.stamp = stamp
        root_transform.header.frame_id = self.world_frame_id
        root_transform.child_frame_id = self.root_frame_id
        self.set_point(root_transform.transform.translation, root_position)
        self.set_quaternion(root_transform.transform.rotation, root_orientation)

        cog_transform = TransformStamped()
        cog_transform.header.stamp = stamp
        cog_transform.header.frame_id = self.world_frame_id
        cog_transform.child_frame_id = self.cog_frame_id
        self.set_point(cog_transform.transform.translation, geometric_cog)
        self.set_quaternion(cog_transform.transform.rotation, root_orientation)

        link2_transform = TransformStamped()
        link2_transform.header.stamp = stamp
        link2_transform.header.frame_id = self.world_frame_id
        link2_transform.child_frame_id = self.link2_frame_id
        self.set_point(link2_transform.transform.translation, link2_position)
        link2_matrix = np.identity(4)
        link2_matrix[:3, :3] = link2_rotation
        self.set_quaternion(
            link2_transform.transform.rotation,
            tf_trans.quaternion_from_matrix(link2_matrix),
        )

        self.tf_broadcaster.sendTransform(
            [root_transform, cog_transform, link2_transform]
        )


if __name__ == "__main__":
    rospy.init_node("geo_dragon_model")
    DragonGeoRobotModel()
    rospy.spin()
