#!/usr/bin/env python3
"""Ideal four-link DRAGON model with instantaneous geometric state updates."""

import copy
import math
import threading
import xml.etree.ElementTree as ET

import numpy as np
import rospy
import tf.transformations as tf_trans
import tf2_ros
from aerial_robot_msgs.msg import FlightNav, FullStateTarget
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray


LINK_COUNT = 4
JOINT_NAMES = tuple(
    f"joint{index}_{axis}"
    for index in range(1, LINK_COUNT)
    for axis in ("pitch", "yaw")
)
GIMBAL_JOINT_NAMES = tuple(
    f"gimbal{index}_{axis}"
    for index in range(1, LINK_COUNT + 1)
    for axis in ("roll", "pitch")
)
ROTOR_JOINT_NAMES = tuple(f"rotor{index}" for index in range(1, LINK_COUNT + 1))
AUXILIARY_JOINT_NAMES = GIMBAL_JOINT_NAMES + ROTOR_JOINT_NAMES
URDF_JOINT_NAMES = JOINT_NAMES + AUXILIARY_JOINT_NAMES
JOINT_INDEX = {name: index for index, name in enumerate(JOINT_NAMES)}
POSITION_MODES = (FlightNav.POS_MODE, FlightNav.POS_VEL_MODE)
DEFAULT_LINK_LENGTH = 0.5255
DEFAULT_PUBLISH_RATE = 40.0
DEFAULT_JOINT_POSITIONS = (0.0, math.pi / 2.0) * (LINK_COUNT - 1)
IDENTITY_QUATERNION = np.array([0.0, 0.0, 0.0, 1.0])
LINK_COLORS = ((0.95, 0.35, 0.10), (0.15, 0.45, 0.90))
JOINT_COLORS = ((1.0, 0.10, 0.05), (0.15, 0.15, 0.15))


def all_finite(values):
    return all(math.isfinite(float(value)) for value in values)


def normalized_quaternion(message):
    quaternion = np.array(
        [message.x, message.y, message.z, message.w], dtype=float
    )
    norm = np.linalg.norm(quaternion)
    if not np.all(np.isfinite(quaternion)) or norm < 1.0e-9:
        return None
    return quaternion / norm


def joint_rotation(pitch, yaw):
    cos_pitch, sin_pitch = math.cos(pitch), math.sin(pitch)
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cos_pitch * cos_yaw, -cos_pitch * sin_yaw, sin_pitch],
            [sin_yaw, cos_yaw, 0.0],
            [-sin_pitch * cos_yaw, sin_pitch * sin_yaw, cos_pitch],
        ],
        dtype=float,
    )


def xml_vector(element, attribute, default):
    value = default if element is None else element.get(attribute, default)
    return np.fromstring(value, sep=" ")


def rotation_from_quaternion(quaternion):
    return tf_trans.quaternion_matrix(quaternion)[:3, :3]


def quaternion_from_rotation(rotation):
    homogeneous_rotation = np.identity(4)
    homogeneous_rotation[:3, :3] = rotation
    return np.asarray(
        tf_trans.quaternion_from_matrix(homogeneous_rotation), dtype=float
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
        publish_rate = float(rospy.get_param("~publish_rate", DEFAULT_PUBLISH_RATE))
        self.link_length = float(
            rospy.get_param("~link_length", DEFAULT_LINK_LENGTH)
        )
        self.link_diameter = float(rospy.get_param("~link_diameter", 0.04))
        self.joint_diameter = float(rospy.get_param("~joint_diameter", 0.06))

        if self.link_length <= 0.0:
            rospy.logwarn(
                "Invalid link_length %.6f; using %.4f m.",
                self.link_length,
                DEFAULT_LINK_LENGTH,
            )
            self.link_length = DEFAULT_LINK_LENGTH

        spawn_x = float(rospy.get_param("~spawn_x", 0.0))
        spawn_y = float(rospy.get_param("~spawn_y", 0.0))
        spawn_z = float(rospy.get_param("~spawn_z", 1.0))
        spawn_yaw = float(rospy.get_param("~spawn_yaw", 0.0))
        initial_joints = rospy.get_param("~initial_joints", DEFAULT_JOINT_POSITIONS)
        if len(initial_joints) != len(JOINT_NAMES) or not all_finite(initial_joints):
            rospy.logwarn("Invalid initial_joints; using the square folded pose.")
            initial_joints = DEFAULT_JOINT_POSITIONS

        self.baselink_chain = self.load_baselink_chain(
            rospy.get_param("~baselink_link_name", "link2")
        )

        self.lock = threading.Lock()
        self.root_position = np.array([spawn_x, spawn_y, spawn_z], dtype=float)
        self.root_orientation = np.asarray(
            tf_trans.quaternion_from_euler(0.0, 0.0, spawn_yaw), dtype=float
        )
        self.cog_orientation = self.root_orientation.copy()
        self.root_twist = Twist()
        self.joint_positions = np.asarray(initial_joints, dtype=float)

        self.odom_pub = rospy.Publisher("uav/cog/odom", Odometry, queue_size=10)
        self.joint_state_pub = rospy.Publisher(
            "joint_states", JointState, queue_size=10
        )
        self.root_pose_pub = rospy.Publisher("root/pose", PoseStamped, queue_size=10)
        self.root_tail_pose_pub = rospy.Publisher(
            "root/tail_pose", PoseStamped, queue_size=10
        )
        self.marker_pub = rospy.Publisher("robot_markers", MarkerArray, queue_size=1)

        def subscribe(topic, message_type, callback):
            return rospy.Subscriber(
                topic, message_type, callback, queue_size=1, tcp_nodelay=True
            )

        self.subscribers = [
            subscribe("uav/nav", FlightNav, self.nav_callback),
            subscribe("full_state_target", FullStateTarget, self.full_state_callback),
            subscribe("joints_ctrl", JointState, self.joint_control_callback),
            subscribe("root/target_pose", PoseStamped, self.root_target_callback),
        ]

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        publish_rate = publish_rate if publish_rate > 0.0 else DEFAULT_PUBLISH_RATE
        self.timer = rospy.Timer(
            rospy.Duration.from_sec(1.0 / publish_rate), self.timer_callback
        )

        rospy.loginfo(
            "DRAGON geometric model is ready with %.4f m link spacing, "
            "non-actuating root/target_pose input, and URDF-managed link TF.",
            self.link_length,
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
    def set_color(message, color, alpha=1.0):
        message.r, message.g, message.b = color
        message.a = alpha

    def create_marker(self, stamp, namespace, marker_id, marker_type):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.world_frame_id
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        return marker

    def make_pose(self, stamp, position, orientation):
        message = PoseStamped()
        message.header.stamp = stamp
        message.header.frame_id = self.world_frame_id
        self.set_point(message.pose.position, position)
        self.set_quaternion(message.pose.orientation, orientation)
        return message

    def make_transform(self, stamp, child_frame_id, position, orientation):
        message = TransformStamped()
        message.header.stamp = stamp
        message.header.frame_id = self.world_frame_id
        message.child_frame_id = child_frame_id
        self.set_point(message.transform.translation, position)
        self.set_quaternion(message.transform.rotation, orientation)
        return message

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

    def load_baselink_chain(self, baselink_name):
        try:
            robot_element = ET.fromstring(rospy.get_param("robot_description"))
            joint_by_child_link = {}
            for joint_element in robot_element.findall("joint"):
                child_element = joint_element.find("child")
                if child_element is not None:
                    joint_by_child_link[child_element.get("link")] = joint_element

            chain_elements = []
            current_link_name = baselink_name
            while current_link_name in joint_by_child_link:
                joint_element = joint_by_child_link[current_link_name]
                chain_elements.append(joint_element)
                current_link_name = joint_element.find("parent").get("link")
            if not chain_elements:
                raise ValueError("No root-to-{} joints found.".format(baselink_name))

            chain = []
            for joint_element in reversed(chain_elements):
                name = joint_element.get("name")
                joint_type = joint_element.get("type")
                if joint_type == "fixed":
                    joint_index = None
                    axis = None
                elif joint_type in ("revolute", "continuous") and name in JOINT_INDEX:
                    joint_index = JOINT_INDEX[name]
                    axis = xml_vector(joint_element.find("axis"), "xyz", "1 0 0")
                else:
                    raise ValueError(
                        "Unsupported {} joint '{}' in root-to-baselink chain.".format(
                            joint_type, name
                        )
                    )

                origin = joint_element.find("origin")
                chain.append(
                    (
                        xml_vector(origin, "xyz", "0 0 0"),
                        tf_trans.euler_matrix(
                            *xml_vector(origin, "rpy", "0 0 0")
                        )[:3, :3],
                        joint_index,
                        axis,
                    )
                )

            rospy.loginfo(
                "Loaded the %s-to-%s kinematic chain from robot_description.",
                current_link_name,
                baselink_name,
            )
            return chain
        except Exception as error:
            rospy.logwarn(
                "Could not load the root-to-%s kinematic chain (%s); "
                "falling back to ideal link geometry.",
                baselink_name,
                error,
            )
            return None

    def nav_callback(self, message):
        xy_active = message.pos_xy_nav_mode in POSITION_MODES
        z_active = message.pos_z_nav_mode in POSITION_MODES
        yaw_active = message.yaw_nav_mode in POSITION_MODES
        active_values = []
        if xy_active:
            active_values.extend((message.target_pos_x, message.target_pos_y))
        if z_active:
            active_values.append(message.target_pos_z)
        if yaw_active:
            active_values.append(message.target_yaw)
        if not all_finite(active_values):
            rospy.logwarn_throttle(1.0, "Ignoring FlightNav with non-finite targets.")
            return
        if not (xy_active or z_active or yaw_active):
            return

        with self.lock:
            candidate_position = self.root_position.copy()
            candidate_orientation = self.root_orientation.copy()
            candidate_cog_orientation = self.cog_orientation.copy()

            if yaw_active:
                fixed_cog_position = self.compute_geometry(
                    candidate_position,
                    candidate_orientation,
                    self.joint_positions,
                )[1]
                previous_cog_rotation = rotation_from_quaternion(
                    candidate_cog_orientation
                )
                roll, pitch, _ = tf_trans.euler_from_quaternion(
                    candidate_cog_orientation
                )
                candidate_cog_orientation = np.asarray(
                    tf_trans.quaternion_from_euler(roll, pitch, message.target_yaw),
                    dtype=float,
                )
                updated_cog_rotation = rotation_from_quaternion(
                    candidate_cog_orientation
                )
                candidate_orientation = quaternion_from_rotation(
                    updated_cog_rotation
                    @ previous_cog_rotation.T
                    @ rotation_from_quaternion(candidate_orientation)
                )
                rotated_cog_position = self.compute_geometry(
                    candidate_position,
                    candidate_orientation,
                    self.joint_positions,
                )[1]
                candidate_position += fixed_cog_position - rotated_cog_position

            if xy_active or z_active:
                geometric_cog = self.compute_geometry(
                    candidate_position, candidate_orientation, self.joint_positions
                )[1]
                if xy_active:
                    candidate_position[0] += message.target_pos_x - geometric_cog[0]
                    candidate_position[1] += message.target_pos_y - geometric_cog[1]
                if z_active:
                    candidate_position[2] += message.target_pos_z - geometric_cog[2]

            self.root_position = candidate_position
            self.root_orientation = candidate_orientation
            self.cog_orientation = candidate_cog_orientation
            self.root_twist = Twist()

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
            self.cog_orientation = self.root_orientation.copy()
            self.root_twist = copy.deepcopy(message.root_state.twist.twist)
            self.joint_positions = joint_positions
        self.publish_state()

    def apply_joint_control(self, joint_updates):
        with self.lock:
            joint_positions = self.joint_positions.copy()
            for index, position in joint_updates:
                joint_positions[index] = position
            (
                self.root_position,
                self.root_orientation,
            ) = self.compensated_root_state(
                self.root_position,
                self.root_orientation,
                self.joint_positions,
                joint_positions,
            )
            self.joint_positions = joint_positions
        self.publish_state()

    def joint_control_callback(self, message):
        if not message.name:
            joint_positions = self.complete_joint_vector(message, "joints_ctrl")
            if joint_positions is None:
                return
            self.apply_joint_control(enumerate(joint_positions))
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
            joint_index = JOINT_INDEX.get(name)
            if joint_index is None:
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
            recognized.append((joint_index, float(position)))

        if not recognized:
            return

        self.apply_joint_control(recognized)

    def root_target_callback(self, _message):
        # Keep the planner-facing topic connected without bypassing the
        # whole-body command generated on full_state_target.
        return

    def compute_geometry(self, root_position, root_orientation, joint_positions):
        rotation = rotation_from_quaternion(root_orientation)
        position = np.asarray(root_position, dtype=float).copy()
        endpoints = [position.copy()]

        for link_index in range(LINK_COUNT):
            position += self.link_length * rotation[:, 0]
            endpoints.append(position.copy())
            if link_index < LINK_COUNT - 1:
                pitch = float(joint_positions[2 * link_index])
                yaw = float(joint_positions[2 * link_index + 1])
                rotation = rotation @ joint_rotation(pitch, yaw)

        endpoints = np.asarray(endpoints, dtype=float)
        link_midpoints = 0.5 * (endpoints[:-1] + endpoints[1:])
        return endpoints, np.mean(link_midpoints, axis=0)

    def baselink_pose(self, root_position, root_orientation, joint_positions):
        rotation = rotation_from_quaternion(root_orientation)
        position = np.asarray(root_position, dtype=float).copy()

        if self.baselink_chain is None:
            position += self.link_length * rotation[:, 0]
            rotation = rotation @ joint_rotation(
                float(joint_positions[0]), float(joint_positions[1])
            )
            return position, rotation

        for origin_position, origin_rotation, joint_index, axis in self.baselink_chain:
            position += rotation @ origin_position
            rotation = rotation @ origin_rotation
            if joint_index is not None:
                rotation = rotation @ tf_trans.rotation_matrix(
                    float(joint_positions[joint_index]), axis
                )[:3, :3]

        return position, rotation

    def compensated_root_state(
        self,
        root_position,
        root_orientation,
        previous_joint_positions,
        updated_joint_positions,
    ):
        # Hold the complete link2/FC pose while changing the articulated body.
        fixed_baselink_position, fixed_baselink_rotation = self.baselink_pose(
            root_position, root_orientation, previous_joint_positions
        )
        relative_baselink_position, relative_baselink_rotation = (
            self.baselink_pose(
                np.zeros(3),
                IDENTITY_QUATERNION,
                updated_joint_positions,
            )
        )

        updated_root_rotation = fixed_baselink_rotation @ relative_baselink_rotation.T
        updated_root_orientation = quaternion_from_rotation(updated_root_rotation)
        updated_root_position = (
            fixed_baselink_position - updated_root_rotation @ relative_baselink_position
        )
        return updated_root_position, updated_root_orientation

    def timer_callback(self, _event):
        self.publish_state()

    def publish_state(self):
        with self.lock:
            root_position = self.root_position.copy()
            root_orientation = self.root_orientation.copy()
            cog_orientation = self.cog_orientation.copy()
            root_twist = copy.deepcopy(self.root_twist)
            joint_positions = self.joint_positions.copy()

        endpoints, geometric_cog = self.compute_geometry(
            root_position, root_orientation, joint_positions
        )
        stamp = rospy.Time.now()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.world_frame_id
        odom.child_frame_id = self.cog_frame_id
        self.set_point(odom.pose.pose.position, geometric_cog)
        self.set_quaternion(odom.pose.pose.orientation, cog_orientation)
        odom.twist.twist = root_twist
        self.odom_pub.publish(odom)

        joint_state = JointState()
        joint_state.header.stamp = stamp
        joint_state.header.frame_id = self.root_frame_id
        joint_state.name = list(URDF_JOINT_NAMES)
        joint_state.position = joint_positions.tolist() + [0.0] * len(
            AUXILIARY_JOINT_NAMES
        )
        joint_state.velocity = [0.0] * len(URDF_JOINT_NAMES)
        joint_state.effort = [0.0] * len(URDF_JOINT_NAMES)
        self.joint_state_pub.publish(joint_state)

        self.root_pose_pub.publish(
            self.make_pose(stamp, root_position, root_orientation)
        )
        self.root_tail_pose_pub.publish(
            self.make_pose(stamp, endpoints[1], root_orientation)
        )

        self.marker_pub.publish(self.build_markers(stamp, endpoints))
        self.publish_transforms(
            stamp,
            root_position,
            root_orientation,
            geometric_cog,
            cog_orientation,
        )

    def build_markers(self, stamp, endpoints):
        marker_array = MarkerArray()
        for link_index in range(LINK_COUNT):
            start = endpoints[link_index]
            end = endpoints[link_index + 1]
            segment = end - start

            marker = self.create_marker(
                stamp, "dragon_links", link_index, Marker.CYLINDER
            )
            self.set_point(marker.pose.position, 0.5 * (start + end))
            self.set_quaternion(
                marker.pose.orientation, quaternion_rotating_z_to(segment)
            )
            marker.scale.x = marker.scale.y = self.link_diameter
            marker.scale.z = float(np.linalg.norm(segment))
            self.set_color(marker.color, LINK_COLORS[link_index != 0], 0.95)
            marker_array.markers.append(marker)

        for joint_index, endpoint in enumerate(endpoints):
            marker = self.create_marker(
                stamp, "dragon_joints", joint_index, Marker.SPHERE
            )
            self.set_point(marker.pose.position, endpoint)
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = self.joint_diameter
            self.set_color(marker.color, JOINT_COLORS[joint_index != 0])
            marker_array.markers.append(marker)

        return marker_array

    def publish_transforms(
        self,
        stamp,
        root_position,
        root_orientation,
        geometric_cog,
        cog_orientation,
    ):
        self.tf_broadcaster.sendTransform(
            [
                self.make_transform(
                    stamp, self.root_frame_id, root_position, root_orientation
                ),
                self.make_transform(
                    stamp, self.cog_frame_id, geometric_cog, cog_orientation
                ),
            ]
        )


if __name__ == "__main__":
    rospy.init_node("geo_dragon_model")
    DragonGeoRobotModel()
    rospy.spin()
