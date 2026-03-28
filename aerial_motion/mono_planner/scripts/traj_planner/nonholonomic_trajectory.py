import math

import numpy as np
import tf.transformations as tf_trans

from traj_planner.waypoint_planner_common import canonicalize_quaternion


def normalize_vector(vector, epsilon, description):
    vector = np.asarray(vector, dtype=float)
    norm = np.linalg.norm(vector)
    if norm < epsilon:
        raise ValueError("Cannot normalize near-zero {}.".format(description))
    return vector / norm


def quaternion_to_rotation_matrix(quaternion):
    matrix = tf_trans.quaternion_matrix(canonicalize_quaternion(quaternion))
    return matrix[:3, :3]


def rotation_about_x(angle):
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, cosine, -sine],
            [0.0, sine, cosine],
        ],
        dtype=float,
    )


def matrix_to_quaternion(rotation_matrix):
    homogeneous = np.eye(4)
    homogeneous[:3, :3] = rotation_matrix
    return tf_trans.quaternion_from_matrix(homogeneous)


def rotation_matrix_from_tangent(tangent, tangent_epsilon, axis_epsilon):
    x_body = normalize_vector(tangent, tangent_epsilon, "trajectory tangent")
    reference_axes = (
        np.array([0.0, 0.0, 1.0], dtype=float),
        np.array([0.0, 1.0, 0.0], dtype=float),
        np.array([1.0, 0.0, 0.0], dtype=float),
    )

    y_body = None
    for reference_axis in reference_axes:
        candidate = np.cross(reference_axis, x_body)
        if np.linalg.norm(candidate) >= axis_epsilon:
            y_body = normalize_vector(candidate, tangent_epsilon, "body y-axis")
            break

    if y_body is None:
        raise ValueError("Failed to construct a body frame from the trajectory tangent.")

    z_body = normalize_vector(np.cross(x_body, y_body), tangent_epsilon, "body z-axis")
    y_body = normalize_vector(np.cross(z_body, x_body), tangent_epsilon, "body y-axis re-orthogonalization")

    return np.column_stack((x_body, y_body, z_body))


def extract_twist_angle(base_rotation, input_rotation, tolerance):
    relative_rotation = np.dot(base_rotation.T, input_rotation)
    twist_angle = math.atan2(relative_rotation[2, 1], relative_rotation[1, 1])
    expected_rotation = rotation_about_x(twist_angle)
    if not np.allclose(relative_rotation, expected_rotation, atol=tolerance, rtol=0.0):
        raise ValueError("Failed to extract a pure body-x twist angle from the waypoint orientation.")
    return twist_angle


class QuinticPolynomialSegment:
    def __init__(
        self,
        start_position,
        start_velocity,
        start_acceleration,
        end_position,
        end_velocity,
        end_acceleration,
        duration,
    ):
        self.duration = float(duration)
        if self.duration <= 0.0:
            raise ValueError("Segment duration must be positive.")

        start_position = np.atleast_1d(np.asarray(start_position, dtype=float))
        start_velocity = np.atleast_1d(np.asarray(start_velocity, dtype=float))
        start_acceleration = np.atleast_1d(np.asarray(start_acceleration, dtype=float))
        end_position = np.atleast_1d(np.asarray(end_position, dtype=float))
        end_velocity = np.atleast_1d(np.asarray(end_velocity, dtype=float))
        end_acceleration = np.atleast_1d(np.asarray(end_acceleration, dtype=float))

        boundary_matrix = self.build_boundary_matrix(self.duration)
        boundary_values = np.vstack(
            [
                start_position,
                start_velocity,
                start_acceleration,
                end_position,
                end_velocity,
                end_acceleration,
            ]
        )
        self.coefficients = np.linalg.solve(boundary_matrix, boundary_values)

    @staticmethod
    def build_boundary_matrix(duration):
        return np.array(
            [
                [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 2.0, 0.0, 0.0, 0.0],
                [1.0, duration, duration**2, duration**3, duration**4, duration**5],
                [0.0, 1.0, 2.0 * duration, 3.0 * duration**2, 4.0 * duration**3, 5.0 * duration**4],
                [0.0, 0.0, 2.0, 6.0 * duration, 12.0 * duration**2, 20.0 * duration**3],
            ],
            dtype=float,
        )

    def sample(self, time_from_segment_start):
        time_from_segment_start = min(max(float(time_from_segment_start), 0.0), self.duration)
        basis = np.array(
            [
                1.0,
                time_from_segment_start,
                time_from_segment_start**2,
                time_from_segment_start**3,
                time_from_segment_start**4,
                time_from_segment_start**5,
            ],
            dtype=float,
        )
        return np.dot(basis, self.coefficients)

    def sample_velocity(self, time_from_segment_start):
        time_from_segment_start = min(max(float(time_from_segment_start), 0.0), self.duration)
        basis = np.array(
            [
                0.0,
                1.0,
                2.0 * time_from_segment_start,
                3.0 * time_from_segment_start**2,
                4.0 * time_from_segment_start**3,
                5.0 * time_from_segment_start**4,
            ],
            dtype=float,
        )
        return np.dot(basis, self.coefficients)


class NonholonomicQuinticTrajectory:
    def __init__(
        self,
        closed_knot_positions,
        closed_knot_rotations,
        total_time,
        tangent_epsilon,
        axis_epsilon,
        feasibility_epsilon,
        rotation_tolerance,
    ):
        self.closed_knot_positions = np.asarray(closed_knot_positions, dtype=float)
        self.closed_knot_rotations = np.asarray(closed_knot_rotations, dtype=float)
        self.total_time = float(total_time)
        self.tangent_epsilon = float(tangent_epsilon)
        self.axis_epsilon = float(axis_epsilon)
        self.feasibility_epsilon = float(feasibility_epsilon)
        self.rotation_tolerance = float(rotation_tolerance)

        if self.closed_knot_positions.ndim != 2 or self.closed_knot_positions.shape[1] != 3:
            raise ValueError("Closed knot positions must have shape (N, 3).")
        if self.closed_knot_rotations.ndim != 3 or self.closed_knot_rotations.shape[1:] != (3, 3):
            raise ValueError("Closed knot rotations must have shape (N, 3, 3).")
        if self.closed_knot_positions.shape[0] != self.closed_knot_rotations.shape[0]:
            raise ValueError("Closed knot positions and rotations must have the same knot count.")
        if self.closed_knot_positions.shape[0] < 3:
            raise ValueError("At least one waypoint is required to build a closed trajectory.")
        if self.total_time <= 0.0:
            raise ValueError("Total trajectory time must be positive.")

        self.segment_count = self.closed_knot_positions.shape[0] - 1
        self.segment_displacements = np.diff(self.closed_knot_positions, axis=0)
        self.segment_distances = np.linalg.norm(self.segment_displacements, axis=1)
        if np.any(self.segment_distances < self.tangent_epsilon):
            raise ValueError("Consecutive knot positions must not be coincident.")

        self.segment_times = self.total_time * self.segment_distances / np.sum(self.segment_distances)
        self.cumulative_times = np.concatenate(([0.0], np.cumsum(self.segment_times)))
        self.closed_knot_forward_axes = self.closed_knot_rotations[:, :, 0]

        self.validate_segment_feasibility()
        self.closed_knot_velocities = self.build_closed_knot_velocities()
        self.closed_knot_base_rotations = np.stack(
            [
                rotation_matrix_from_tangent(forward_axis, self.tangent_epsilon, self.axis_epsilon)
                for forward_axis in self.closed_knot_forward_axes
            ],
            axis=0,
        )
        self.closed_knot_twist_angles = self.build_closed_knot_twist_angles()
        self.position_segments = self.build_position_segments()
        self.twist_segments = self.build_twist_segments()

    def validate_segment_feasibility(self):
        for segment_index in range(self.segment_count):
            chord = normalize_vector(
                self.segment_displacements[segment_index],
                self.tangent_epsilon,
                "segment chord",
            )
            start_alignment = np.dot(chord, self.closed_knot_forward_axes[segment_index])
            end_alignment = np.dot(chord, self.closed_knot_forward_axes[segment_index + 1])
            if start_alignment <= self.feasibility_epsilon or end_alignment <= self.feasibility_epsilon:
                raise ValueError(
                    "Segment {} is incompatible with the waypoint/root forward-axis constraint.".format(segment_index)
                )

    def build_closed_knot_velocities(self):
        segment_speeds = self.segment_distances / self.segment_times
        unique_knot_speeds = np.zeros(self.segment_count, dtype=float)
        for knot_index in range(self.segment_count):
            previous_segment_index = (knot_index - 1) % self.segment_count
            next_segment_index = knot_index
            unique_knot_speeds[knot_index] = 0.5 * (
                segment_speeds[previous_segment_index] + segment_speeds[next_segment_index]
            )

        closed_knot_speeds = np.concatenate((unique_knot_speeds, unique_knot_speeds[:1]))
        return closed_knot_speeds[:, np.newaxis] * self.closed_knot_forward_axes

    def build_closed_knot_twist_angles(self):
        unique_twist_angles = [
            extract_twist_angle(
                self.closed_knot_base_rotations[knot_index],
                self.closed_knot_rotations[knot_index],
                self.rotation_tolerance,
            )
            for knot_index in range(self.segment_count)
        ]
        return np.unwrap(np.asarray(unique_twist_angles + unique_twist_angles[:1], dtype=float))

    def build_position_segments(self):
        zero_acceleration = np.zeros(3, dtype=float)
        return [
            QuinticPolynomialSegment(
                self.closed_knot_positions[segment_index],
                self.closed_knot_velocities[segment_index],
                zero_acceleration,
                self.closed_knot_positions[segment_index + 1],
                self.closed_knot_velocities[segment_index + 1],
                zero_acceleration,
                self.segment_times[segment_index],
            )
            for segment_index in range(self.segment_count)
        ]

    def build_twist_segments(self):
        zero_scalar = np.array([0.0], dtype=float)
        return [
            QuinticPolynomialSegment(
                np.array([self.closed_knot_twist_angles[segment_index]], dtype=float),
                zero_scalar,
                zero_scalar,
                np.array([self.closed_knot_twist_angles[segment_index + 1]], dtype=float),
                zero_scalar,
                zero_scalar,
                self.segment_times[segment_index],
            )
            for segment_index in range(self.segment_count)
        ]

    def locate_segment(self, sample_time):
        if sample_time >= self.total_time:
            return self.segment_count - 1, self.segment_times[-1]

        segment_index = np.searchsorted(self.cumulative_times, sample_time, side="right") - 1
        segment_index = min(max(segment_index, 0), self.segment_count - 1)
        local_time = sample_time - self.cumulative_times[segment_index]
        return segment_index, local_time

    def sample(self, sample_time):
        segment_index, local_time = self.locate_segment(sample_time)
        position = self.position_segments[segment_index].sample(local_time)
        velocity = self.position_segments[segment_index].sample_velocity(local_time)
        twist_angle = self.twist_segments[segment_index].sample(local_time)[0]
        return position, velocity, twist_angle
