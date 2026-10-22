#!/usr/bin/env python3
"""Numerical and boundary checks for the demo's ROS-independent entry model."""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts", "demo"))
from dragon_dance_trajectory import AdaptiveEntryTrajectory, yaw_rate_from_body_rates


class AdaptiveEntryTrajectoryTest(unittest.TestCase):
    def trajectory(self, **overrides):
        arguments = dict(
            start_position=(0.0, 0.0, 0.0), start_velocity=(0.0, 0.0, 0.0),
            start_yaw=0.0, start_yaw_rate=0.0, goal_position=(1.0, 0.0, 0.0),
            goal_yaw=0.0, max_linear_speed=0.3, max_yaw_rate=0.3, publish_rate=40.0,
        )
        arguments.update(overrides)
        return AdaptiveEntryTrajectory(**arguments)

    def assert_bounds(self, trajectory, linear_limit=0.3, yaw_limit=0.3):
        for index in range(2001):
            _, velocity, _, yaw_rate = trajectory.sample(index * trajectory.duration / 2000)
            self.assertLessEqual(math.hypot(*velocity), linear_limit + 1e-12)
            self.assertLessEqual(abs(yaw_rate), yaw_limit + 1e-12)

    def test_linear_duration_and_exact_peak(self):
        trajectory = self.trajectory()
        self.assertAlmostEqual(trajectory.duration, 6.25)
        self.assertEqual(trajectory.limiting_factor, "linear_speed")
        self.assertAlmostEqual(math.hypot(*trajectory.sample(trajectory.duration / 2)[1]), 0.3)
        self.assert_bounds(trajectory)

    def test_rotation_only_and_shortest_arc(self):
        trajectory = self.trajectory(
            goal_position=(0.0, 0.0, 0.0),
            start_yaw=math.radians(179.0), goal_yaw=math.radians(-179.0),
        )
        self.assertAlmostEqual(trajectory.yaw_delta, math.radians(2.0))
        self.assertEqual(trajectory.limiting_factor, "yaw_rate")
        self.assertAlmostEqual(trajectory.sample(trajectory.duration)[2], math.radians(181.0))
        self.assert_bounds(trajectory)

    def test_endpoint_position_velocity_and_acceleration(self):
        trajectory = self.trajectory(
            start_position=(1.0, -3.0, 2.0), start_velocity=(0.02, -0.01, 0.03),
            start_yaw=1.0, start_yaw_rate=-0.02,
            goal_position=(-1.0, 2.0, 5.0), goal_yaw=-1.0,
        )
        expected_start = ((1.0, -3.0, 2.0), (0.02, -0.01, 0.03), 1.0, -0.02)
        self.assertEqual(trajectory.sample(0.0), expected_start)
        self.assertEqual(trajectory.sample(-1.0), expected_start)
        self.assertEqual(trajectory.sample(trajectory.duration),
                         ((-1.0, 2.0, 5.0), (0.0, 0.0, 0.0), -1.0, 0.0))
        self.assertEqual(trajectory.sample(trajectory.duration + 1.0),
                         trajectory.sample(trajectory.duration))
        step = 1e-5
        for endpoint, offset in ((0.0, step), (trajectory.duration, -step)):
            _, velocity, _, yaw_rate = trajectory.sample(endpoint)
            _, nearby_velocity, _, nearby_yaw_rate = trajectory.sample(endpoint + offset)
            for component, nearby_component in zip(velocity, nearby_velocity):
                self.assertAlmostEqual((nearby_component - component) / offset, 0.0, places=5)
            self.assertAlmostEqual((nearby_yaw_rate - yaw_rate) / offset, 0.0, places=5)
        self.assert_bounds(trajectory)

    def test_combined_motion_and_arbitrary_initial_velocities(self):
        for velocity in ((0.02, 0.0, 0.0), (-0.02, 0.0, 0.0),
                         (0.0, 0.02, 0.0), (0.0, 0.0, -0.02),
                         (-0.1, 0.1, -0.2)):
            for yaw_rate in (-0.2, 0.0, 0.2):
                with self.subTest(velocity=velocity, yaw_rate=yaw_rate):
                    self.assert_bounds(self.trajectory(start_velocity=velocity,
                                                       start_yaw_rate=yaw_rate, goal_yaw=0.8))

    def test_velocity_matches_reference_derivative(self):
        trajectory = self.trajectory(
            start_velocity=(0.02, -0.01, 0.03), start_yaw_rate=-0.02,
            goal_position=(1.0, -2.0, 0.5), goal_yaw=0.8,
        )
        step = 1e-4
        for fraction in (0.1, 0.3, 0.5, 0.8):
            elapsed = trajectory.duration * fraction
            _, velocity, _, yaw_rate = trajectory.sample(elapsed)
            before = trajectory.sample(elapsed - step)
            after = trajectory.sample(elapsed + step)
            for axis in range(3):
                derivative = (after[0][axis] - before[0][axis]) / (2.0 * step)
                self.assertAlmostEqual(derivative, velocity[axis], places=8)
            self.assertAlmostEqual((after[2] - before[2]) / (2.0 * step), yaw_rate, places=8)

    def test_distance_and_orientation_select_duration(self):
        close = self.trajectory(goal_position=(0.1, 0.0, 0.0))
        far = self.trajectory(goal_position=(10.0, 0.0, 0.0))
        self.assertGreater(far.duration, 30.0)
        self.assertAlmostEqual(far.duration, 100.0 * close.duration)
        rotating = self.trajectory(goal_position=(0.1, 0.0, 0.0), goal_yaw=2.0)
        self.assertEqual(rotating.limiting_factor, "yaw_rate")
        self.assertGreater(rotating.duration, close.duration)
        self.assert_bounds(far)
        self.assert_bounds(rotating)

    def test_stationary_and_near_target(self):
        trajectory = self.trajectory(goal_position=(0.0, 0.0, 0.0))
        self.assertEqual(trajectory.limiting_factor, "publish_rate")
        self.assertAlmostEqual(trajectory.duration, 0.05)
        self.assertEqual(trajectory.sample(0.025), ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0, 0.0))
        self.assert_bounds(self.trajectory(goal_position=(1e-6, 0.0, 0.0),
                                          start_velocity=(0.0, 0.02, 0.0)))

    def test_invalid_inputs(self):
        invalid = (
            {"start_position": (1.0, 2.0)}, {"goal_position": (1.0, 2.0, 3.0, 4.0)},
            {"start_velocity": None}, {"goal_position": (float("nan"), 0.0, 0.0)},
            {"start_yaw": float("inf")}, {"start_yaw_rate": float("nan")},
            {"max_linear_speed": 0.0}, {"max_yaw_rate": -1.0},
            {"publish_rate": 0.0}, {"publish_rate": float("inf")},
            {"publish_rate": True}, {"start_velocity": (0.3, 0.0, 0.0)},
            {"start_velocity": (0.2, 0.2, 0.2)}, {"start_yaw_rate": 0.3},
            {"start_yaw_rate": -0.31}, {"goal_yaw": "invalid"},
            {"goal_position": (1e308, 1e308, 1e308)},
        )
        for overrides in invalid:
            with self.subTest(overrides=overrides), self.assertRaises(ValueError):
                self.trajectory(**overrides)
        with self.assertRaises(ValueError):
            self.trajectory().sample(float("nan"))


class BodyYawRateTest(unittest.TestCase):
    def test_level_body(self):
        self.assertAlmostEqual(yaw_rate_from_body_rates(0.0, 0.0, (0.1, 0.2, 0.3)), 0.3)

    def test_tilted_body_reconstructs_euler_yaw_rate(self):
        roll, pitch = 0.4, -0.6
        roll_rate, pitch_rate, yaw_rate = 0.1, -0.2, 0.3
        body_rates = (
            roll_rate - yaw_rate * math.sin(pitch),
            pitch_rate * math.cos(roll) + yaw_rate * math.sin(roll) * math.cos(pitch),
            -pitch_rate * math.sin(roll) + yaw_rate * math.cos(roll) * math.cos(pitch),
        )
        self.assertAlmostEqual(yaw_rate_from_body_rates(roll, pitch, body_rates), yaw_rate)

    def test_singular_and_invalid_inputs(self):
        for pitch in (math.pi / 2, -math.pi / 2, math.pi / 2 - 1e-8):
            with self.assertRaisesRegex(ValueError, "singularity"):
                yaw_rate_from_body_rates(0.0, pitch, (0.0, 0.0, 0.0))
        with self.assertRaises(ValueError):
            yaw_rate_from_body_rates(0.0, 0.0, (float("nan"), 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
