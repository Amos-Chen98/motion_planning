#!/usr/bin/env python3
"""Verify case geometry and time coloring without starting a simulator."""
import io
from pathlib import Path
import sys
import unittest

import numpy as np
import rospkg
import yaml
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts/evaluation"))
from robustness_reference_publisher import build_samples, build_visualization


class VisualizationTest(unittest.TestCase):
    def test_all_waypoints_and_exact_reference_samples(self):
        base = Path(rospkg.RosPack().get_path("mono_planner")) / "test_data/batch_robustness_sim"
        for case in ("case_0001.yaml", "case_0002.yaml"):
            with self.subTest(case=case):
                path = base / case
                data = yaml.safe_load(path.read_text())
                samples = build_samples(path, 50, 40)
                poses, rings, trajectory = build_visualization(path, samples, 50, 40)
                self.assertEqual(len(poses), 4)
                self.assertEqual(rings.markers[0].action, Marker.DELETEALL)
                self.assertEqual(trajectory.markers[0].action, Marker.DELETEALL)
                self.assertEqual([m.id for m in rings.markers[1:]], list(range(4)))
                for waypoint, pose, ring in zip(data["waypoints"], poses, rings.markers[1:]):
                    p, q = pose.pose.position, pose.pose.orientation
                    np.testing.assert_allclose([p.x, p.y, p.z], waypoint[:3])
                    np.testing.assert_allclose([q.x, q.y, q.z, q.w], quaternion_from_euler(*waypoint[3:]))
                    self.assertEqual(ring.pose, pose.pose)
                    self.assertEqual(ring.header.frame_id, "world")
                    self.assertEqual(ring.lifetime.to_sec(), 0)
                    self.assertEqual(len(ring.points), 73)
                line = trajectory.markers[-1]
                self.assertEqual(line.type, Marker.LINE_STRIP)
                self.assertEqual(len(line.points), 2001)
                self.assertEqual(line.points, [m.pose.position for m in samples])
                self.assertEqual(len(line.colors), len(line.points))
                from traj_planner.waypoint_planner_common import interpolate_time_color
                for i in (0, 320, 1000, 2000):
                    self.assertEqual(line.colors[i], interpolate_time_color(i/2000))
                self.assertNotEqual(line.colors[0], line.colors[-1])
                rings.serialize(io.BytesIO())
                trajectory.serialize(io.BytesIO())


if __name__ == "__main__":
    unittest.main()
