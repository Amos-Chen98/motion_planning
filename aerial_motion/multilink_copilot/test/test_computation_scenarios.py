#!/usr/bin/env python3
"""Scenario selection/reference checks; no simulation outputs are created."""
import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np
import rospkg
import xacro
from tf.transformations import euler_from_quaternion

package = Path(rospkg.RosPack().get_path('multilink_copilot'))
mono = Path(rospkg.RosPack().get_path('mono_planner'))
spec = importlib.util.spec_from_file_location('batch', package / 'scripts/evaluation/run_computation_benchmark_batch.py')
batch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(batch)
sys.path.insert(0, str(mono / 'scripts'))
from traj_planner.waypoint_reference import build_samples


class ScenariosTest(unittest.TestCase):
    def test_selected_dataset_and_cartesian_product(self):
        cases = batch.select_cases(mono / 'test_data/computation_benchmark')
        self.assertEqual(len(cases), 74)
        trials = batch.trial_list(cases, range(4, 9))
        self.assertEqual(len(trials), 370)
        self.assertEqual(len({t['name'] for t in trials}), 370)
        self.assertEqual(trials[0]['name'], 'case_0001/n4')
        with self.assertRaises(ValueError):
            batch.select_cases(mono / 'test_data/computation_benchmark', 3)

    def test_all_models_have_complete_indices(self):
        for n in range(4, 9):
            model = xacro.process_file(str(package / 'models/computation_benchmark.urdf.xacro'),
                                       mappings={'link_num': str(n)}).toxml()
            batch.validate_model(model, n)
            with self.assertRaises(ValueError):
                batch.validate_model(model, n + 1)

    def test_reference_includes_endpoint_and_full_attitude(self):
        case = mono / 'test_data/computation_benchmark/case_0001.yaml'
        poses = build_samples(case, 50, 40)
        self.assertEqual(len(poses), 2001)
        for pose in (poses[0], poses[-1]):
            p = pose.pose.position
            np.testing.assert_allclose([p.x, p.y, p.z], [.53, 0, 1], atol=1e-12)
        pitches = []
        for pose in poses:
            q = pose.pose.orientation
            vector = [q.x, q.y, q.z, q.w]
            self.assertAlmostEqual(np.linalg.norm(vector), 1)
            pitches.append(euler_from_quaternion(vector)[1])
        self.assertGreater(max(pitches), .8)
        self.assertLess(min(pitches), -.7)
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'reference.csv'
            batch.write_poses(path, poses[:81], .025)
            rows = np.loadtxt(path, delimiter=',', skiprows=1)
            self.assertEqual(rows.shape, (81, 8))
            self.assertEqual(rows[-1, 0], 2)
            self.assertAlmostEqual(rows[-1, 1], poses[80].pose.position.x)


if __name__ == '__main__':
    unittest.main()
