#!/usr/bin/env python3
"""Validate rendered state/TF and equality of visual and headless decisions."""
import csv
import json
import os
import signal
import sys
import subprocess
import time
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import rospkg
import rospy
import yaml
import rostest
import tf
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from visualization_msgs.msg import MarkerArray


class BenchmarkVisualizationTest(unittest.TestCase):
    def setUp(self):
        root = Path(rospy.get_param('~output_dir'))
        root.mkdir(parents=True, exist_ok=False)
        package = Path(rospkg.RosPack().get_path('multilink_copilot'))
        mono = Path(rospkg.RosPack().get_path('mono_planner'))
        sys.path.insert(0, str(mono / 'scripts'))
        sys.path.insert(0, str(package / 'scripts/evaluation'))
        from traj_planner.waypoint_reference import build_samples
        from run_computation_benchmark_batch import write_poses
        case = mono / 'test_data/computation_benchmark/case_0001.yaml'
        config = yaml.safe_load((package / 'config/computation_benchmark.yaml').read_text())
        dt = 1.0 / config['reference_rate']
        reference = build_samples(case, config['trajectory_time'], config['reference_rate'])[:81]
        write_poses(root / 'reference.csv', reference, dt)
        waypoints = []
        for waypoint in yaml.safe_load(case.read_text())['waypoints']:
            pose = PoseStamped()
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = waypoint[:3]
            q = tf.transformations.quaternion_from_euler(*waypoint[3:])
            pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
            waypoints.append(pose)
        write_poses(root / 'waypoints.csv', waypoints, 1.0)
        params = yaml.safe_load((package / 'config/copilot_planner.yaml').read_text())
        params.update(config)
        params.update(dt=dt, visualize=True, keep_open=True, playback_speed=20.0, start_delay=0.5,
                      link_count=8, reference_file=str(root / 'reference.csv'),
                      waypoint_file=str(root / 'waypoints.csv'), output_dir=str(root))
        rospy.set_param('/copilot_benchmark/computation_benchmark', params)
        self.process = subprocess.Popen(['rosrun', 'multilink_copilot', 'computation_benchmark',
                                         '__ns:=/copilot_benchmark'],
                                        start_new_session=True, stdout=subprocess.DEVNULL)

    def tearDown(self):
        if self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait()

    def test_visual_state_and_headless_parity(self):
        root = Path(rospy.get_param('~output_dir'))
        listener = tf.TransformListener()
        deadline = time.monotonic() + 50
        finished = None
        while time.monotonic() < deadline:
            message = rospy.wait_for_message('/copilot_benchmark/visualization', MarkerArray, timeout=10)
            if any('Finished' in marker.text for marker in message.markers):
                finished = message
                break
        self.assertIsNotNone(finished)
        marker_map = {marker.ns: marker for marker in finished.markers}
        self.assertEqual(len(marker_map['reference_path'].points), 81)
        self.assertEqual(len(marker_map['commanded_body'].points), 9)
        self.assertEqual(sum(m.ns == 'waypoint_ring' for m in finished.markers), 4)
        states = rospy.wait_for_message('/copilot_benchmark/joint_states', JointState, timeout=5)
        model = ET.fromstring(rospy.get_param('/copilot_benchmark/robot_description'))
        movable = {j.attrib['name'] for j in model.findall('joint') if j.attrib['type'] != 'fixed'}
        self.assertEqual(set(states.name), movable)
        self.assertEqual(len(states.name), len(states.position))
        listener.waitForTransform('world', 'copilot_benchmark/edf8_right', rospy.Time(0), rospy.Duration(5))
        position, rotation = listener.lookupTransform('world', 'copilot_benchmark/edf8_right', rospy.Time(0))
        self.assertTrue(all(abs(value) < 100 for value in position + rotation))
        with (root / 'samples.csv').open() as stream:
            visual = list(csv.DictReader(stream))
        self.assertEqual(len(visual), 81)
        self.assertTrue(json.loads((root / 'run_settings.json').read_text())['visualize'])
        parameters = rospy.get_param('/copilot_benchmark/computation_benchmark')
        headless = root.with_name(root.name + '_headless')
        parameters.update(visualize=False, keep_open=False, output_dir=str(headless))
        rospy.set_param('/copilot_benchmark/headless', parameters)
        subprocess.run(['rosrun', 'multilink_copilot', 'computation_benchmark',
                        '__ns:=/copilot_benchmark', '__name:=headless'],
                       check=True, timeout=40, stdout=subprocess.DEVNULL)
        with (headless / 'samples.csv').open() as stream:
            plain = list(csv.DictReader(stream))
        self.assertEqual(len(plain), len(visual))
        fields = ('step', 'status', 'source', 'fallback', 'projection_calls', 'qp_solves',
                  'projection_iterations', 'published', 'output_safe', 'geometry_warmup')
        for a, b in zip(visual, plain):
            self.assertEqual({k: a[k] for k in fields}, {k: b[k] for k in fields})
        self.assertTrue(any(int(row['qp_solves']) > 0 for row in visual))


if __name__ == '__main__':
    rospy.init_node('benchmark_visualization_test')
    rostest.rosrun('multilink_copilot', 'benchmark_visualization_test', BenchmarkVisualizationTest)
