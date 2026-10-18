#!/usr/bin/env python3
"""Run selected waypoint scenarios with the complete Copilot on 4–8 links."""
import argparse
import csv
import importlib.util
import json
import os
import platform
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

import yaml


def select_cases(directory, case=None):
    available = sorted(Path(directory).glob('case_*.yaml'))
    if not available:
        raise ValueError(f'No cases in {directory}')
    if case is None:
        return available
    name = f'case_{case:04d}'
    selected = [p for p in available if p.stem == name]
    if not selected:
        raise ValueError(f'{name} is not in the selected no_skip_completed dataset')
    return selected


def trial_list(cases, links):
    return [dict(name=f'{case.stem}/n{n}', case=case.stem, links=n, status='pending')
            for case in cases for n in sorted(links)]


def write_poses(path, poses, dt):
    with Path(path).open('w', newline='') as stream:
        writer = csv.writer(stream, lineterminator='\n')
        writer.writerow(['time_s', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
        for i, message in enumerate(poses):
            p, q = message.pose.position, message.pose.orientation
            writer.writerow([format(v, '.17g') for v in (i * dt, p.x, p.y, p.z, q.x, q.y, q.z, q.w)])


def xml_structure(element):
    return element.tag, tuple(sorted(element.attrib.items())), tuple(xml_structure(c) for c in element)


def validate_model(xml, links):
    root = ET.fromstring(xml)
    joints = [j.attrib['name'] for j in root.findall('joint')]
    names = [item.attrib['name'] for item in root.findall('link')]
    expected = {f'joint{i}_{axis}' for i in range(1, links) for axis in ('pitch', 'yaw')}
    actual = {j for j in joints if j.startswith('joint') and j.endswith(('_pitch', '_yaw'))}
    edfs = {f'edf{i}_{side}' for i in range(1, links + 1) for side in ('left', 'right')}
    if actual != expected or {n for n in names if n.startswith('edf')} != edfs:
        raise ValueError(f'Invalid joint/EDF indexing for {links} links')
    if len(joints) != len(set(joints)) or len(names) != len(set(names)):
        raise ValueError('Duplicate model names')


def stop(process):
    if process is not None and process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def build_information(package):
    cache = package.parents[3] / 'build/multilink_copilot/CMakeCache.txt'
    keys = ('CMAKE_BUILD_TYPE:', 'CMAKE_CXX_COMPILER:', 'CMAKE_CXX_FLAGS:', 'CMAKE_CXX_FLAGS_RELWITHDEBINFO:')
    info = [line for line in cache.read_text().splitlines() if line.startswith(keys)] if cache.is_file() else []
    repo = package.parents[1]
    revision = subprocess.run(['git', '-C', str(repo), 'rev-parse', 'HEAD'], capture_output=True, text=True)
    status = subprocess.run(['git', '-C', str(repo), 'status', '--porcelain'], capture_output=True, text=True)
    flags = cache.parent / 'CMakeFiles/computation_benchmark.dir/flags.make'
    effective = [line for line in flags.read_text().splitlines() if line.startswith('CXX_FLAGS =')] if flags.is_file() else []
    return dict(cmake_cache=info, effective_compile_flags=effective, revision=revision.stdout.strip() or None,
                working_tree_dirty=bool(status.stdout.strip()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', type=int, help='Original case number; default: all 74 selected cases')
    parser.add_argument('--links', type=int, nargs='+', choices=range(4, 9), default=list(range(4, 9)))
    parser.add_argument('--visualize', action='store_true', help='One case and one link count; open RViz')
    parser.add_argument('--no-keep-open', action='store_true', help='Close RViz after the last sample')
    parser.add_argument('--playback-speed', type=float, default=1.0)
    parser.add_argument('--start-delay', type=float, default=3.0)
    parser.add_argument('--output-dir', type=Path, help='New, nonexisting result directory')
    parser.add_argument('--dry-run', action='store_true', help='List combinations without ROS processes or output files')
    parser.add_argument('--smoke', action='store_true', help='First case, all 5 link counts, first 2 s of the unchanged 50 s path')
    args = parser.parse_args()
    import math
    if not math.isfinite(args.playback_speed) or args.playback_speed <= 0 or not math.isfinite(args.start_delay) or args.start_delay < 0:
        parser.error('Invalid playback speed or start delay')
    if len(set(args.links)) != len(args.links):
        parser.error('Duplicate link counts')
    if args.smoke and (args.case is not None or args.links != list(range(4, 9))):
        parser.error('--smoke uses the first case and all five link counts')
    import rospkg
    packages = rospkg.RosPack()
    package = Path(packages.get_path('multilink_copilot'))
    mono = Path(packages.get_path('mono_planner'))
    dragon = Path(packages.get_path('dragon'))
    manager = Path(packages.get_path('data_manager'))
    try:
        cases = select_cases(mono / 'test_data/computation_benchmark', args.case)
    except ValueError as error:
        parser.error(str(error))
    if args.smoke:
        cases = cases[:1]
    if args.visualize and (len(cases) != 1 or len(args.links) != 1):
        parser.error('--visualize requires --case N and exactly one --links N')
    trials = trial_list(cases, args.links)
    if args.dry_run:
        for trial in trials:
            print(trial['name'])
        print(f'{len(cases)} cases, {len(trials)} unique trials; no simulation started')
        return 0
    analysis = manager / 'dragon_copilot/data_analysis/computation_benchmark/analyze_computation_benchmark.py'
    if not analysis.is_file():
        raise FileNotFoundError(analysis)
    config = yaml.safe_load((package / 'config/computation_benchmark.yaml').read_text())
    duration, rate = config['trajectory_time'], config['reference_rate']
    if duration <= 0 or rate <= 0 or not math.isclose(duration * rate, round(duration * rate)):
        raise ValueError('Invalid trajectory duration/rate')
    dt = 1.0 / rate
    expected = round((2.0 if args.smoke else duration) * rate) + 1
    for trial in trials:
        trial['expected_samples'] = expected
    import xacro
    models = {n: xacro.process_file(str(package / 'models/computation_benchmark.urdf.xacro'),
                                   mappings={'link_num': str(n)}).toxml() for n in range(4, 9)}
    for n, xml in models.items():
        validate_model(xml, n)
    original = xacro.process_file(str(dragon / 'robots/quad/v1_5_202601.urdf.xacro')).toxml()
    if xml_structure(ET.fromstring(original)) != xml_structure(ET.fromstring(models[4])):
        raise RuntimeError('Four-link model differs from original v1.5 model')
    root_parent = (Path(tempfile.gettempdir()) / 'copilot_benchmark_tests' if args.smoke else
                   manager / 'dragon_copilot/data/computation_benchmark')
    root = (args.output_dir or root_parent / (datetime.now(ZoneInfo('Asia/Tokyo')).strftime('%Y%m%d_%H%M%S') +
                                            ('_scenarios_smoke' if args.smoke else '_scenarios'))).resolve()
    root.mkdir(parents=True, exist_ok=False)
    print(f'Results: {root}', flush=True)
    algorithm = yaml.safe_load((package / 'config/copilot_planner.yaml').read_text())
    settings = dict(algorithm, **{k: config[k] for k in ('target_pose_frame_type', 'publish_stability_metrics', 'verbose')})
    settings.update(dt=dt, visualize=args.visualize,
                    keep_open=args.visualize and not args.no_keep_open,
                    playback_speed=args.playback_speed, start_delay=args.start_delay)
    robot = yaml.safe_load((dragon / 'config/quad/v1_5_202601/model/HydrusLikeRobotModel.yaml').read_text())
    (root / 'parameters.yaml').write_text(yaml.safe_dump(dict(algorithm=settings, robot=robot, scenario=config)))
    manifest = dict(schema_version=2, suite='scenarios', dt=dt, duration_seconds=duration,
                    smoke=args.smoke, visualize=args.visualize, model_four_equivalent=True, trials=trials,
                    platform=platform.platform(), cpu=subprocess.check_output(['lscpu'], text=True),
                    build=build_information(package))
    def save():
        temporary = root / 'manifest.json.tmp'
        temporary.write_text(json.dumps(manifest, indent=2) + '\n')
        temporary.replace(root / 'manifest.json')
    def analyze():
        # Automatic analysis uses this run; standalone analysis uses its top-level constants.
        spec = importlib.util.spec_from_file_location('computation_analysis', analysis)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module.analyze(root)
    save()
    sys.path.insert(0, str(mono / 'scripts'))
    from traj_planner.waypoint_reference import build_samples
    from geometry_msgs.msg import PoseStamped
    from tf.transformations import quaternion_from_euler
    import rosgraph
    import rospy
    with socket.socket() as socket_:
        socket_.bind(('127.0.0.1', 0))
        port = socket_.getsockname()[1]
    os.environ['ROS_MASTER_URI'] = f'http://127.0.0.1:{port}'
    os.environ['ROS_HOSTNAME'] = 'localhost'
    os.environ.pop('ROS_IP', None)
    master = child = None
    viewers = []
    active = None
    interrupted = False
    with tempfile.TemporaryDirectory(prefix='copilot_scenario_logs_') as temporary:
        os.environ['ROS_LOG_DIR'] = temporary
        try:
            with (Path(temporary) / 'roscore.log').open('w') as stream:
                master = subprocess.Popen(['roscore', '-p', str(port)], stdout=stream, stderr=subprocess.STDOUT,
                                          start_new_session=True)
            deadline = time.monotonic() + 20
            while not rosgraph.is_master_online():
                if master.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError('Private ROS master failed to start')
                time.sleep(0.1)
            rospy.init_node('scenario_benchmark_runner', anonymous=True, disable_signals=True)
            for case in cases:
                case_dir = root / case.stem
                case_dir.mkdir()
                shutil.copyfile(case, case_dir / 'case.yaml')
                try:
                    poses = build_samples(case_dir / 'case.yaml', duration, rate)
                    write_poses(case_dir / 'reference.csv', poses[:expected], dt)
                except Exception as error:
                    for trial in trials:
                        if trial['case'] == case.stem:
                            trial.update(status='failed', error=f'Reference generation: {error}')
                    save()
                    continue
                # Display waypoints are temporary derivatives of the saved case YAML.
                waypoints = []
                for w in yaml.safe_load((case_dir / 'case.yaml').read_text())['waypoints']:
                    pose = PoseStamped()
                    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = w[:3]
                    q = quaternion_from_euler(*w[3:])
                    pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
                    waypoints.append(pose)
                waypoint_file = Path(temporary) / 'waypoints.csv'
                write_poses(waypoint_file, waypoints, 1.0)
                for trial in [t for t in trials if t['case'] == case.stem]:
                    active = trial
                    directory = root / trial['name']
                    directory.mkdir()
                    trial['status'] = 'running'
                    save()
                    node = dict(settings, link_count=trial['links'], reference_file=str(case_dir / 'reference.csv'),
                                waypoint_file=str(waypoint_file), output_dir=str(directory))
                    rospy.set_param('/copilot_benchmark', dict(robot, robot_description=models[trial['links']],
                                                              computation_benchmark=node))
                    if args.visualize:
                        viewers.append(subprocess.Popen(['rosrun', 'robot_state_publisher', 'robot_state_publisher',
                                                         '__ns:=/copilot_benchmark', '_tf_prefix:=copilot_benchmark'],
                                                        start_new_session=True, stdout=subprocess.DEVNULL))
                        viewers.append(subprocess.Popen(['rosrun', 'rviz', 'rviz', '-d',
                                                         str(package / 'rviz/computation_benchmark.rviz')],
                                                        start_new_session=True, stdout=subprocess.DEVNULL))
                    print(f'Running {trial["name"]}: {expected} samples', flush=True)
                    with tempfile.TemporaryFile(mode='w+') as log:
                        child = subprocess.Popen(['rosrun', 'multilink_copilot', 'computation_benchmark',
                                                  '__ns:=/copilot_benchmark'], start_new_session=True,
                                                 stdout=None if args.visualize else log,
                                                 stderr=None if args.visualize else subprocess.STDOUT)
                        try:
                            # A finished visual run deliberately waits for Ctrl+C.
                            code = child.wait(timeout=None if settings['keep_open'] else config['trial_timeout'])
                            trial.update(returncode=code, status='completed' if code == 0 else 'failed')
                        except subprocess.TimeoutExpired:
                            stop(child)
                            trial.update(returncode=child.returncode, status='timeout', error='Trial wall timeout')
                        log.seek(0)
                        error = log.read()[-4000:].strip()
                        if trial['status'] != 'completed' and error:
                            trial['error'] = error
                    save()
                    print(f'{trial["name"]}: {trial["status"]}', flush=True)
                    active = None
                    for viewer in viewers:
                        stop(viewer)
                    viewers.clear()
        except KeyboardInterrupt:
            interrupted = True
            stop(child)  # Let the C++ process flush completed samples before analysis.
            if active is not None:
                active.update(returncode=child.returncode if child else None,
                              status='completed' if child and child.returncode == 0 else 'interrupted')
        except Exception as error:
            manifest['error'] = str(error)
            if active is not None:
                active.update(status='failed', error=str(error))
        finally:
            stop(child)
            for viewer in viewers:
                stop(viewer)
            stop(master)
            save()
    report = analyze()
    print(f'Saved per-trial median/P90/P95: {root / "summary.csv"}', flush=True)
    for issue in report['issues']:
        print(f'Note: {issue}', flush=True)
    if manifest.get('error'):
        print(f'Runner error: {manifest["error"]}', file=sys.stderr)
    complete = all(t['complete'] for t in report['trials'].values())
    if interrupted:
        return 0 if complete else 130
    return 1 if manifest.get('error') or not complete else 0


if __name__ == '__main__':
    sys.exit(main())
