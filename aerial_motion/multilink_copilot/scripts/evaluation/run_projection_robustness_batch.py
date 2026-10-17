#!/usr/bin/env python3
"""Isolated full-dynamics trials; --attach runs one trial in a manually started simulator."""
import argparse
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
import uuid
from collections import deque
from contextlib import contextmanager
from datetime import datetime, timezone

import rospkg
import yaml

# Edit these paths here; launching a batch requires no path arguments or exports.
ROS_PACK = rospkg.RosPack()
CASE_DIR = Path(ROS_PACK.get_path("mono_planner")) / "test_data/batch_robustness_sim"
CONFIG_FILE = Path(ROS_PACK.get_path("multilink_copilot")) / "config/projection_robustness.yaml"
DATA_DIR = Path(ROS_PACK.get_path("data_manager")) / "dragon_copilot/data/rosbag/projection_robustness"
OUTPUT_DIR = DATA_DIR / "batch_100"
SELECTED_OUTPUT_DIR = DATA_DIR / "batch_100_single"
MANUAL_OUTPUT_DIR = DATA_DIR / "batch_100_manual"


def package_dir():
    return Path(ROS_PACK.get_path("multilink_copilot"))


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save_json(path, value):
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
    tmp.replace(path)


def save_error_excerpt(output, log_dir, result):
    """Only infrastructure failures need a bounded text log; trial.json records flight failures."""
    if result.get("status") != "failed" or result.get("reason", "").startswith("left_hover_state_"):
        return
    lines = deque(maxlen=99)
    for path in sorted(log_dir.glob("*.log")):
        with path.open(errors="replace") as stream:
            tail = deque(stream, maxlen=15)
        lines.append("--- {} ---\n".format(path.name))
        lines.extend(line if line.endswith("\n") else line+"\n" for line in tail)
    (output / "error.log").write_text(result.get("reason", "unknown failure").replace("\n", " ")+"\n"+"".join(lines))


@contextmanager
def mirror_log(path, enabled):
    """Follow a child log from the foreground runner, retaining the original file."""
    if not enabled:
        yield
        return
    stream = None
    done = threading.Event()

    def forward():
        nonlocal stream
        if stream is None:
            try:
                stream = path.open(errors="replace")
            except FileNotFoundError:
                return  # The planner starts only after initialization has settled.
        while True:
            chunk = stream.read(65536)
            if not chunk:
                break
            sys.stdout.write(chunk)
            sys.stdout.flush()

    def follow():
        while not done.wait(0.05):
            forward()

    follower = threading.Thread(target=follow, name="copilot-log", daemon=True)
    follower.start()
    try:
        yield
    finally:
        done.set()
        follower.join()
        try:
            forward()  # Include the final output after worker/process cleanup.
        finally:
            if stream is not None:
                stream.close()


def free_port():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def stop(process):
    if process.poll() is not None:
        return
    for sig, timeout in ((signal.SIGINT, 15), (signal.SIGTERM, 5), (signal.SIGKILL, 3)):
        try:
            os.killpg(process.pid, sig)
            process.wait(timeout=timeout)
            return
        except ProcessLookupError:
            return
        except subprocess.TimeoutExpired:
            continue


def interrupted(_signum, _frame):
    raise KeyboardInterrupt


def cleanup_trial_processes(token):
    """Recover only this trial's descendants after a worker/roslaunch crash."""
    marker = ("DRAGON_ROBUSTNESS_TRIAL_TOKEN=" + token).encode()
    def remaining():
        pids = []
        for path in Path("/proc").glob("[0-9]*/environ"):
            try:
                if marker in path.read_bytes().split(b"\0"):
                    pids.append(int(path.parent.name))
            except (OSError, ProcessLookupError):
                pass
        return pids
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        pids = remaining()
        if not pids: return
        for pid in pids:
            try: os.kill(pid, sig)
            except ProcessLookupError: pass
        deadline = time.monotonic()+3
        while remaining() and time.monotonic() < deadline:
            time.sleep(0.1)


def quaternion_error(a, b):
    dot = sum(x*y for x, y in zip(a, b))
    norm = math.sqrt(sum(x*x for x in a) * sum(x*x for x in b))
    return 2 * math.acos(min(1.0, abs(dot / norm))) if norm > 1e-12 else math.inf


class Trial:
    def __init__(self, args, config):
        self.args, self.config = args, config
        self.output = Path(args.output)
        self.temp_logs = None if os.environ.get("DRAGON_ROBUSTNESS_LOG_DIR") else tempfile.TemporaryDirectory(prefix="dragon_robustness_")
        self.log_dir = Path(os.environ.get("DRAGON_ROBUSTNESS_LOG_DIR") or self.temp_logs.name)
        os.environ["ROS_LOG_DIR"] = str(self.log_dir / "ros")
        self.processes, self.logs = [], []
        self.latest, self.received = {}, {}
        self.events = []
        self.reference_count = 0
        self.meta = dict(case=Path(args.case_file).stem, case_sha256=digest(args.case_file),
                         status="starting", config=config, started_utc=datetime.now(timezone.utc).isoformat())
        self.start_wall = time.monotonic()
        self.last_clock = None
        self.last_clock_wall = time.monotonic()

    def launch(self, name, command):
        log = (self.log_dir / (name + ".log")).open("w")
        self.logs.append(log)
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   stdin=subprocess.DEVNULL, start_new_session=True)
        self.processes.append((name, process))
        return process

    def receive(self, key, msg):
        self.latest[key] = msg
        self.received[key] = time.monotonic()
        if key == "reference":
            self.reference_count += 1
        elif key == "event":
            self.events.append(json.loads(msg.data))
        elif key == "flight" and msg.data != 5 and any(e["phase"] == "started" for e in self.events) \
                and "instability_stamp" not in self.meta and "observation_end" not in self.meta:
            import rospy
            self.meta.update(instability_stamp=rospy.Time.now().to_sec(), instability_state=msg.data)

    def wait(self, predicate, timeout, reason, check_clock=False):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() > deadline:
                raise RuntimeError(reason)
            if check_clock:
                self.check_clock()
            time.sleep(0.025)

    def check_clock(self):
        import rospy
        clock = rospy.Time.now().to_sec()
        if self.last_clock != clock:
            self.last_clock, self.last_clock_wall = clock, time.monotonic()
        elif time.monotonic() - self.last_clock_wall > self.config["clock_stall_timeout"]:
            raise RuntimeError("simulation_clock_stalled")
        if time.monotonic() - self.start_wall > self.config["case_wall_timeout"]:
            raise RuntimeError("case_wall_timeout")

    def run(self):
        import rosgraph
        import rospy
        from aerial_robot_msgs.msg import FullStateTarget
        from geometry_msgs.msg import PoseStamped
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Empty, UInt8, String, Time
        from visualization_msgs.msg import MarkerArray
        if not self.args.attach:
            port = free_port()
            gazebo_port = free_port()
            while gazebo_port == port:
                gazebo_port = free_port()
            os.environ["ROS_MASTER_URI"] = "http://127.0.0.1:{}".format(port)
            os.environ["GAZEBO_MASTER_URI"] = "http://127.0.0.1:{}".format(gazebo_port)
            os.environ["ROS_IP"] = "127.0.0.1"
            os.environ.pop("ROS_HOSTNAME", None)
            self.launch("master", ["roscore", "-p", str(port)])
        self.wait(rosgraph.is_master_online, 30, "ROS master unavailable")
        # rospy chooses its clock at initialization. Set this before Gazebo starts.
        if not self.args.attach:
            rospy.set_param("/use_sim_time", True)
        elif not rospy.get_param("/use_sim_time", False):
            raise RuntimeError("--attach requires /use_sim_time=true before node initialization")
        rospy.init_node("robustness_trial", disable_signals=True)
        subscriptions = [
            ("root", "/dragon/root/pose", PoseStamped),
            ("tail", "/dragon/root/tail_pose", PoseStamped),
            ("joints", "/dragon/joint_states", JointState),
            ("flight", "/dragon/flight_state", UInt8),
            ("initial", "/dragon/robustness/initial_target", FullStateTarget),
            ("skip", "/dragon/copilot/first_skip_stamp", Time),
            ("reference", "/dragon/root/target_pose", PoseStamped),
            ("event", "/dragon/robustness/trial_event", String),
        ]
        self.subscribers = [rospy.Subscriber(topic, kind, lambda msg, k=key: self.receive(k, msg),
                                              queue_size=5000) for key, topic, kind in subscriptions]
        if not self.args.attach:
            self.launch("simulation", ["roslaunch", "multilink_copilot", "projection_robustness_sim.launch",
                                       "headless:=" + ("false" if self.args.gui else "true")])
        self.wait(lambda: all(k in self.latest for k in ("root", "tail", "joints", "flight", "initial"))
                  and rospy.Time.now().to_sec() > 0,
                  self.config["startup_timeout"], "simulation_startup_failed")
        if not rospy.get_param("/use_sim_time", False) or not rospy.has_param("/gazebo"):
            raise RuntimeError("Only a Gazebo simulation with /use_sim_time is supported")
        nodes = rosgraph.Master("/robustness_trial").getSystemState()
        producers = {topic: names for topic, names in nodes[0]}
        for topic in ("/dragon/root/target_pose", "/dragon/full_state_target",
                      "/dragon/mono_planner/traj_marker", "/waypoint/ring_markers",
                      *("/waypoint/pose_{}".format(i) for i in range(4))):
            if producers.get(topic):
                raise RuntimeError("Existing publisher on {}: {}".format(topic, producers[topic]))

        # Shared by automatic and --attach trials; visualization is not added to the bag topic set.
        self.subscribers.append(rospy.Subscriber("/dragon/mono_planner/traj_marker", MarkerArray,
                                                lambda msg: self.receive("visualization", msg), queue_size=1))
        visualization = self.launch("visualization", ["rosrun", "multilink_copilot", "robustness_reference_publisher.py",
            "--case", self.args.case_file, "--duration", str(self.config["trajectory_time"]),
            "--rate", str(self.config["reference_rate"]), "--visualization-only"])
        self.wait(lambda: "visualization" in self.latest, 30, "visualization_startup_failed", True)

        topics = [topic for _, topic, _ in subscriptions] + ["/clock"]
        recorder = self.launch("recorder", ["rosbag", "record", "--lz4", "--buffsize=256", "-O",
                                           str(self.output / "trial.bag"), "__name:=robustness_recorder"] + topics)
        def recorder_ready():
            return any("/robustness_recorder" in names for topic, names in
                       rosgraph.Master("/robustness_trial").getSystemState()[1]
                       if topic == "/dragon/robustness/trial_event")
        self.wait(recorder_ready, 20, "recorder_startup_failed")
        arm = rospy.Publisher("/dragon/teleop_command/start", Empty, queue_size=1)
        takeoff = rospy.Publisher("/dragon/teleop_command/takeoff", Empty, queue_size=1)
        if self.latest["flight"].data != 5:
            self.wait(lambda: arm.get_num_connections() and takeoff.get_num_connections(), 10, "no navigator")
            arm.publish(Empty())
            time.sleep(2.0)
            takeoff.publish(Empty())
            self.wait(lambda: self.latest["flight"].data == 5, 60, "takeoff_failed", True)

        initial = self.latest["initial"]
        reset_pub = rospy.Publisher("/dragon/full_state_target", FullStateTarget, queue_size=1)
        self.wait(lambda: reset_pub.get_num_connections() > 0, 10, "no full-state subscriber")
        deadline = time.monotonic() + self.config["initialization_timeout"]
        settled_since = None
        next_publish = 0.0
        while True:
            self.check_clock()
            if time.monotonic() > deadline:
                raise RuntimeError("initialization_not_settled")
            if time.monotonic() >= next_publish:
                initial.header.stamp = initial.root_state.header.stamp = initial.joint_state.header.stamp = rospy.Time.now()
                reset_pub.publish(initial)
                next_publish = time.monotonic() + 0.1
            p, q = self.latest["tail"].pose.position, self.latest["tail"].pose.orientation
            measured = dict(zip(self.latest["joints"].name, self.latest["joints"].position))
            pos_error = math.sqrt((p.x-0.5)**2 + p.y**2 + (p.z-1.0)**2)
            rot_error = quaternion_error([q.x, q.y, q.z, q.w], [0, 0, 0, 1])
            joint_error = max(abs(measured.get(n, math.inf)-v)
                              for n, v in zip(initial.joint_state.name, initial.joint_state.position))
            now = rospy.Time.now().to_sec()
            fresh = all(time.monotonic()-self.received[k] < 1.0 for k in ("tail", "joints", "flight"))
            settled = fresh and self.latest["flight"].data == 5 and pos_error <= self.config["position_tolerance"] \
                and rot_error <= self.config["rotation_tolerance"] and joint_error <= self.config["joint_tolerance"]
            settled_since = (now if settled_since is None else settled_since) if settled else None
            self.meta["initial_errors"] = dict(position=pos_error, rotation=rot_error, joint=joint_error)
            if settled_since is not None and now-settled_since >= self.config["settle_time"]:
                break
            time.sleep(0.025)
        reset_pub.unregister()
        copilot = self.launch("copilot", ["roslaunch", "multilink_copilot", "copilot_planner.launch",
                                         "publish_skip_event:=true", "target_pose_frame_type:=FLU"])
        self.wait(lambda: "skip" in self.latest, 30, "copilot_startup_failed", True)
        (self.output / "parameters.yaml").write_text(yaml.safe_dump(rospy.get_param("/")))
        self.meta["initial_target"] = dict(root=[initial.root_state.pose.pose.position.x,
            initial.root_state.pose.pose.position.y, initial.root_state.pose.pose.position.z],
            root_to_tail=[0.5-initial.root_state.pose.pose.position.x,
                          -initial.root_state.pose.pose.position.y, 1.0-initial.root_state.pose.pose.position.z],
            joint_names=list(initial.joint_state.name), joint_positions=list(initial.joint_state.position))
        self.meta["status"] = "running"
        save_json(self.output / "trial.json", self.meta)
        reference = self.launch("reference", ["rosrun", "multilink_copilot", "robustness_reference_publisher.py",
            "--case", self.args.case_file, "--duration", str(self.config["trajectory_time"]),
            "--rate", str(self.config["reference_rate"])])
        finished = None
        while True:
            self.check_clock()
            if "instability_state" in self.meta:
                raise RuntimeError("left_hover_state_{}".format(self.meta["instability_state"]))
            if time.monotonic()-self.received["flight"] > 10:
                raise RuntimeError("controller_state_stale")
            if recorder.poll() is not None or copilot.poll() is not None:
                raise RuntimeError("recorder_or_copilot_exited")
            if visualization.poll() is not None:
                raise RuntimeError("visualization_exited")
            finished_events = [e for e in self.events if e["phase"] == "finished"]
            if finished_events:
                finished = finished_events[-1]
                if rospy.Time.now().to_sec() >= finished["stamp"] + self.config["post_time"]:
                    break
            elif reference.poll() is not None:
                raise RuntimeError("reference_exited_without_completion")
            time.sleep(0.025)
        expected = int(math.ceil(self.config["trajectory_time"] * self.config["reference_rate"])) + 1
        if finished["samples"] != expected or self.reference_count != expected:
            raise RuntimeError("reference_sample_count_mismatch: {} != {}".format(self.reference_count, expected))
        self.meta["status"] = "completed"
        self.meta["observation_end"] = rospy.Time.now().to_sec()

    def finish(self):
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if any(e["phase"] == "started" for e in self.events):
            import rospy
            self.meta.setdefault("observation_end", rospy.Time.now().to_sec())
        self.meta["events"] = self.events
        self.meta["reference_samples_received"] = self.reference_count
        self.meta["wall_seconds"] = time.monotonic()-self.start_wall
        self.meta["finished_utc"] = datetime.now(timezone.utc).isoformat()
        self.meta["first_skip_stamp"] = self.latest["skip"].data.to_sec() if "skip" in self.latest else None
        # Stop command generation, then close the bag before shutting down the simulator/master.
        for name, process in reversed(self.processes):
            if name in ("reference", "copilot"):
                stop(process)
        for name, process in reversed(self.processes):
            if name == "recorder":
                stop(process)
        for name, process in reversed(self.processes):
            if name not in ("reference", "copilot", "recorder"):
                stop(process)
        for log in self.logs:
            log.close()
        save_json(self.output / "trial.json", self.meta)
        if self.temp_logs is not None:
            save_error_excerpt(self.output, self.log_dir, self.meta)
            self.temp_logs.cleanup()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", type=int, help="Case number for a single manual run; requires --attach")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--gui", action="store_true", default=True,
                        help="Show simulation windows (enabled by default)")
    parser.add_argument("--attach", action="store_true",
                        help="Use a manually started simulator, stream planner output, and overwrite the selected case unless --resume")
    parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    # Per-case paths are passed internally to isolated workers, not supplied by the user.
    parser.add_argument("--case-file", help=argparse.SUPPRESS)
    parser.add_argument("--output", help=argparse.SUPPRESS)
    parser.add_argument("--config", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.worker:
        if not all((args.output, args.config, args.case_file)) or args.case is not None:
            parser.error("Internal worker requires case-file, output, and configuration paths")
    else:
        if any(value is not None for value in (args.output, args.config, args.case_file)):
            parser.error("Edit the path constants at the top of this script; path arguments are internal to workers")
        if args.attach:
            if args.case is None or args.case <= 0:
                parser.error("--attach requires --case with a positive case number")
        elif args.case is not None:
            parser.error("--case requires --attach; automatic runs use the complete dataset")
    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    pkg = package_dir()
    config_path = Path(args.config or CONFIG_FILE).resolve()
    config = yaml.safe_load(config_path.read_text())
    default_output = MANUAL_OUTPUT_DIR if args.attach else OUTPUT_DIR
    output = Path(args.output or default_output).resolve()
    args.output = str(output)
    output.mkdir(parents=True, exist_ok=True)
    if args.worker:
        trial = Trial(args, config)
        try:
            trial.run()
        except KeyboardInterrupt:
            trial.meta.update(status="interrupted", reason="interrupted")
        except Exception as exc:
            import traceback
            traceback.print_exc()
            trial.meta.update(status="failed", reason=str(exc))
        finally:
            trial.finish()
        return 0 if trial.meta["status"] == "completed" else 1
    with (output / ".runner.lock").open("a+") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            parser.error("Another batch runner owns this output directory")
        return run_batch(args, parser, pkg, config, config_path, output)


def run_batch(args, parser, pkg, config, config_path, output):
    config_checksum = digest(config_path)
    case_dir = Path(CASE_DIR)
    if args.attach:
        case = case_dir / "case_{:04d}.yaml".format(args.case)
        if not case.is_file():
            parser.error("Manual case does not exist: {}".format(case))
        files = [case]
    else:
        files = sorted(case_dir.glob("case_*.yaml"))
        if len(files) != 100:
            parser.error("Expected the complete 100-case dataset")
    signature = dict(config=config, cases={p.name: digest(p) for p in files})
    runtime_sources = {str(p.relative_to(pkg)): digest(p) for p in sorted(pkg.rglob("*"))
                       if p.is_file() and (p.suffix in (".cpp", ".h", ".msg", ".yaml", ".launch", ".xml") or
                          p.name in ("run_projection_robustness_batch.py", "robustness_reference_publisher.py"))}
    manifest_path = output / "manifest.json"
    overwrite = args.attach and not args.resume
    if manifest_path.exists() and not overwrite:
        old = json.loads(manifest_path.read_text())
        if not args.resume or old["signature"] != signature:
            parser.error("Existing output requires --resume with identical configuration and case selection")
        if old.get("runtime_sources") != runtime_sources:
            parser.error("Runtime sources changed; use a new output directory to avoid mixing experiments")
    else:
        def git_info(path):
            return dict(commit=subprocess.check_output(["git", "-C", str(path), "rev-parse", "HEAD"], text=True).strip(),
                        status=subprocess.check_output(["git", "-C", str(path), "status", "--short"], text=True),
                        diff=subprocess.check_output(["git", "-C", str(path), "diff", "HEAD"], text=True))
        import rospkg
        dragon = Path(rospkg.RosPack().get_path("dragon"))
        save_json(manifest_path, dict(signature=signature, created_utc=datetime.now(timezone.utc).isoformat(),
                  code=dict(motion_planning=git_info(pkg), dragon=git_info(dragon)), runtime_sources=runtime_sources,
                  package_sources={str(p.relative_to(pkg)): digest(p) for p in sorted(pkg.rglob("*"))
                                   if p.is_file() and p.suffix in (".py", ".cpp", ".h", ".msg", ".yaml", ".launch", ".xml", ".txt")}))
    results = []
    for index, case in enumerate(files, 1):
        if digest(config_path) != config_checksum:
            raise RuntimeError("Experiment configuration changed during the batch")
        if any(not (pkg / name).exists() or digest(pkg / name) != checksum
               for name, checksum in runtime_sources.items()):
            raise RuntimeError("Runtime sources changed during the batch; stop rather than mix versions")
        if digest(case) != signature["cases"][case.name]:
            raise RuntimeError("Case input changed during the batch: {}".format(case))
        dest = output / case.stem
        trial_json = dest / "trial.json"
        if dest.exists() and overwrite:
            shutil.rmtree(dest)
        elif dest.exists():
            previous = json.loads(trial_json.read_text()) if trial_json.exists() else {}
            if args.resume and previous.get("status") in ("completed", "failed"):
                results.append(previous)
                continue
            if not args.resume:
                raise RuntimeError("Refusing to overwrite {}".format(dest))
            archived = output / "interrupted" / (case.stem + "_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S"))
            archived.parent.mkdir(exist_ok=True)
            dest.rename(archived)
        dest.mkdir()
        shutil.copy2(case, dest / "case.yaml")
        command = [sys.executable, str(Path(__file__).resolve()), "--worker", "--case-file", str(case.resolve()),
                   "--output", str(dest), "--config", str(config_path)]
        if args.gui: command.append("--gui")
        if args.attach: command.append("--attach")
        print("[{}/{}] {}".format(index, len(files), case.stem), flush=True)
        with tempfile.TemporaryDirectory(prefix="dragon_robustness_") as temporary:
            log_dir = Path(temporary)
            with (log_dir / "worker.log").open("w") as log, \
                 mirror_log(log_dir / "copilot.log", args.attach):
                token = uuid.uuid4().hex
                env = dict(os.environ, DRAGON_ROBUSTNESS_TRIAL_TOKEN=token, DRAGON_ROBUSTNESS_LOG_DIR=temporary)
                if args.attach:
                    # roslaunch/Python and roscpp otherwise buffer stdout redirected to a file.
                    env.update(PYTHONUNBUFFERED="1", ROSCONSOLE_STDOUT_LINE_BUFFERED="1")
                worker = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                interrupted_batch, worker_timed_out = False, False
                try:
                    worker.wait(timeout=config["case_wall_timeout"] + 120)
                except KeyboardInterrupt:
                    stop(worker)
                    interrupted_batch = True
                except subprocess.TimeoutExpired:
                    stop(worker)
                    worker_timed_out = True
                finally:
                    cleanup_trial_processes(token)
            result = json.loads(trial_json.read_text()) if trial_json.exists() else dict(case=case.stem, status="failed", reason="worker_died")
            if worker_timed_out:
                result.update(status="failed", reason="worker_wall_timeout")
            elif interrupted_batch:
                result.update(status="interrupted", reason="batch_interrupted")
            elif result.get("status") not in ("completed", "failed", "interrupted"):
                result.update(status="failed", reason="worker_died")
            save_error_excerpt(dest, log_dir, result)
        save_json(trial_json, result)
        results.append(result)
        save_json(output / "batch.json", results)
        print("  {} {}".format(result["status"], result.get("reason", "")), flush=True)
        if interrupted_batch:
            return 130
    return 0 if all(r["status"] == "completed" for r in results) else 1


if __name__ == "__main__":
    sys.exit(main())
