#!/usr/bin/env python3

import argparse
import atexit
import os
import pty
import shlex
import signal
import shutil
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path

import rosgraph
import rospy
from std_msgs.msg import UInt8


ROBOT_NS = "dragon"
HOVER_STATE = 5
BRINGUP_DELAY_SEC = 5.0
KEY_COMMAND_DELAY_SEC = 1.0
ARM_TO_TAKEOFF_DELAY_SEC = 2.0
HOVER_TIMEOUT_SEC = 40.0
CASE_TIMEOUT_SEC = 55.0
PROCESS_STARTUP_GRACE_SEC = 2.0
PROCESS_STOP_SIGINT_TIMEOUT_SEC = 5.0
PROCESS_STOP_SIGTERM_TIMEOUT_SEC = 5.0
ROS_MASTER_TIMEOUT_SEC = 60.0
TERMINAL_PID_CAPTURE_TIMEOUT_SEC = 5.0
RESIDUAL_PROCESS_STOP_SIGINT_TIMEOUT_SEC = 3.0
RESIDUAL_PROCESS_STOP_SIGTERM_TIMEOUT_SEC = 3.0
PROCESS_SESSION_ENV_VAR = "WAYPOINT_BATCH_RUNNER_SESSION_ID"
RUNNER_LOCK_PATH = Path(tempfile.gettempdir()) / "waypoint_conditioned_maneuvering_batch_runner.lock"

SCRIPT_PATH = Path(__file__).resolve()
PACKAGE_DIR = SCRIPT_PATH.parents[2]
MONO_PLANNER_DIR = PACKAGE_DIR.parent / "mono_planner"
DEFAULT_CASE_DIR = MONO_PLANNER_DIR / "test_data"
DEFAULT_OUTPUT_ROOT = PACKAGE_DIR / "data" / "envelope_width" / "waypoint_conditioned_batch"


def timestamp_string():
    return time.strftime("%Y%m%d%H%M%S", time.localtime())


def short_sleep(duration_sec):
    deadline = time.monotonic() + duration_sec
    while time.monotonic() < deadline:
        time.sleep(min(0.1, deadline - time.monotonic()))


def terminal_tabs_supported():
    return shutil.which("gnome-terminal") is not None and (
        os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
    )


def pid_exists(pid):
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True

    try:
        stat_fields = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8").split()
    except OSError:
        return True

    return len(stat_fields) < 3 or stat_fields[2] != "Z"


@dataclass
class CaseResult:
    case_name: str
    success: bool
    reason: str
    json_path: str = ""


class ManagedProcess:
    def __init__(self, name, command, env=None):
        self.name = name
        self.command = list(command)
        self.env = None if env is None else dict(env)
        self.process = None

    def start(self):
        if self.process is not None:
            raise RuntimeError(f"Process {self.name} is already running.")
        self.process = subprocess.Popen(
            self.command,
            env=self.env,
            stdin=subprocess.DEVNULL,
            stdout=None,
            stderr=None,
            start_new_session=True,
            close_fds=True,
        )

        return self

    def is_running(self):
        return self.process is not None and self.process.poll() is None

    def poll(self):
        if self.process is None:
            return None
        return self.process.poll()

    def pid(self):
        if self.process is None:
            return None
        return self.process.pid

    def stop(self):
        if self.process is None:
            return

        try:
            if self.is_running():
                self._signal_group(signal.SIGINT, PROCESS_STOP_SIGINT_TIMEOUT_SEC)
            if self.is_running():
                self._signal_group(signal.SIGTERM, PROCESS_STOP_SIGTERM_TIMEOUT_SEC)
            if self.is_running():
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait(timeout=1.0)
        except ProcessLookupError:
            pass
        except subprocess.TimeoutExpired:
            pass
        finally:
            self.process = None

    def _signal_group(self, sig, timeout_sec):
        os.killpg(self.process.pid, sig)
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                return
            short_sleep(0.1)


class ManagedTerminalTabProcess:
    def __init__(self, name, command, open_in_new_window, working_directory, env=None):
        self.name = name
        self.command = list(command)
        self.open_in_new_window = open_in_new_window
        self.working_directory = str(working_directory)
        self.env = None if env is None else dict(env)
        self.process = None
        self.command_pid = None
        self.command_pid_file = None

    def start(self):
        if self.process is not None:
            raise RuntimeError(f"Process {self.name} is already running.")

        fd, pid_file = tempfile.mkstemp(prefix=f"{self.name}_", suffix=".pid")
        os.close(fd)
        self.command_pid_file = pid_file

        # gnome-terminal only gives us the terminal client PID, so capture the
        # shell PID before exec and use it as the roslaunch process-group ID.
        shell_command = (
            f"printf '%s\\n' \"$$\" > {shlex.quote(self.command_pid_file)}; "
            f"exec {shlex.join(self.command)}"
        )
        terminal_command = [
            "gnome-terminal",
            "--wait",
            "--window" if self.open_in_new_window else "--tab",
            f"--title={self.name}",
            f"--working-directory={self.working_directory}",
            "--",
            "bash",
            "-lc",
            shell_command,
        ]
        try:
            self.process = subprocess.Popen(
                terminal_command,
                env=self.env,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=None,
                start_new_session=True,
                close_fds=True,
            )
            self.command_pid = self._wait_for_command_pid()
        except Exception:
            self._cleanup_pid_file()
            raise
        return self

    def is_running(self):
        return self.process is not None and self.process.poll() is None

    def poll(self):
        if self.process is None:
            return None
        return self.process.poll()

    def pid(self):
        if self.command_pid is not None:
            return self.command_pid
        if self.process is None:
            return None
        return self.process.pid

    def stop(self):
        if self.process is None:
            self._cleanup_pid_file()
            return

        try:
            if self.is_running():
                self._signal_group(signal.SIGINT, PROCESS_STOP_SIGINT_TIMEOUT_SEC)
            if self.is_running():
                self._signal_group(signal.SIGTERM, PROCESS_STOP_SIGTERM_TIMEOUT_SEC)
            if self.is_running():
                os.killpg(self.pid(), signal.SIGKILL)
                self.process.wait(timeout=1.0)
            if self.is_running():
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait(timeout=1.0)
        except ProcessLookupError:
            pass
        except subprocess.TimeoutExpired:
            pass
        finally:
            self.process = None
            self.command_pid = None
            self._cleanup_pid_file()

    def _wait_for_command_pid(self):
        deadline = time.monotonic() + TERMINAL_PID_CAPTURE_TIMEOUT_SEC
        while time.monotonic() < deadline:
            pid = self._read_command_pid()
            if pid is not None:
                return pid
            if self.process.poll() is not None:
                break
            short_sleep(0.1)

        raise RuntimeError(f"Failed to capture child PID for {self.name} terminal tab.")

    def _read_command_pid(self):
        if self.command_pid_file is None or not os.path.exists(self.command_pid_file):
            return None
        try:
            pid_text = Path(self.command_pid_file).read_text(encoding="utf-8").strip()
        except OSError:
            return None
        if not pid_text:
            return None
        try:
            return int(pid_text)
        except ValueError:
            return None

    def _signal_group(self, sig, timeout_sec):
        os.killpg(self.pid(), sig)
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                return
            short_sleep(0.1)

    def _cleanup_pid_file(self):
        if self.command_pid_file is None:
            return
        try:
            os.unlink(self.command_pid_file)
        except OSError:
            pass
        finally:
            self.command_pid_file = None


class ManagedPtyProcess:
    def __init__(self, name, command, env=None):
        self.name = name
        self.command = list(command)
        self.env = None if env is None else dict(env)
        self.pid = None
        self.master_fd = None
        self.returncode = None

    def start(self):
        if self.pid is not None:
            raise RuntimeError(f"Process {self.name} is already running.")

        pid, master_fd = pty.fork()
        if pid == 0:
            if self.env is None:
                os.execvp(self.command[0], self.command)
            os.execvpe(self.command[0], self.command, self.env)

        self.pid = pid
        self.master_fd = master_fd
        return self

    def is_running(self):
        return self.poll() is None

    def poll(self):
        if self.pid is None:
            return self.returncode
        try:
            waited_pid, status = os.waitpid(self.pid, os.WNOHANG)
        except ChildProcessError:
            waited_pid = self.pid
            status = 0

        if waited_pid == 0:
            return None

        self.returncode = os.waitstatus_to_exitcode(status)
        self._close_master_fd()
        self.pid = None
        return self.returncode

    def write(self, chars):
        if not self.is_running():
            raise RuntimeError(f"Process {self.name} is not running.")
        os.write(self.master_fd, chars.encode("utf-8"))

    def stop(self):
        if self.pid is None:
            self._close_master_fd()
            return

        try:
            if self.is_running():
                self._signal_group(signal.SIGINT, PROCESS_STOP_SIGINT_TIMEOUT_SEC)
            if self.is_running():
                self._signal_group(signal.SIGTERM, PROCESS_STOP_SIGTERM_TIMEOUT_SEC)
            if self.is_running():
                os.killpg(self.pid, signal.SIGKILL)
                os.waitpid(self.pid, 0)
        except ProcessLookupError:
            pass
        except ChildProcessError:
            pass
        finally:
            self.pid = None
            self._close_master_fd()

    def _signal_group(self, sig, timeout_sec):
        os.killpg(self.pid, sig)
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self.poll() is not None:
                return
            short_sleep(0.1)

    def _close_master_fd(self):
        if self.master_fd is None:
            return
        try:
            os.close(self.master_fd)
        except OSError:
            pass
        finally:
            self.master_fd = None


class WaypointConditionedBatchRunner:
    def __init__(self, case_limit=None):
        self.case_limit = case_limit
        self.case_directory = DEFAULT_CASE_DIR
        self.output_root = DEFAULT_OUTPUT_ROOT / timestamp_string()
        self.persistent_processes = []
        self.current_case_processes = []
        self.case_results = []
        self._ros_initialized = False
        self._cleanup_done = False
        self._use_terminal_tabs = terminal_tabs_supported()
        self._terminal_window_opened = False
        self._process_session_id = f"{timestamp_string()}_{os.getpid()}"
        self._lock_acquired = False

    def run(self):
        self.acquire_runner_lock()
        self.cleanup_stale_managed_processes()
        case_files = self.discover_case_files()
        self.output_root.mkdir(parents=True, exist_ok=True)
        if self._use_terminal_tabs:
            print("Launching ROS processes in gnome-terminal tabs.")
        else:
            print("gnome-terminal tabs unavailable; launching ROS processes inline.")

        self.launch_bringup()
        self.launch_keyboard_command_and_takeoff()
        self.launch_persistent_copilot()

        for case_file in case_files:
            result = self.run_case(case_file)
            self.case_results.append(result)

        self.print_summary()
        if any(not result.success for result in self.case_results):
            raise SystemExit(1)

    def discover_case_files(self):
        if not self.case_directory.is_dir():
            raise FileNotFoundError(f"Case directory not found: {self.case_directory}")

        case_files = sorted(self.case_directory.glob("case_*.yaml"))
        if not case_files:
            raise RuntimeError(f"No case_*.yaml files found in {self.case_directory}")

        if self.case_limit is not None:
            case_files = case_files[: self.case_limit]

        print(f"Discovered {len(case_files)} waypoint cases in {self.case_directory}")
        return case_files

    def launch_bringup(self):
        bringup = self.start_process(
            "dragon_bringup",
            [
                "roslaunch",
                "dragon",
                "bringup.launch",
                "rm:=false",
                "sim:=true",
                "headless:=false",
            ],
            persistent=True,
        )
        short_sleep(BRINGUP_DELAY_SEC)
        self.ensure_process_alive(bringup, "Dragon bringup exited before the batch could start.")
        self.wait_for_ros_master()
        self.ensure_ros_node()

    def launch_keyboard_command_and_takeoff(self):
        keyboard = self.start_pty_process(
            "keyboard_command",
            [
                "rosrun",
                "aerial_robot_base",
                "keyboard_command.py",
            ],
            persistent=True,
        )
        short_sleep(PROCESS_STARTUP_GRACE_SEC)
        self.ensure_process_alive(keyboard, "keyboard_command exited before takeoff commands were sent.")
        self.send_takeoff_sequence(keyboard)
        if not self.wait_for_hover_state(HOVER_TIMEOUT_SEC):
            raise RuntimeError(f"Dragon did not reach HOVER state {HOVER_STATE} within {HOVER_TIMEOUT_SEC:.1f} s.")

    def launch_persistent_copilot(self):
        copilot = self.start_process(
            "copilot_planner",
            [
                "roslaunch",
                "multilink_copilot",
                "copilot_planner.launch",
                "target_pose_frame_type:=FLU",
                "evaluation:=false",
            ],
            persistent=True,
        )
        short_sleep(PROCESS_STARTUP_GRACE_SEC)
        self.ensure_process_alive(copilot, "copilot_planner.launch exited during startup.")

    def run_case(self, case_file):
        case_name = case_file.stem
        case_output_dir = self.output_root / case_name
        case_output_dir.mkdir(parents=True, exist_ok=True)
        relative_case_path = case_file.relative_to(MONO_PLANNER_DIR).as_posix()
        self.current_case_processes = []

        print(f"\n=== Running {case_name} ===")
        print(f"Case file: {relative_case_path}")
        print(f"Output dir: {case_output_dir}")

        try:
            evaluator = self.start_process(
                f"{case_name}_evaluation",
                [
                    "roslaunch",
                    "multilink_copilot",
                    "run_evaluation.launch",
                    f"robot_ns:={ROBOT_NS}",
                    f"output_dir:={case_output_dir}",
                ],
            )
            short_sleep(PROCESS_STARTUP_GRACE_SEC)
            self.ensure_process_alive(evaluator, f"Evaluation launch exited during startup for {case_name}.")

            publisher = self.start_process(
                f"{case_name}_publisher",
                [
                    "roslaunch",
                    "mono_planner",
                    "waypoint_pose_publisher.launch",
                    f"config_file:={relative_case_path}",
                    "launch_rviz:=true",
                ],
            )
            short_sleep(PROCESS_STARTUP_GRACE_SEC)
            self.ensure_process_alive(publisher, f"Waypoint publisher exited during startup for {case_name}.")

            planner = self.start_process(
                f"{case_name}_planner",
                [
                    "roslaunch",
                    "mono_planner",
                    "waypoint_conditioned_planner.launch",
                    "nonholo:=true",
                    "robot_frame_type:=LINK",
                    "total_trajectory_time:=50",
                    f"ns:={ROBOT_NS}",
                ],
            )
            short_sleep(PROCESS_STARTUP_GRACE_SEC)
            self.ensure_process_alive(planner, f"Waypoint-conditioned planner exited during startup for {case_name}.")

            return self.wait_for_case_completion(case_name, publisher, planner, evaluator, case_output_dir)
        except Exception as exc:
            return CaseResult(case_name=case_name, success=False, reason=str(exc))
        finally:
            self.stop_current_case_processes()

    def wait_for_case_completion(self, case_name, publisher, planner, evaluator, case_output_dir):
        deadline = time.monotonic() + CASE_TIMEOUT_SEC
        while time.monotonic() < deadline:
            if publisher.poll() is not None:
                return CaseResult(case_name=case_name, success=False, reason="Waypoint publisher exited unexpectedly.")
            if planner.poll() is not None:
                return CaseResult(case_name=case_name, success=False, reason="Waypoint-conditioned planner exited unexpectedly.")
            if evaluator.poll() is not None:
                json_files = sorted(case_output_dir.glob("*.json"))
                if not json_files:
                    return CaseResult(case_name=case_name, success=False, reason="Evaluation exited without writing JSON.")
                return CaseResult(
                    case_name=case_name,
                    success=True,
                    reason="completed",
                    json_path=str(json_files[-1]),
                )
            short_sleep(0.5)

        return CaseResult(case_name=case_name, success=False, reason=f"Timed out after {CASE_TIMEOUT_SEC:.1f} s.")

    def start_process(self, name, command, persistent=False):
        print(f"Starting {name}: {' '.join(command)}")
        if self._use_terminal_tabs:
            process = ManagedTerminalTabProcess(
                name,
                command,
                open_in_new_window=not self._terminal_window_opened,
                working_directory=Path.cwd(),
                env=self.child_process_env(),
            ).start()
            self._terminal_window_opened = True
        else:
            process = ManagedProcess(name, command, env=self.child_process_env()).start()
        if persistent:
            self.persistent_processes.append(process)
        else:
            self.current_case_processes.append(process)
        return process

    def start_pty_process(self, name, command, persistent=False):
        print(f"Starting {name}: {' '.join(command)}")
        process = ManagedPtyProcess(name, command, env=self.child_process_env()).start()
        if persistent:
            self.persistent_processes.append(process)
        else:
            self.current_case_processes.append(process)
        return process

    def stop_current_case_processes(self):
        while self.current_case_processes:
            process = self.current_case_processes.pop()
            print(f"Stopping {process.name}")
            process.stop()

    def cleanup(self):
        if self._cleanup_done:
            return
        self._cleanup_done = True

        previous_sigint = signal.getsignal(signal.SIGINT)
        previous_sigterm = signal.getsignal(signal.SIGTERM)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        try:
            if self._ros_initialized and not rospy.is_shutdown():
                rospy.signal_shutdown("Batch runner cleanup requested.")

            self.stop_current_case_processes()
            while self.persistent_processes:
                process = self.persistent_processes.pop()
                print(f"Stopping {process.name}")
                process.stop()

            self.cleanup_residual_managed_processes()
        finally:
            self.release_runner_lock()
            signal.signal(signal.SIGINT, previous_sigint)
            signal.signal(signal.SIGTERM, previous_sigterm)

    def ensure_process_alive(self, process, message):
        if not process.is_running():
            raise RuntimeError(message)

    def send_takeoff_sequence(self, keyboard_process):
        # keyboard_command.py reads raw one-character stdin in a terminal and only
        # recognizes lowercase commands such as 'r' and 't'.
        keyboard_process.write("r")
        short_sleep(KEY_COMMAND_DELAY_SEC)
        short_sleep(ARM_TO_TAKEOFF_DELAY_SEC)
        keyboard_process.write("t")
        short_sleep(KEY_COMMAND_DELAY_SEC)

    def wait_for_ros_master(self):
        deadline = time.monotonic() + ROS_MASTER_TIMEOUT_SEC
        while time.monotonic() < deadline:
            if rosgraph.is_master_online():
                return
            short_sleep(0.5)
        raise RuntimeError(f"ROS master did not become available within {ROS_MASTER_TIMEOUT_SEC:.1f} s.")

    def ensure_ros_node(self):
        if self._ros_initialized:
            return

        rospy.init_node(
            "waypoint_conditioned_maneuvering_batch_runner",
            anonymous=False,
            disable_signals=True,
        )
        self._ros_initialized = True

    def wait_for_hover_state(self, timeout_sec):
        hover_topic = f"/{ROBOT_NS}/flight_state"
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            remaining = max(0.1, deadline - time.monotonic())
            try:
                msg = rospy.wait_for_message(hover_topic, UInt8, timeout=min(1.0, remaining))
            except rospy.ROSException:
                continue
            if msg.data == HOVER_STATE:
                print(f"Reached HOVER state on {hover_topic}.")
                return True
        return False

    def print_summary(self):
        passed = [result for result in self.case_results if result.success]
        failed = [result for result in self.case_results if not result.success]

        print("\n=== Batch Summary ===")
        print(f"Output root: {self.output_root}")
        print(f"Passed: {len(passed)}")
        for result in passed:
            print(f"  PASS {result.case_name}: {result.json_path}")
        print(f"Failed: {len(failed)}")
        for result in failed:
            print(f"  FAIL {result.case_name}: {result.reason}")

    def child_process_env(self):
        env = os.environ.copy()
        env[PROCESS_SESSION_ENV_VAR] = self._process_session_id
        return env

    def acquire_runner_lock(self):
        existing_pid = self._read_runner_lock_pid()
        if existing_pid is not None and existing_pid != os.getpid() and pid_exists(existing_pid):
            raise RuntimeError(
                f"Another waypoint batch runner is still active with PID {existing_pid}. "
                "Stop it before starting a new batch."
            )

        RUNNER_LOCK_PATH.write_text(f"{os.getpid()}\n", encoding="utf-8")
        self._lock_acquired = True

    def release_runner_lock(self):
        if not self._lock_acquired:
            return

        existing_pid = self._read_runner_lock_pid()
        if existing_pid == os.getpid():
            try:
                RUNNER_LOCK_PATH.unlink()
            except FileNotFoundError:
                pass
        self._lock_acquired = False

    def cleanup_stale_managed_processes(self):
        stale_pids = self.find_managed_process_pids()
        if not stale_pids:
            return

        print(f"Cleaning up {len(stale_pids)} managed processes left by a previous batch run.")
        self.stop_managed_pids(stale_pids)

    def cleanup_residual_managed_processes(self):
        residual_pids = self.find_managed_process_pids(session_id=self._process_session_id)
        if not residual_pids:
            return

        print(f"Cleaning up {len(residual_pids)} residual managed ROS/Gazebo processes.")
        self.stop_managed_pids(residual_pids)

    def find_managed_process_pids(self, session_id=None):
        pids = []
        target = None if session_id is None else f"{PROCESS_SESSION_ENV_VAR}={session_id}".encode("utf-8")
        prefix = f"{PROCESS_SESSION_ENV_VAR}=".encode("utf-8")

        for proc_dir in Path("/proc").iterdir():
            if not proc_dir.name.isdigit():
                continue

            pid = int(proc_dir.name)
            if pid == os.getpid():
                continue

            try:
                environ = (proc_dir / "environ").read_bytes()
            except OSError:
                continue

            entries = environ.split(b"\0")
            if target is not None:
                if target not in entries:
                    continue
            elif not any(entry.startswith(prefix) for entry in entries):
                continue

            pids.append(pid)

        return sorted(set(pids))

    def stop_managed_pids(self, pids):
        remaining = [pid for pid in sorted(set(pids)) if pid > 1 and pid_exists(pid)]
        if not remaining:
            return

        for sig, timeout_sec in (
            (signal.SIGINT, RESIDUAL_PROCESS_STOP_SIGINT_TIMEOUT_SEC),
            (signal.SIGTERM, RESIDUAL_PROCESS_STOP_SIGTERM_TIMEOUT_SEC),
            (signal.SIGKILL, 0.0),
        ):
            for pid in remaining:
                try:
                    os.kill(pid, sig)
                except ProcessLookupError:
                    pass
                except PermissionError:
                    pass

            if sig == signal.SIGKILL:
                deadline = time.monotonic() + 1.0
                while time.monotonic() < deadline:
                    remaining = [pid for pid in remaining if pid_exists(pid)]
                    if not remaining:
                        return
                    short_sleep(0.1)
                break

            deadline = time.monotonic() + timeout_sec
            while time.monotonic() < deadline:
                remaining = [pid for pid in remaining if pid_exists(pid)]
                if not remaining:
                    return
                short_sleep(0.1)

        final_remaining = [pid for pid in remaining if pid_exists(pid)]
        if final_remaining:
            print(f"Warning: failed to stop managed processes {final_remaining}", file=sys.stderr)

    def _read_runner_lock_pid(self):
        try:
            pid_text = RUNNER_LOCK_PATH.read_text(encoding="utf-8").strip()
        except FileNotFoundError:
            return None
        except OSError:
            return None

        if not pid_text:
            return None

        try:
            return int(pid_text)
        except ValueError:
            return None


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Run waypoint-conditioned maneuvering cases in batch.")
    parser.add_argument(
        "--case-limit",
        type=int,
        default=None,
        help="Run only the first N case_*.yaml files after lexicographic sorting.",
    )
    return parser.parse_args(argv)


def install_signal_handlers():
    def _raise_keyboard_interrupt(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, _raise_keyboard_interrupt)
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)


def main():
    install_signal_handlers()
    args = parse_args(rospy.myargv(argv=sys.argv)[1:])
    runner = WaypointConditionedBatchRunner(case_limit=args.case_limit)
    atexit.register(runner.cleanup)

    try:
        runner.run()
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted; stopping batch runner.")
        return 130
    except Exception as exc:
        print(f"\nBatch runner failed: {exc}", file=sys.stderr)
        return 1
    finally:
        runner.cleanup()


if __name__ == "__main__":
    raise SystemExit(main())
