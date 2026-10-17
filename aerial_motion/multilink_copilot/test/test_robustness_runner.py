#!/usr/bin/env python3
"""Exercise batch failure handling without launching a simulator."""
import fcntl
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest
from contextlib import redirect_stdout
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts/evaluation"))
import run_projection_robustness_batch as runner


class RunnerTest(unittest.TestCase):
    def test_planner_output_is_visible_before_process_exit_and_log_is_retained(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "copilot.log"
            console = io.StringIO()
            with redirect_stdout(console), runner.mirror_log(path, True):
                # The log appears after monitoring starts, as in a real trial.
                with path.open("w") as log:
                    process = subprocess.Popen([sys.executable, "-u", "-c",
                        "import sys; print('planner ready'); "
                        "print('Skipping full_state_target: joint step', file=sys.stderr); "
                        "sys.stdin.readline(); print('planner stopped')"],
                        stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.PIPE, text=True)
                    try:
                        deadline = time.monotonic() + 5
                        while "Skipping full_state_target" not in console.getvalue() and time.monotonic() < deadline:
                            time.sleep(0.01)
                        self.assertIn("planner ready", console.getvalue())
                        self.assertIn("Skipping full_state_target", console.getvalue())
                        self.assertIsNone(process.poll())
                        process.communicate("stop\n", timeout=5)
                    finally:
                        if process.poll() is None:
                            process.kill()
                        process.communicate(timeout=5)
            self.assertEqual(console.getvalue(), path.read_text())
            self.assertIn("planner stopped", console.getvalue())

    def test_log_mirroring_handles_missing_log_and_disabled_mode(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "copilot.log"
            console = io.StringIO()
            with redirect_stdout(console):
                with runner.mirror_log(path, True):
                    pass  # Initialization may fail before copilot starts.
                path.write_text("planner output\n")
                with runner.mirror_log(path, False):
                    pass
            self.assertEqual(console.getvalue(), "")

    def test_attach_streams_output_and_overwrites_selected_case_by_default(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp)
            console = io.StringIO()
            real_popen = subprocess.Popen
            worker_calls = []

            def popen(command, *args, **kwargs):
                if "--worker" not in command:
                    return real_popen(command, *args, **kwargs)
                worker_calls.append(command)
                self.assertIn("--attach", command)
                self.assertFalse((output / "case_0001/trial.bag").exists())
                self.assertFalse((output / "case_0001/error.log").exists())
                env = kwargs["env"]
                self.assertEqual(env["PYTHONUNBUFFERED"], "1")
                self.assertEqual(env["ROSCONSOLE_STDOUT_LINE_BUFFERED"], "1")
                (Path(env["DRAGON_ROBUSTNESS_LOG_DIR"]) / "copilot.log").write_text("planner skip warning\n")
                (output / "case_0001/trial.json").write_text(json.dumps(dict(case="case_0001", status="completed")))
                return real_popen([sys.executable, "-c", "pass"], *args, **kwargs)

            with patch.object(sys, "argv", ["runner", "--cases", "1", "--attach"]), \
                 patch.object(runner, "MANUAL_OUTPUT_DIR", output), \
                 patch.object(runner.subprocess, "Popen", side_effect=popen), \
                 patch.object(runner, "cleanup_trial_processes"), redirect_stdout(console):
                self.assertEqual(runner.main(), 0)
                (output / "case_0001/trial.bag").write_text("old bag")
                (output / "case_0001/error.log").write_text("old failure")
                (output / "case_0001/trial.json").write_text("incomplete old metadata")
                # Manual reruns also replace stale provenance from changed code/configuration/case selection.
                (output / "manifest.json").write_text(json.dumps(dict(signature="old", runtime_sources={})))
                other_case = output / "case_0002"
                other_case.mkdir()
                (other_case / "trial.bag").write_text("keep other case")
                self.assertEqual(runner.main(), 0)
                self.assertEqual(len(worker_calls), 2)
                self.assertEqual((other_case / "trial.bag").read_text(), "keep other case")
                manifest = json.loads((output / "manifest.json").read_text())
                self.assertEqual(set(manifest["signature"]["cases"]), {"case_0001.yaml"})
                self.assertTrue(manifest["runtime_sources"])
                self.assertEqual(len(json.loads((output / "batch.json").read_text())), 1)
                with patch.object(sys, "argv", ["runner", "--cases", "1", "--attach", "--resume"]):
                    self.assertEqual(runner.main(), 0)
                self.assertEqual(len(worker_calls), 2)
            self.assertEqual(console.getvalue().count("planner skip warning"), 2)

    def test_automatic_run_refuses_existing_output_without_resume(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp)
            manifest = output / "manifest.json"
            manifest.write_text("{}")
            with patch.object(sys, "argv", ["runner", "--cases", "1"]), \
                 patch.object(runner, "SELECTED_OUTPUT_DIR", output):
                with self.assertRaises(SystemExit) as result:
                    runner.main()
                self.assertEqual(result.exception.code, 2)
            self.assertEqual(manifest.read_text(), "{}")

    def invoke(self, output, failure):
        real_popen = subprocess.Popen
        calls = []
        class Worker:
            pid = 99999999
            def __init__(self, command):
                self.output = Path(command[command.index("--output")+1])
                calls.append(self.output.name)
            def wait(self, timeout):
                if len(calls) == 1:
                    if failure == "timeout": raise subprocess.TimeoutExpired("worker", timeout)
                    raise KeyboardInterrupt
                (self.output / "trial.json").write_text(json.dumps(dict(case=self.output.name, status="completed")))
                return 0
        def popen(command, *args, **kwargs):
            return Worker(command) if "--worker" in command else real_popen(command, *args, **kwargs)
        with patch.object(sys, "argv", ["runner", "--cases", "1", "2"]), \
             patch.object(runner, "SELECTED_OUTPUT_DIR", output), \
             patch.object(runner.subprocess, "Popen", side_effect=popen), \
             patch.object(runner, "stop") as stop, \
             patch.object(runner, "cleanup_trial_processes") as cleanup:
            result = runner.main()
            self.assertEqual(stop.call_count, 1)
            self.assertEqual(cleanup.call_count, len(calls))
        return result, calls

    def test_worker_timeout_is_recorded_and_next_case_runs(self):
        with tempfile.TemporaryDirectory() as tmp:
            result, calls = self.invoke(Path(tmp), "timeout")
            self.assertEqual(result, 1)
            self.assertEqual(calls, ["case_0001", "case_0002"])
            batch = json.loads((Path(tmp) / "batch.json").read_text())
            self.assertEqual(batch[0]["reason"], "worker_wall_timeout")
            self.assertEqual(batch[1]["status"], "completed")
            self.assertTrue((Path(tmp)/"case_0001/error.log").exists())
            self.assertFalse(list(Path(tmp).rglob("worker.log")))

    def test_interrupt_preserves_status_and_stops_batch(self):
        with tempfile.TemporaryDirectory() as tmp:
            result, calls = self.invoke(Path(tmp), "interrupt")
            self.assertEqual(result, 130)
            self.assertEqual(calls, ["case_0001"])
            record = json.loads((Path(tmp) / "case_0001/trial.json").read_text())
            self.assertEqual(record["status"], "interrupted")

    def test_same_output_cannot_have_two_runners(self):
        with tempfile.TemporaryDirectory() as tmp:
            with (Path(tmp) / ".runner.lock").open("w") as lock:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                with patch.object(sys, "argv", ["runner", "--cases", "1"]), \
                     patch.object(runner, "SELECTED_OUTPUT_DIR", Path(tmp)), \
                     patch.object(runner.subprocess, "Popen") as spawn:
                    with self.assertRaises(SystemExit) as result:
                        runner.main()
                    self.assertEqual(result.exception.code, 2)
                    spawn.assert_not_called()

    def test_fixed_output_directories_separate_batch_selected_and_manual_runs(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            paths = dict(OUTPUT_DIR=root/"batch", SELECTED_OUTPUT_DIR=root/"selected", MANUAL_OUTPUT_DIR=root/"manual")
            for flags, key in (([], "OUTPUT_DIR"), (["--resume"], "OUTPUT_DIR"),
                               (["--cases", "1"], "SELECTED_OUTPUT_DIR"),
                               (["--cases", "1", "--resume"], "SELECTED_OUTPUT_DIR"),
                               (["--case-limit", "2"], "SELECTED_OUTPUT_DIR"),
                               (["--cases", "1", "--attach"], "MANUAL_OUTPUT_DIR")):
                with self.subTest(flags=flags), patch.multiple(runner, **paths), \
                     patch.object(sys, "argv", ["runner"]+flags), \
                     patch.object(runner, "run_batch", return_value=0) as batch:
                    self.assertEqual(runner.main(), 0)
                    self.assertEqual(batch.call_args.args[-1], paths[key])

    def test_logs_are_bounded_and_only_saved_for_infrastructure_failure(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            logs, output = root/"logs", root/"output"
            logs.mkdir(); output.mkdir()
            for i in range(12):
                (logs/"{}.log".format(i)).write_text("some log line\n"*1000)
            for result in (dict(status="completed"), dict(status="interrupted"),
                           dict(status="failed", reason="left_hover_state_17")):
                runner.save_error_excerpt(output, logs, result)
                self.assertFalse((output/"error.log").exists())
            runner.save_error_excerpt(output, logs, dict(status="failed", reason="worker_died"))
            self.assertLessEqual(len((output/"error.log").read_text().splitlines()), 100)


if __name__ == "__main__":
    unittest.main()
