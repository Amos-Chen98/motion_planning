#!/usr/bin/env python3
"""Build identical benchmark sources against archived/current ABIs and compare.

Run after sourcing the workspace. The baseline directory must contain source/
{motion_primitive_planner,multilink_copilot} and their unmodified shared libraries.
All ROS parameters and nodes belong to an isolated temporary ROS master.
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import shlex
import socket
import statistics
import subprocess
import time
import xmlrpc.client


def compile_harness(package, build, baseline, output, env):
    flag_lines = (build / "CMakeFiles/motion_primitive_planner_core.dir/flags.make").read_text().splitlines()
    flags = {}
    for line in flag_lines:
        if " = " in line:
            key, value = line.split(" = ", 1)
            flags[key] = shlex.split(value)
    link = shlex.split((build / "CMakeFiles/motion_primitive_planner_core_test.dir/link.txt").read_text())
    for version in ("baseline", "current"):
        includes = flags["CXX_INCLUDES"][:]
        defines = flags["CXX_DEFINES"][:]
        if version == "baseline":
            includes = [x.replace(str(package / "include"), str(baseline / "source/motion_primitive_planner/include")) for x in includes]
            defines += ["-DMOTION_PRIMITIVE_BASELINE"]
        obj = output / (version + ".o")
        executable = output / (version + "_benchmark")
        subprocess.run([link[0], *defines, *includes, *flags["CXX_FLAGS"], "-c",
                        str(package / "test/planner_benchmark.cpp"), "-o", str(obj)], check=True, env=env)
        command = [str(obj) if x.endswith(".cpp.o") else x for x in link]
        command[command.index("-o") + 1] = str(executable)
        if version == "baseline":
            command = [str(baseline / Path(x).name) if x.endswith(".so") and (baseline / Path(x).name).exists() else x for x in command]
        subprocess.run(command, check=True, env=env)


def stats(values):
    ordered = sorted(values)
    return {"mean": statistics.mean(values), "median": statistics.median(values),
            "p95": ordered[math.ceil(len(ordered) * 0.95) - 1], "max": max(values)}


def differences(left, right):
    if left["details"] != right["details"]:
        return "details differ"
    a, b = left["values"], right["values"]
    if len(a) != len(b):
        return f"snapshot lengths {len(a)} != {len(b)}"
    for i, (x, y) in enumerate(zip(a, b)):
        if isinstance(x, (int, float)) and isinstance(y, (int, float)):
            if not math.isclose(x, y, rel_tol=1e-10, abs_tol=1e-10):
                return f"value[{i}]: {x} != {y}"
        elif x != y:
            return f"value[{i}]: {x} != {y}"
    return None


def summarize(output):
    records = []
    for path in sorted(output.glob("*_round*.json")):
        record = json.loads(path.read_text())
        record["run"] = path.stem
        records.append(record)
    groups = {}
    rows = []
    for record in records:
        key = (record["mode"], record["version"])
        groups.setdefault(key, []).extend(record["scenarios"])
        for s in record["scenarios"]:
            rows.append({"run": record["run"], "mode": record["mode"], "version": record["version"],
                         **{k: v for k, v in s.items() if k != "snapshot"}})
    if rows:
        with (output / "measurements.csv").open("w") as file:
            writer = csv.DictWriter(file, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    summary = {}
    for (mode, version), scenes in groups.items():
        baseline = groups.get((mode, "baseline"))
        entry = {"batches": len(scenes), "batch_ms": stats([s["batch_ms"] for s in scenes]),
                 "total_ms": stats([s["total_ms"] for s in scenes]),
                 "successful_batches": sum(s["selected"] >= 0 for s in scenes),
                 "successful_full_plans": sum(s["full_selected"] >= 0 for s in scenes),
                 "attempted": sum(s["attempted"] for s in scenes),
                 "feasible": sum(s["feasible"] for s in scenes),
                 "exhausted": sum(s["exhausted"] for s in scenes)}
        if baseline:
            for metric in ("batch_ms", "total_ms"):
                before = statistics.median(s[metric] for s in baseline)
                after = entry[metric]["median"]
                entry[metric]["speedup"] = before / after
                entry[metric]["reduction_percent"] = 100 * (1 - after / before)
        summary[f"{mode}/{version}"] = entry
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    scenario_rows = []
    for (mode, version), scenes in groups.items():
        for name in sorted({s["name"] for s in scenes}):
            samples = [s for s in scenes if s["name"] == name]
            row = {"mode": mode, "version": version, "scenario": name, "samples": len(samples)}
            for metric in ("batch_ms", "total_ms"):
                row.update({metric + "_" + k: v for k, v in stats([s[metric] for s in samples]).items()})
                before = [s[metric] for s in groups.get((mode, "baseline"), []) if s["name"] == name]
                if before:
                    row[metric + "_speedup"] = statistics.median(before) / row[metric + "_median"]
                    row[metric + "_reduction_percent"] = 100 * (1 - row[metric + "_median"] / statistics.median(before))
            row["successful_batches"] = sum(s["selected"] >= 0 for s in samples)
            scenario_rows.append(row)
    if scenario_rows:
        with (output / "per_scenario.csv").open("w") as file:
            writer = csv.DictWriter(file, fieldnames=list(scenario_rows[0]))
            writer.writeheader()
            writer.writerows(scenario_rows)
    reference = {}
    errors = []
    compared = 0
    exactly_equal = 0
    for record in sorted(records, key=lambda r: (r["version"] != "baseline", r["run"])):
        if record["mode"] != "ample":
            continue
        for scene in record["scenarios"]:
            if scene["name"] not in reference:
                reference[scene["name"]] = scene["snapshot"]
                continue
            compared += 1
            exactly_equal += reference[scene["name"]] == scene["snapshot"]
            error = differences(reference[scene["name"]], scene["snapshot"])
            if error:
                errors.append({"run": record["run"], "scene": scene["name"], "difference": error})
    verification = {"compared_batches": compared, "mismatches": errors,
                    "exactly_equal_batches": exactly_equal,
                    "relative_tolerance": 1e-10, "absolute_tolerance": 1e-10}
    (output / "equivalence.json").write_text(json.dumps(verification, indent=2) + "\n")
    print(f"Compared {compared} ample-budget batches; {len(errors)} mismatches", flush=True)
    return errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--rounds", type=int, default=5)
    parser.add_argument("--threads", type=int, nargs="+", default=[1, 2, 4, 8, 9])
    parser.add_argument("--modes", nargs="+", choices=["ample", "online", "tight"], default=["ample", "online"])
    parser.add_argument("--cpus", help="taskset CPU list; default: first nine distinct physical cores")
    parser.add_argument("--summarize-only", action="store_true")
    args = parser.parse_args()
    args.baseline = args.baseline.resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    if args.summarize_only:
        return bool(summarize(output))
    package = Path(__file__).resolve().parents[1]
    workspace = next(p for p in package.parents if (p / "devel/.catkin").exists())
    build = workspace / "build/motion_primitive_planner"
    env = os.environ.copy()
    cpus, physical = [], set()
    for cpu in sorted(os.sched_getaffinity(0)):
        topology = Path(f"/sys/devices/system/cpu/cpu{cpu}/topology")
        key = tuple((topology / name).read_text().strip() for name in ("physical_package_id", "core_id"))
        if key not in physical:
            cpus.append(cpu)
            physical.add(key)
    affinity = args.cpus or ",".join(str(c) for c in cpus[:9])
    compile_harness(package, build, args.baseline, output, env)
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
    env["ROS_MASTER_URI"] = f"http://127.0.0.1:{port}"
    env["ROS_HOSTNAME"] = "127.0.0.1"
    env["ROS_LOG_DIR"] = str(output / "ros_logs")
    master_log = (output / "roscore.log").open("w")
    master = subprocess.Popen(["roscore", "-p", str(port)], env=env, stdout=master_log, stderr=subprocess.STDOUT)
    try:
        proxy = xmlrpc.client.ServerProxy(env["ROS_MASTER_URI"])
        for _ in range(100):
            try:
                proxy.getPid("benchmark_runner")
                break
            except OSError:
                time.sleep(0.1)
        else:
            raise RuntimeError("isolated ROS master did not start")
        model = subprocess.check_output(["rospack", "find", "dragon"], env=env, text=True).strip()
        description = subprocess.check_output(["xacro", str(Path(model) / "robots/quad/v1_5_202601.urdf.xacro"), "robot_name:=dragon"], env=env, text=True)
        proxy.setParam("benchmark_runner", "/dragon/robot_description", description)
        subprocess.run(["rosparam", "load", str(args.baseline / "source/motion_primitive_planner/config/whole_body_motion_primitive_planner.yaml"), "/dragon/parallel_benchmark"], env=env, check=True)
        for name, value in {"MapBound": [-10.0, 10.0, -10.0, 10.0, 0.2, 8.0], "VoxelWidth": 0.1, "MaxVelMag": 1.0, "Verbose": False}.items():
            proxy.setParam("benchmark_runner", "/dragon/parallel_benchmark/" + name, value)
        metadata = {"affinity": affinity, "baseline": str(args.baseline), "rounds": args.rounds,
                    "threads": args.threads, "modes": args.modes,
                    "cpu": subprocess.check_output(["lscpu"], text=True),
                    "compiler": subprocess.check_output(["c++", "--version"], text=True),
                    "flags": (build / "CMakeFiles/motion_primitive_planner_core.dir/flags.make").read_text()}
        (output / "environment.json").write_text(json.dumps(metadata, indent=2) + "\n")
        versions = [("baseline", 1)] + [(f"threads_{n}", n) for n in args.threads]
        for mode in args.modes:
            for repeat in range(1, args.rounds + 1):
                order = versions if repeat % 2 else list(reversed(versions))
                for version, threads in order:
                    label = f"{mode}_{version}_round{repeat}"
                    result = output / (label + ".json")
                    if result.exists():
                        continue
                    run_env = env.copy()
                    executable = output / ("baseline_benchmark" if version == "baseline" else "current_benchmark")
                    if version == "baseline":
                        run_env["LD_LIBRARY_PATH"] = str(args.baseline) + ":" + env.get("LD_LIBRARY_PATH", "")
                    linked = subprocess.check_output(["ldd", str(executable)], env=run_env, text=True)
                    (output / (label + "_libraries.txt")).write_text(linked)
                    expected = args.baseline if version == "baseline" else workspace / "devel/.private/motion_primitive_planner/lib"
                    for library in ("libmotion_primitive_planner_core.so", "libmultilink_copilot_stability.so"):
                        actual = next(line.split(" => ", 1)[1].split(" (", 1)[0]
                                      for line in linked.splitlines() if line.strip().startswith(library + " => "))
                        expected_lib = expected / library
                        if version != "baseline" and "stability" in library:
                            expected_lib = workspace / "devel/.private/multilink_copilot/lib" / library
                        assert Path(actual).resolve() == expected_lib.resolve(), (actual, expected_lib)
                    with (output / (label + ".log")).open("w") as log:
                        subprocess.run(["taskset", "-c", affinity, str(executable), "__ns:=/dragon",
                                        "_output:=" + str(result), "_threads:=" + str(threads), "_mode:=" + mode],
                                       env=run_env, check=True, stdout=log, stderr=subprocess.STDOUT, timeout=600)
                    data = json.loads(result.read_text())
                    data["version"] = version
                    result.write_text(json.dumps(data, separators=(",", ":")) + "\n")
                    print(label, "median batch ms", round(statistics.median(s["batch_ms"] for s in data["scenarios"]), 3),
                          "successful", sum(s["selected"] >= 0 for s in data["scenarios"]), flush=True)
    finally:
        master.terminate()
        try:
            master.wait(timeout=15)
        except subprocess.TimeoutExpired:
            master.kill()
            master.wait()
        master_log.close()
    return bool(summarize(output))


if __name__ == "__main__":
    raise SystemExit(main())
