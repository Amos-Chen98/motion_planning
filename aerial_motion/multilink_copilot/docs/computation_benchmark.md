# Four-to-eight-link computation benchmark

Run the complete Copilot, including feasibility projection, on 4–8-link models with ideal tracking. The default batch runs 74 scenarios with all five link counts, for 370 trials. Gazebo is not required.

## Build and environment

Run the commands below from the workspace. Source the workspace in each new terminal.

```bash
cd /home/chen/projects/dragon_dev/motion_planning_ws
source devel/setup.bash
catkin build mono_planner multilink_copilot data_manager --no-deps -j4 -p1 --no-status
source devel/setup.bash
```

## Check before running

List the planned combinations without starting simulations:

```bash
rosrun multilink_copilot run_computation_benchmark_batch.py --dry-run
```

Run a short check using the first 2 seconds of the first scenario with all five link counts. Results go to `/tmp/copilot_benchmark_tests/` by default.

```bash
rosrun multilink_copilot run_computation_benchmark_batch.py --smoke
```

## Run simulations

Choose one of the following commands:

```bash
# One scenario with eight links.
rosrun multilink_copilot run_computation_benchmark_batch.py --case 1 --links 8

# One scenario with selected link counts.
rosrun multilink_copilot run_computation_benchmark_batch.py --case 1 --links 4 6 8

# All 74 scenarios with all link counts from four through eight.
rosrun multilink_copilot run_computation_benchmark_batch.py
```

`--case` selects a scenario; omitting it selects all scenarios. `--links` accepts one or more distinct integers from 4 to 8; omitting it selects all five counts. Use `--dry-run` with either option to check the selection.

Trials run serially without real-time playback delays. The script starts and stops its own ROS master. Ctrl+C stops the batch and saves available results. Automatic resume is not supported; restarting creates a new run.

## Watch in RViz

Visualization requires one scenario and one link count:

```bash
rosrun multilink_copilot run_computation_benchmark_batch.py --case 1 --links 8 --visualize
```

RViz displays the robot, waypoints, reference trajectory, and update status. Playback defaults to real time, and the window stays open after completion. Press Ctrl+C to close it and finish the summary analysis.

Optional display settings are `--playback-speed 0.5` for half-speed playback, `--start-delay 5` for a five-second initial pause, and `--no-keep-open` to close automatically after completion. Use runs without RViz for timing comparisons.

## Results and offline analysis

Results are saved under `motion_planning_ws/src/data_manager/dragon_copilot/data/computation_benchmark/`. The script prints the actual result directory and automatically produces per-trial timing statistics, including median, P90, P95, and comparison plots. To choose a destination, add `--output-dir /absolute/path/to/new_directory`; the directory must not already exist.

To analyze saved results again, edit `INPUT_DIR` and `OUTPUT_DIR` at the top of `data_manager/dragon_copilot/data_analysis/computation_benchmark/analyze_computation_benchmark.py`, then run from `motion_planning_ws`:

```bash
python3 src/data_manager/dragon_copilot/data_analysis/computation_benchmark/analyze_computation_benchmark.py
```

Timing covers the complete Copilot update and excludes initialization, visualization, waiting, and file output. Further analysis guidance and result interpretation are in the [analysis documentation](../../../../data_manager/dragon_copilot/data_analysis/computation_benchmark/README.md).
