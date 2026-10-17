# Projection Robustness Evaluation

Each case runs full DRAGON dynamics for 50 seconds at 40 Hz (2001 references), followed by 5 seconds of observation. Cases run serially with GUI windows and restart all simulation components. Source the workspace in each terminal.

~~~bash
cd /home/chen/projects/dragon_dev/motion_planning_ws
source devel/setup.bash
catkin build multilink_copilot data_manager --no-deps -j4 -p1
source devel/setup.bash
~~~

Both automatic and manual trials publish every waypoint pose/ring and the complete root-link tail reference before initialization, keeping them visible throughout observation. Reference colors follow trajectory time from blue to red, using the same palette as the waypoint demo. The runner supplies these displays automatically; close any separately launched waypoint publishers before starting a trial.

**Run experiments.** Paths are constants in `run_projection_robustness_batch.py`, resolved through `rospkg`; no exports or path arguments are needed. Results go under `data_manager/dragon_copilot/data/rosbag/projection_robustness/`.

~~~bash
# All 100 cases → batch_100/
rosrun multilink_copilot run_projection_robustness_batch.py
# Resume the same batch
rosrun multilink_copilot run_projection_robustness_batch.py --resume
# Selected cases → batch_100_single/
rosrun multilink_copilot run_projection_robustness_batch.py --cases 1
~~~

Ctrl+C interrupts and cleans up the processes. Resume with unchanged code, configuration, and case selection; completed/failed cases are skipped, and interrupted cases are archived before retrying. Edit the output constants for a separate batch.

**Manual simulation and recording.** Use three terminals. In B, press `r` to arm and `t` to take off. C initializes the robot, starts copilot, plays references, and records to `batch_100_manual/`. With `--attach`, C also streams the planner's stdout/stderr as it runs, starting when copilot launches after initialization.

By default, each manual run replaces the selected case's existing directory (including its bag and trial metadata) and refreshes the batch manifest and summary. Other case directories are kept. No overwrite flag is required, and changed code/configuration or case selection is accepted. Add `--resume` explicitly to use the usual resume checks and skip completed/failed cases instead.

~~~bash
# Terminal A
roslaunch multilink_copilot projection_robustness_sim.launch
# Terminal B
rosrun aerial_robot_base keyboard_command.py
# Terminal C
rosrun multilink_copilot run_projection_robustness_batch.py --cases 1 --attach
~~~

For skip-command-publication events, watch Terminal C for `[CopilotPlanner] Failed to find a stable joint target; skipping this control cycle` or `[CopilotPlanner] Skipping full_state_target: ...` (joint-step or predicted-tilt limit). These ERROR/WARN messages appear with the default `verbose:=false`; repeated messages are throttled to once per 1.0/0.5 seconds, respectively, so console line counts are not skip counts. The first due root update rejected by the planner is recorded on `/dragon/copilot/first_skip_stamp` and in `trial.json` as `first_skip_stamp`.

**Offline analysis.** No arguments or ROS master are required. The script's `INPUT_DIR` selects `validation_20261016`; change it to the desired batch directory when analyzing other runs.

~~~bash
python3 /home/chen/projects/dragon_dev/motion_planning_ws/src/data_manager/dragon_copilot/data_analysis/projection_robustness/analyze_projection_robustness.py
~~~

A **skip** is a due root update rejected by feasibility, continuity, or final safety checks; threshold filtering and successful fallback do not count. **Instability** means an abnormal exit from HOVER during playback or observation. Completion requires all references and the full observation period.

| Outcome | Reported metrics |
|---|---|
| Completed without skips or instability | Count, position RMSE, P95 of experiment RMSEs |
| Completed with skips, without instability | Count, position RMSE, P95 of experiment RMSEs |
| Skips followed by instability, including early termination | Count |
| Startup/process failure, interruption, missing data, or instability without skips | Separate count and reasons |

Position error is the Euclidean distance between measured and reference root origins, after LINK/FLU and tail-to-root conversion. Each experiment RMSE uses valid 40-Hz samples from playback only. Group RMSE uses pooled samples; P95 is the 95th percentile of the experiment RMSEs within that group, with each experiment weighted equally. Interpolation cannot bridge gaps over 0.1 seconds; stable experiments require at least 98% coverage. Empty groups report `null`; missing samples are not zero-filled. Unrun cases and nested archives are excluded.

Each case retains `trial.bag`, `trial.json`, `case.yaml`, and `parameters.yaml`; infrastructure failures also retain up to 100 lines in `error.log`. Batch provenance stays at the batch root. Reports are `analysis/experiments.csv`, `analysis/summary.json`, and `analysis/analysis_manifest.json`.
