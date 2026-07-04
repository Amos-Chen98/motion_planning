# Root Trajectory Stability Metrics Workflow

This workflow runs the Dragon simulation with the multilink copilot, records the
stability metrics generated during root trajectory tracking, and converts the
recorded rosbag into stability metric plots.

## 1. Prepare the Workspace

From the workspace root:

```bash
source devel/setup.bash
```

Use separate terminals for the bringup, copilot planner, trajectory publisher,
and data analysis steps. Source the workspace in each terminal before running
ROS commands.

## 2. Launch Dragon and Take Off

Start Dragon in simulation:

```bash
roslaunch dragon bringup.launch rm:=false sim:=true headless:=false
```

Use the standard Dragon bringup teleoperation flow to arm the robot, take off,
and enter hover. Start the root trajectory test only after the robot is
hovering stably.

## 3. Start the Copilot Planner

Launch the copilot planner with stability metric publishing enabled:

```bash
roslaunch multilink_copilot copilot_planner.launch publish_stability_metrics:=true
```

The root trajectory launch files record the stability metric topics, so this
argument must be enabled before starting the trajectory publisher.

## 4. Run a Root Trajectory Test and Record a Rosbag

Run one of the root trajectory publishers with rosbag recording enabled.

For the straight trajectory test:

```bash
roslaunch multilink_copilot root_straight_trajectory.launch record_rosbag:=true
```

For the circular trajectory test:

```bash
roslaunch multilink_copilot root_circular_trajectory.launch record_rosbag:=true
```

Each launch records the stability metric topics while the trajectory publisher
runs. When the trajectory launch exits, the generated bag is saved under:

```bash
data_manager/dragon_copilot/data/rosbag/stability_metrics/
```

## 5. Generate Stability Metric Plots

Open:

```bash
data_manager/dragon_copilot/data_analysis/plot_stability_metrics.py
```

Set `BAG_PATH` to the rosbag generated in the previous step. Then run from the
`data_manager` package root:

```bash
roscd data_manager
python3 dragon_copilot/data_analysis/plot_stability_metrics.py
```

The script reads the recorded stability metric topics and writes the generated
PDF figures to:

```bash
data_manager/dragon_copilot/data/figures/stability_metrics/
```
