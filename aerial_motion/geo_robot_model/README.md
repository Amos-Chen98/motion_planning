# geo_robot_model

`geo_robot_model` provides lightweight, Gazebo-free geometric robot models and simulated LiDAR sensing for aerial motion-planning tests. The models update their state directly from command messages, so they are intended for planner integration and visualization rather than flight-dynamics evaluation.

## Components

- `point_robot_model.py`: ideal point-robot model that follows `aerial_robot_msgs/FlightNav` commands and publishes odometry and TF.
- `geo_dragon_model.py`: ideal four-link DRAGON model that publishes its geometric state, TF tree, and visualization from root and joint commands.
- `livox_mid360_simulator`: converts a global point-cloud map into a local Livox Mid-360 point cloud using robot odometry and the robot-provided LiDAR TF.
- `pose_to_flight_nav.py`: converts `geometry_msgs/PoseStamped` commands to `aerial_robot_msgs/FlightNav`.

## Build

```bash
catkin build geo_robot_model
source devel/setup.bash
```

## Run

Start the point-robot model:

```bash
roslaunch geo_robot_model point_robot_bringup.launch
```

Start the geometric DRAGON model:

```bash
roslaunch geo_robot_model geo_dragon_bringup.launch
```

Pass `lidar:=true` to either bringup launch when a global point-cloud map is available and local simulated LiDAR sensing is required.
