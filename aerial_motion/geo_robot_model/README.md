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

The geometric DRAGON bringup publishes `/dragon/robot_description` from the canonical `dragon/robots/quad/v1_5_202601.urdf.xacro` model. This description supplies the kinematics, inertia, thrust, gimbal, and EDF data required by `multilink_copilot`; state updates and visualization still use the lightweight Gazebo-free geometric model, and no `robot_state_publisher` is started.

To debug the Copilot planner, start the geometric model and planner in separate terminals. Wait until the first launch reports that the DRAGON geometric model is ready before starting Copilot, because the planner reads `robot_description` during initialization.

Override `robot_model` when a different compatible DRAGON xacro is required. When changing `robot_ns`, pass the same value to both launches so that the generated `/<robot_ns>/robot_description`, joint-state input, and full-state target output remain in the same namespace.

Pass `lidar:=true` to either bringup launch when a global point-cloud map is available and local simulated LiDAR sensing is required.
