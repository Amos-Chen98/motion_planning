# Whole-Body Motion Primitive Planner Algorithm Design

## Architecture

The package provides an online whole-body planner for DRAGON. It generates root-motion primitives, plans continuous joint trajectories, checks the final synchronized body motion, and publishes `FullStateTarget` directly. Planning failures enter a hover hold when a suitable command state is available or preserve the current plan when a replacement misses its activation deadline.

The planner uses GCOPTER map/path and MINCO trajectory tools, DRAGON model metadata, and the follow-the-leader and stability libraries from `multilink_copilot`. Configuration is loaded from [whole_body_motion_primitive_planner.yaml](../config/whole_body_motion_primitive_planner.yaml), followed by the launch-argument overrides. Defaults include `ReplanTriggerRatio=0.3`, `PlanActivationLeadTime=0.75`, and `CommandHz=40.0`.

### Implementation Boundaries

The [source organization](../README.md#source-organization) uses six C++ files. `whole_body_planner_node.cpp` owns ROS IO, model lifetime, the worker, and trajectory execution. It obtains a root-candidate batch and the corresponding occupancy snapshot under its map lock, using `root_primitive_generator.cpp`, then calls `WholeBodyPlanner::plan()` from `whole_body_planner.cpp` outside that lock. The batch planner owns one joint planner per candidate and returns candidate results and the selected index without publishing commands or modifying active/pending plans. `joint_trajectory_planner.cpp` contains history, nominal prediction, and joint planning; `dragon_geometry.cpp` contains frame transforms and instantaneous collision geometry; `planner_config.cpp` contains parameter loading and validation. The shared candidate deadline follows ROS activation time, while joint-search timeout accounting continues to use `steady_clock`.

## Planning Pipeline

1. `voxel_mapping` accumulates observations or replaces them with each point cloud. The planner replaces its collision map with the received voxel snapshot and uses an immutable occupancy snapshot for the planning batch.
2. The global target is clamped to the valid map region. A route search produces a collision-free root route, which is truncated at `PlanningHorizon` to obtain the local target.
3. `PrimitiveGenerator` creates a direct MINCO primitive and configurable midpoint-offset alternatives. The route's intermediate search points select the local target but do not constrain the polynomial.
4. Candidates are checked for sampled whole-body collision and DRAGON flight feasibility.
5. The best feasible candidate is scheduled for activation; if none is feasible, the planner attempts to enter a hover hold and retries on subsequent input or command-publication events.

### Root Primitive Boundary Conditions

Every primitive preserves the planner-provided root position, velocity, and acceleration at handover. The local target always has zero acceleration. `ZeroLocalTargetVel` selects between a zero terminal velocity and a tangent-aligned cruise velocity for non-terminal local targets; a global terminal target has zero velocity. Candidate durations are enlarged when necessary to satisfy the configured root-velocity limit.

### Collision and Flight-Feasibility Checks

For an instantaneous DRAGON configuration, the collision checker reconstructs every link centerline and samples it against the dilated binary occupancy map at a spacing derived from the voxel resolution. Each trajectory of duration `T` is checked on a fixed `CommandHz` time grid with `N = ceil(T * CommandHz)` equal intervals, including both endpoints and without adding joint or yaw waypoints. The collision check evaluates the final time-scaled root and joint trajectories.

Flight-feasibility evaluation covers joint limits, static-thrust bounds, roll/pitch feasible-control margin (`fc_rp_min`), rotor clearance, and baselink tilt. `CommandHz` controls collision sampling, command publication, and nominal follow-the-leader prediction.

## Whole-Body Planner

### Planning and Activation

Planning runs on a dedicated worker so expensive candidate evaluation cannot block command publication. A request predicts the root and joint state at the configured activation time by sampling the current active plan and extending its trajectory history. Concurrent requests are coalesced, and target sequence checks prevent a result for an obsolete goal from being activated.

For each root candidate, the planner constructs a continuous joint trajectory, time-scales the synchronized root and joint motion, and checks the final body against the occupancy snapshot on the fixed `CommandHz` time grid. It also samples the executed shape at the command rate and measures each downstream link tail against the prefix of the root-tail trace available at that instant. The resulting RMS error captures sustained body expansion, while the maximum error records the transient tracking envelope.

Feasible candidates are ranked by `duration + JointMotionCostWeight * joint_motion + TrackingErrorCostWeight * tracking_error_rms`, with tracking RMS, joint motion, duration, and root jerk used as deterministic tie-breakers. The default weights of 0.25 s/rad and 6.0 s/m discourage both unnecessary joint detours and root candidates that make the downstream body sweep far from the first-link trace. `TrackingErrorCostWeight=0` restores duration-and-joint-only ranking. The selected plan remains pending until its activation time, then becomes the active command source.

### Root Attitude and Joint1-Priority Allocation

The planner derives FLU yaw and pitch from the three-dimensional root-velocity tangent, with roll fixed to zero. Near-zero velocity retains the current attitude, and a purely vertical tangent retains yaw. The goal quaternion does not specify the planned attitude.

Each plan begins with synchronized root yaw/pitch and joint1 pitch/yaw changes, keeping the other joints fixed during this allocation interval. Root pitch is compensated by same-sign joint1 pitch, and root yaw by opposite-sign joint1 yaw, subject to joint limits. The allocation prefix must satisfy the flight-feasibility constraints before RRT runs, and the RRT cannot rewrite it.

For a stationary start, root translation is delayed until the allocation completes. For a moving handover, allocation overlaps translation and the new primitive preserves root position, velocity, and acceleration; candidates requiring global time scaling are rejected to preserve those boundary states. Root yaw, root pitch, and all inter-link joints share `MaxAngularVel`, whose launch default is `1.0 rad/s`; the joint command-step bound remains independently enforced.

`FullStateTarget.root_state` describes the root-link origin with its full quaternion and linear/angular velocity. `root/target_pose` describes the link1 tail in FLU. The origin position and velocity are converted from the tail state using the full attitude, angular velocity, and link1 lever arm.

### Joint-Trajectory Construction and Repair

The nominal follow-the-leader sequence supplies the terminal joint reference, the root-attitude schedule, and the tracking-error reference; its intermediate joint samples do not constrain the planned joint path. If the measured start configuration is infeasible, the shared stability QP projects it to a nearby feasible configuration before joint1 allocation. The nominal sequence is generated from the allocation endpoint and updated root trace. If its terminal configuration is infeasible, the same QP projects the terminal reference into the feasible set using the joint1-allocation endpoint as the stable reference. Failure to obtain either feasible endpoint rejects the candidate.

OMPL RRT-Connect performs one global search from the joint1-allocation endpoint to the repaired follow-the-leader terminal configuration. Its seeded sampler is uniform over the configured joint limits and has no morphology-specific bias. This treats all low-control-authority regions uniformly and lets the configuration-space search find any feasible homotopy instead of prescribing local repair intervals or shape-specific detours.

The returned path is greedily shortened and then mapped across the root-trajectory horizon after the joint1-allocation prefix according to cumulative joint-space arc length. Every edge is revalidated at `JointValidityResolution` under the actual time-varying root-attitude schedule and the full flight-feasibility constraints. An RRT timeout, search failure, shortcut validation failure, or final timed-path validation failure rejects the complete root candidate.

If start repair was required, the published trajectory still begins at the measured joint state for command continuity. The measured-to-repaired transition precedes joint1 allocation and participates in final whole-body fixed-rate collision sampling; strict flight-feasibility guarantees begin at the repaired start because the measured endpoint is already known to be infeasible. For stationary starts, the completed path uniformly time-scales the synchronized root translation, attitude, and all joint trajectories to satisfy the root-attitude and joint angular-rate limit `MaxAngularVel`; `MaxJointCommandStep * CommandHz` remains an independent discrete command-rate bound, and the stricter bound is applied.

When a root path retraces recent history, the exact Euclidean sphere intersections used by follow-the-leader can move between overlapping history branches even though the root motion is smooth. A single global configuration-space search can bypass the resulting low-control-authority region without depending on where intermediate nominal samples become infeasible, while the trace-tracking cost continues to favor root primitives whose executed body remains compact through the reversal.

The whole-body planner exclusively owns `full_state_target` and must not run with another full-state producer such as `multilink_copilot`. The root `traj_server` pipeline should also remain inactive because it creates a competing root-command path.

## Replanning and Execution Lifecycle

A new target invalidates any pending plan and requests planning immediately. Every activated non-terminal trajectory arms one trigger at `ReplanTriggerRatio` of its execution duration; each trajectory can trigger only once.

Progress is measured using current ROS time relative to the active time-scaled trajectory's activation time. After failure, point clouds, odometry, valid joint states, and command-publication cycles can request a retry.

There is no periodic replanning timer. A plan is terminal when route generation reaches the global target or its endpoint lies within `GoalTolerance`; terminal plans complete without arming another progress trigger. Whole-body replacement plans retain their configured activation lead time and become active only at the predicted handover instant.

After a failed attempt, retries remain event driven. The whole-body planner uses the last validated command as a zero-velocity hover hold for generation or evaluation failures; a plan that misses its activation deadline is discarded while the existing safe state remains active.

## Safety Boundaries

- Fixed `CommandHz` collision sampling is not continuous collision detection and can miss a short collision that occurs only between adjacent snapshots, especially during fast joint or yaw motion.
- Keeping an old trajectory or hover hold after planning failure is a continuity fallback, not a dynamic-obstacle avoidance guarantee. Fast environmental changes require an independent emergency or tracking-safety layer.
- The planner checks its final planned articulated motion, but correctness still depends on the occupancy map, state estimates, model calibration, and command tracking.

## Diagnostics

`candidate_markers` publishes all root-motion candidates as a `visualization_msgs/MarkerArray` in the robot namespace (`/dragon` by default). Colors indicate the evaluation result:

- **Green:** selected feasible candidate.
- **Cyan:** feasible but not selected.
- **Orange:** joint planning failed, including an infeasible configuration or failed endpoint repair.
- **Red:** rejected because of predicted whole-body collision, trajectory-generation failure, or another non-feasible status.

The planner publishes `selected_candidate`, `selected_min_fc_rp`, and `selected_joint_motion` in the same robot namespace. The whole-body planner's selected-candidate log additionally reports the downstream-link tracking RMS and maximum in metres. Collision results are binary and no clearance diagnostic is published.
