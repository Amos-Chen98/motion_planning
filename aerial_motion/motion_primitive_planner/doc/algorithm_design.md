# Motion Primitive Planner Algorithm Design

## Architecture

The package provides two online planners built on the same root-motion and DRAGON feasibility model:

| Aspect | Root-link planner | Whole-body planner |
| --- | --- | --- |
| Output | GCOPTER polynomial trajectory executed by `traj_server` | Synchronized root and joint commands published as `FullStateTarget` |
| Joint treatment | Predicts nominal follow-the-leader motion for candidate validation | Plans and executes a continuous joint trajectory |
| Swept collision check | Uses the nominal predicted body shape | Uses the final time-scaled root and joint trajectories |
| Planning failure | Keeps the active root trajectory | Enters a validated hover hold or keeps the current safe plan |

Both modes share map handling, target clamping, route search, local-horizon selection, MINCO primitive generation, DRAGON model metadata, trajectory history, candidate diagnostics, and the instantaneous whole-body collision checker.

## Shared Planning Pipeline

1. The planner updates an accumulated or replaceable voxel map and obtains an immutable occupancy snapshot for each candidate evaluation.
2. The global target is clamped to the valid map region. A route search produces a collision-free root route, which is truncated at `PlanningHorizon` to obtain the local target.
3. `PrimitiveGenerator` creates a direct MINCO primitive and configurable midpoint-offset alternatives. The route's intermediate search points select the local target but do not constrain the polynomial.
4. Candidates are checked for whole-body swept collision and DRAGON flight feasibility.
5. The best feasible candidate is activated or published; if none is feasible, the mode-specific failure policy is applied.

### Root Primitive Boundary Conditions

Every primitive preserves the planner-provided root position, velocity, and acceleration at handover. The local target always has zero acceleration. `ZeroLocalTargetVel` selects between a zero terminal velocity and a tangent-aligned cruise velocity for non-terminal local targets; a global terminal target has zero velocity. Candidate durations are enlarged when necessary to satisfy the configured root-velocity limit.

### Collision and Flight-Feasibility Checks

For an instantaneous DRAGON configuration, the shared collision checker reconstructs every link centerline and samples it against the dilated binary occupancy map at a spacing derived from the voxel resolution. Swept checks adaptively subdivide each time interval from a conservative bound on root translation, body yaw, and joint motion. The root-link planner evaluates the nominal follow-the-leader body, while the whole-body planner evaluates the final planned body.

Flight-feasibility evaluation covers joint limits, static-thrust bounds, roll/pitch feasible-control margin (`fc_rp_min`), rotor clearance, and baselink tilt. In root-link mode, `PredictionDt` controls nominal stability evaluation only; collision sampling is governed independently by command intervals and the adaptive displacement bound.

## Root-Link Planner

The root-link planner samples the active polynomial's position, velocity, and acceleration at handover when available; otherwise it uses the latest odometry position with zero initial derivatives. It predicts the nominal joint sequence from root history, rejects candidates with collision or flight-feasibility violations, and ranks feasible candidates by root-path length and then jerk.

When `AllowCopilotStabilityProjectionFallback` is enabled and no nominally feasible candidate exists, a candidate whose only violation is insufficient `fc_rp_min` may be delegated to downstream Copilot projection. Fallback candidates are ranked by path length, nominal joint motion, stability margin, and jerk. The projected body can differ from the collision-checked nominal body, so this fallback is appropriate only when the environment is sufficiently open or the projected shape is checked elsewhere.

The selected polynomial is handed to `traj_server`. A failed planning attempt does not replace the active trajectory.

## Whole-Body Planner

### Planning and Activation

Planning runs on a dedicated worker so expensive candidate evaluation cannot block command publication. A request predicts the root and joint state at the configured activation time by sampling the current active plan and extending its trajectory history. Concurrent requests are coalesced, and target sequence checks prevent a result for an obsolete goal from being activated.

For each root candidate, the planner constructs a continuous joint trajectory, time-scales the synchronized root and joint motion, and rechecks the final swept body against the occupancy snapshot. It also samples the executed shape at the command rate and measures each downstream link tail against the prefix of the root-tail trace available at that instant. The resulting RMS error captures sustained body expansion, while the maximum error records the transient tracking envelope.

Feasible candidates are ranked by `duration + JointMotionCostWeight * joint_motion + TrackingErrorCostWeight * tracking_error_rms`, with tracking RMS, joint motion, duration, and root jerk used as deterministic tie-breakers. The default weights of 0.25 s/rad and 6.0 s/m discourage both unnecessary joint detours and root candidates that make the downstream body sweep far from the first-link trace. `TrackingErrorCostWeight=0` restores duration-and-joint-only ranking. The selected plan remains pending until its activation time, then becomes the active command source.

### Joint-Trajectory Construction and Repair

The nominal follow-the-leader sequence supplies the terminal joint reference, the root-yaw schedule, and the tracking-error reference; its intermediate joint samples do not constrain the planned joint path. If the measured start configuration is infeasible, the shared stability QP projects it to a nearby feasible configuration. The nominal sequence is then regenerated from that repaired start. If its terminal configuration is infeasible, the same QP projects the terminal reference into the feasible set using the repaired start as the stable reference. Failure to obtain either feasible endpoint rejects the candidate.

OMPL RRT-Connect performs one global search from the repaired start to the repaired follow-the-leader terminal configuration. Its seeded sampler is uniform over the configured joint limits and has no morphology-specific bias. This treats all low-control-authority regions uniformly and lets the configuration-space search find any feasible homotopy instead of prescribing local repair intervals or shape-specific detours.

The returned path is greedily shortened and then mapped across the complete root-trajectory horizon according to cumulative joint-space arc length. Every edge is revalidated at `JointValidityResolution` under the actual time-varying root-yaw schedule and the full flight-feasibility constraints. An RRT timeout, search failure, shortcut validation failure, or final timed-path validation failure rejects the complete root candidate.

If start repair was required, the published trajectory still begins at the measured joint state for command continuity. The measured-to-repaired transition is prepended to the RRT path and participates in arc-length timing and final whole-body swept-collision checking; strict flight-feasibility guarantees begin at the repaired start because the measured endpoint is already known to be infeasible. The completed joint path uniformly time-scales the synchronized root, yaw, and joint trajectories to satisfy joint-velocity and command-step limits.

When a root path retraces recent history, the exact Euclidean sphere intersections used by follow-the-leader can move between overlapping history branches even though the root motion is smooth. A single global configuration-space search can bypass the resulting low-control-authority region without depending on where intermediate nominal samples become infeasible, while the trace-tracking cost continues to favor root primitives whose executed body remains compact through the reversal.

The whole-body planner exclusively owns `full_state_target` and must not run with another full-state producer such as `multilink_copilot`. The root `traj_server` pipeline should also remain inactive in whole-body mode because it creates a competing root-command path.

## Replanning and Execution Lifecycle

A new target invalidates any pending plan and requests planning immediately. Every successfully published or activated non-terminal trajectory arms one trigger at `ReplanTriggerRatio` of its execution duration; each trajectory can trigger only once.

| Mode | Progress clock | Retry events after failure |
| --- | --- | --- |
| Root-link | Executed-command timestamp relative to the published polynomial start time | Point cloud, valid joint state, or executed command |
| Whole-body | Current ROS time relative to the active time-scaled trajectory's activation time | Point cloud, odometry, valid joint state, or command-publication cycle |

There is no periodic replanning timer. A plan is terminal when route generation reaches the global target or its endpoint lies within `GoalTolerance`; terminal plans complete without arming another progress trigger. Whole-body replacement plans retain their configured activation lead time and become active only at the predicted handover instant.

After a failed attempt, retries remain event driven. The root-link planner keeps its previous trajectory. The whole-body planner uses the last validated command as a zero-velocity hover hold for generation or evaluation failures; a plan that misses its activation deadline is discarded while the existing safe state remains active.

## Safety Boundaries

- Root-link mode collision-checks the nominal follow-the-leader shape, not a shape later modified by Copilot. Enabling stability projection therefore weakens the coupling between the checked body and the commanded body.
- Keeping an old trajectory or hover hold after planning failure is a continuity fallback, not a dynamic-obstacle avoidance guarantee. Fast environmental changes require an independent emergency or tracking-safety layer.
- Whole-body mode checks its final planned articulated motion, but correctness still depends on the occupancy map, state estimates, model calibration, and command tracking.

## Diagnostics

`/candidate_markers` publishes all root-link candidates as a `visualization_msgs/MarkerArray`. Colors indicate the evaluation result:

- **Green:** selected feasible candidate.
- **Cyan:** feasible but not selected.
- **Orange:** requires stability projection, fails a stability check, or fails joint planning.
- **Magenta:** rejected because the nominal joint configuration violates a joint limit.
- **Red:** rejected because of predicted whole-body collision, trajectory-generation failure, or another non-feasible status.

Both nodes publish `/selected_candidate`, `/selected_min_fc_rp`, and `/selected_joint_motion`. The whole-body planner's selected-candidate log additionally reports the downstream-link tracking RMS and maximum in metres. Collision results are binary and no clearance diagnostic is published.
