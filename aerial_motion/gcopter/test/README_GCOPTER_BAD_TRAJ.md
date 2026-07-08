# Reproducing the GCOPTER Out-of-Bound Corridor / Trajectory Anomaly

This document explains how to reproduce and interpret the planning anomaly observed during the 2026-06-23 Naraha quadrotor flight:

1. The safe flight corridor drawn in RViz extended beyond the configured `map_bound`.
2. Some published trajectories left the planning volume by a huge margin (the worst one reached x = -161.7 m while the map spanned x, y = ±8 m).

Evidence bag:

```
src/data_manager/dragon_nav/data/rosbag/20260622_naraha/2026-06-23-15-11-32_quad_bad_traj.bag
```

## Root cause

Both symptoms share one root cause: **the polytope vertex enumeration `geo_utils::enumerateVs()`** (`gcopter/include/gcopter/geo_utils.hpp`) **silently returns spurious vertices far outside the polytope** when the half-space set contains nearly parallel or duplicated planes.

The safe flight corridor itself is stored as an H-representation:

```
h0 * x + h1 * y + h2 * z + h3 <= 0
```

Those half-spaces are the authoritative corridor constraints. In the bad Naraha flight, they remained clipped to the local map bounds. The failure was introduced only when GCOPTER converted the H-representation into a V-representation for downstream consumers.

The original `enumerateVs()` implementation used a polar-dual construction:

1. Find an interior point `inner`.
2. For each plane, compute its signed margin to `inner`.
3. Divide the plane normal by that margin to create a dual point.
4. Run QuickHull on the dual points.
5. Convert each hull triangle back into a primal vertex using `normal / normal.dot(point)`.

This construction is fast for well-conditioned planes, but it is fragile for the corridor planes produced here. When two planes are duplicated or nearly parallel, their dual points become duplicated or nearly coincident. QuickHull can then produce a triangle whose area is extremely small. The triangle normal is close to zero, and `normal.dot(point)` can also be close to zero. The final division amplifies round-off error into a huge coordinate, sometimes reaching `1e17`, `1e33`, or infinity.

The critical missing check was that the recovered vertices were not substituted back into the original H-representation. As a result, `enumerateVs()` could report success while returning vertices that violated the same half-space constraints by meters to astronomical distances.

## Why the Naraha configuration triggers it

Degenerate inputs are common in this experiment, not exceptional:

- The planner truncates every route to the 3 m planning horizon. This is much shorter than `convexCover()`'s 7 m segment progress, so each replan usually produces only one or two polytopes.
- With one polytope, `sfc_gen::shortCut()` duplicates it. Then `GCOPTER_PolytopeSFC::processCorridor()` stacks consecutive polytopes to enumerate their overlap, producing a plane set that contains exact duplicates.
- With two polytopes, the overlap is thin and many stacked planes are nearly parallel.
- `VoxelMap::getSurf()` returns voxel-grid surface points on a regular lattice. FIRI frequently turns those regular points into coplanar or nearly coplanar tangent planes.

In short, the route horizon, shortcut behavior, overlap construction, and regular voxel surface all push the vertex enumerator into the numerical corner case that the original dual QuickHull path did not validate.

## Failure propagation

Once a bad vertex appears, the rest of the system trusts it:

- **Optimizer path.** The MINCO inner waypoint is parameterized as a convex combination of the enumerated overlap vertices (`GCOPTER_PolytopeSFC::forwardP`). One garbage vertex therefore lets the optimizer place the waypoint hundreds of meters outside the map. This is provable from the bag alone: the recorded piece junctions (the waypoint) sit at, e.g., (-161.7, -0.6, 4.5) with near-zero velocity, while the corridor H-representation is mathematically confined to the map bound (`convexCover()` clips every bounding box to the voxel-map corners).
- **Publication path.** `PlannerBackend::enforceVelocityLimit()` only rescales time when the optimized trajectory exceeds the velocity limit (scale factors of 5-24x appear in the bag's `/rosout`); it never checks position feasibility, so the wild geometry is published and tracked.
- **Visualization path.** `Visualizer::visualizePolytope()` builds the RViz mesh from the same vertex enumeration, one polytope at a time. Spurious vertices make the corridor mesh appear to extend beyond `map_bound` even though the underlying H-polytope never does.

This explains why two apparently different symptoms share the same origin: the optimizer and RViz are both consuming the same corrupted V-representation.

## Fix principle

The fix changes the vertex enumeration from an unvalidated dual-hull recovery to direct, constraint-checked H-polytope vertex enumeration.

For a 3D H-polytope, every true vertex lies at the intersection of at least three active planes. The repaired implementation therefore:

1. Normalizes each non-degenerate half-space row so tolerances are meaningful.
2. Rejects impossible zero-normal constraints and skips harmless redundant zero-normal rows.
3. Enumerates every triple of planes.
4. Skips triples with a near-zero determinant, which are parallel, duplicate, or otherwise too ill-conditioned to define a stable point.
5. Solves the remaining 3 x 3 linear systems to get candidate vertices.
6. Rejects non-finite candidates.
7. Substitutes every candidate back into all original half-space constraints. Only candidates whose maximum violation is within tolerance are accepted.
8. Deduplicates accepted vertices with `filterVs()`.

This makes the H-representation the source of truth. A numerically unstable plane triple can no longer create a far-away vertex because the candidate must still satisfy every corridor half-space before it is returned to the optimizer or visualizer.

The cost is `O(m^3)` in the number of planes `m`, but GCOPTER corridors in this pipeline have small plane counts, typically around tens of rows. The extra work is negligible compared with planning and optimization, while the resulting vertices are directly validated against the constraints they represent.

The fix is implemented in `geo_utils::enumerateVsByTriples()` and is used by the existing `enumerateVs()` public overloads, so existing callers keep the same API. `filterVs()` was also made robust to empty input and zero-scale deduplication cases.

## Reproduction 1: analyze the recorded bag

`gcopter/test/analyze_bad_traj_bag.py` evaluates every `gcopter/PolyTraj` message and flags trajectories that leave the planning volume:

```bash
rosrun gcopter analyze_bad_traj_bag.py \
    src/data_manager/dragon_nav/data/rosbag/20260622_naraha/2026-06-23-15-11-32_quad_bad_traj.bag \
    --show-log
```

Expected observations (54 trajectories, all with only 2-3 quintic pieces):

- Five trajectories are flagged `OUT-OF-BOUND`, with extents such as x_min = -161.73 m (traj_id 18), x_max = +13.85 m (id 25), y_min = -10.22 m (id 51).
- Roughly a quarter of the trajectories bulge to z ~ 5.5-6.1 m mid-flight (hugging the 6 m map ceiling) although start and goal are below 3 m.
- With `--show-log`, the planner log shows ~20 % of replans failing outright ("GCOPTER optimization failed") and the anomalous successes being time-scaled by factors of 5-24 ("max velocity 1.060 -> 0.198 m/s" etc.), which is how geometrically wild solutions passed the only sanity check.

The per-piece structure makes the mechanism visible: each bad trajectory travels to an absurd waypoint, stops there (v ~ 0 at the piece junction), and returns to the local target — the signature of a corrupted waypoint basis rather than polynomial oscillation.

## Reproduction 2: offline corridor degeneracy demo

`gcopter/test/corridor_degeneracy_repro.cpp` runs the exact corridor pipeline from the `gcopter` package (`convexCover` -> `shortCut` -> row normalization -> overlap stacking -> `enumerateVs`) on synthetic scenes shaped like the Naraha experiment (16 x 16 m map, voxel-grid ground surface, 3 m horizon-truncated routes), then validates every enumerated vertex against the polytope's own half-space constraints. No ROS master or bag is needed.

```bash
catkin build gcopter   # or catkin_make
rosrun gcopter corridor_degeneracy_repro
```

Pre-fix expected output (deterministic, fixed RNG seeds):

- **Scenario A (optimizer path):** out of 60 overlap enumerations, 12 return vertices that violate their own half-space constraints, 5 of them by more than 1 m — ranging from ~5 m to ~1e17 m — while `enumerateVs()` reports success for all of them.
- **Scenario B (visualization path):** with sparse airborne clutter points added (rain / dust / vegetation returns), roughly three quarters of the single-polytope enumerations return invalid vertices, two thirds violating by more than 1 m (worst cases reach 1e33 m and infinity) — this is what the out-of-bound corridor mesh in RViz was showing. Log lines appear in pairs because `shortCut()` duplicates a lone polytope and the visualizer draws both copies.

The program exits with status 1 while the degeneracy reproduces.

Fixed expected output:

```text
Scenario A (processCorridor overlap enumeration): 60 enumerations, 0 hard failures, 0 with invalid vertices (0 violating by > 1 m), worst constraint violation 6.95126e-06 m, worst |vertex coordinate| 6 m
Scenario B (visualizePolytope single enumeration): 400 enumerations, 0 hard failures, 0 with invalid vertices (0 violating by > 1 m), worst constraint violation 1.6181e-06 m, worst |vertex coordinate| 6 m

No degeneracy observed.
```

The fixed program exits with status 0, so this executable can be kept as a regression check for corridor vertex enumeration.

## Affected code

| Stage | Location |
| --- | --- |
| Vertex enumeration (root cause) | `gcopter/include/gcopter/geo_utils.hpp` (`enumerateVs`, `filterVs`) |
| Overlap construction consuming it | `gcopter/include/gcopter/gcopter.hpp` (`processCorridor`, `forwardP`) |
| Corridor generation (duplicated lone polytope) | `gcopter/include/gcopter/sfc_gen.hpp` (`shortCut`; `convexCover` also ignores `firi()`'s return value) |
| Publication without position check | `gcopter/src/planner_common.cpp` (`enforceVelocityLimit`) |
| Visualization consuming it | `gcopter/include/misc/visualizer.hpp` (`visualizePolytope`) |

## Implemented fix and remaining hardening

Implemented:

1. Replace the fragile dual QuickHull vertex recovery in `enumerateVs()` with triple-plane enumeration plus H-constraint validation.
2. Reject non-finite and constraint-violating vertices before any caller can use them.
3. Keep the existing `enumerateVs()` API so `processCorridor()` and `visualizePolytope()` receive validated vertices without call-site changes.

Recommended remaining hardening:

1. Validate trajectories before publishing: sample positions against the corridor (or at least `map_bound`) and treat violations — and large time scale factors (> ~1.2) — as optimization failure, keeping the previous trajectory.
2. Deduplicate / merge nearly parallel planes before enumerating stacked overlap polytopes; avoid the `[P, P]` duplication for single-polytope corridors.
3. Match parameters to the low-speed short-horizon regime: reduce `convexCover` progress (7 m) below the planning horizon, pass a finite `lengthPerPiece` to `GCOPTER_PolytopeSFC::setup()` instead of `INFINITY`, and increase `IntegralIntervs` so constraint sampling stays dense on minutes-long pieces.

## Note on the experiment configuration

The snapshot in `data_manager/.../20260622_naraha/20260622_exp_config/quad_gcopter.launch` (`map_bound = [-8, 8, -8, 8, 0.4, 0.8]`) predates the afternoon flight and does not match the bag: the recorded goal (-0.82, -3.21, 2.51) was accepted without clamping and planning succeeded from z = -0.06 m, which implies the actual bounds were x, y = ±8 m with z from a negative value up to 6.0 m. The trajectories repeatedly touching z ~ 6.0 m corroborate the 6 m ceiling.
