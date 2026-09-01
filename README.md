# IG-RRTstar-rapid — PRM with EKF-Propagated Covariance

This `PRM-stigmergy` branch is rebuilt from the `PRM` branch. It preserves the
PRM code structure, environment, output format, plotting script, and collision
checker.

The one algorithmic change is that PRM vertices sample position only. The
covariance `P` is no longer randomly sampled at every vertex. It starts from
the initial covariance and is EKF-predicted each time the shortest-path search
extends a roadmap edge. Elapsed time is stored alongside each propagated
belief.

## Running the planner

1. Create a `data` folder in the repository root if it does not already
   exist.
2. Open MATLAB in this repository folder.
3. Run `main.m`.
4. Run `plot_for_paper_multipleObs.m` after changing its `load(...)` line to
   the name of the saved result, if necessary.

Results retain the PRM save format, so the existing plotter continues to use
`saver.path` and `node`. The saved node structure also includes elapsed time
as `node.t`. Saved names include marker mode, for example
`PRM_stigmergy_marker_on_N2000_lambda_1000_radius_05.mat`.

## EKF covariance propagation

For an accepted edge of length `Dtravel`, covariance is propagated as

```
P_next = F * P_current * F' + R * Dtravel
t_next = t_current + Dtravel / robot_speed
```

where the current configuration uses `F = I`, `R = 1e-4 * I` as process noise
per traveled meter, and `robot_speed = 1 m/s`. This is a temporary
prediction-only value selected
so that the confidence ellipse can fit within the current goal region; it
should be calibrated to the robot/process model before reporting final
results. The initial node remains the only node with a directly specified
covariance.

## Cost function

The planner minimizes

```
Dtotal = Dtravel + lambda * trace(Pgoal)
```

`Dtravel` is the original Euclidean edge-length term. `lambda` is configured
in `main.m`. Dijkstra uses the equivalent telescoping edge increment

```
Dtravel_edge + lambda * (trace(P_next) - trace(P_current))
```

and initializes the source cost to `lambda * trace(P_initial)`. Therefore the
stored distance at every node is exactly `Dtravel + lambda * trace(P)`.

With the current fixed-speed, prediction-only model, both time and covariance
are deterministic functions of traveled distance. They are stored as state
fields but do not create a costly expanded roadmap search.

## Marker comparison toggle

`main.m` defines `marker_enabled` near the PRM parameters.

- Set `marker_enabled = false` for the prediction-only baseline.
- Set `marker_enabled = true` to make the static marker available as one EKF
  measurement update when an edge enters its sensing radius.

The marker is optional: the planner compares paths that never sense it with
paths that do. For the enabled case, `dijkstra_ekf_marker_prm.m` uses two
finite Dijkstra layers (before and after sensing) instead of the earlier
unbounded multi-label search.

## Files used by this branch

- `main.m` — PRM parameters, environment, spatial sampling, neighborhood
  construction, search, goal selection, and data saving.
- `sample_free_position.m` — samples a collision-free spatial PRM vertex;
  does not sample covariance.
- `dijkstra_ekf_prm.m` — performs Dijkstra relaxation, propagates covariance
  and elapsed time, and evaluates the `Dtotal` edge increment.
- `dijkstra_ekf_marker_prm.m` — compares the no-marker and marker-sensed
  layers when `marker_enabled` is true.
- `marker_encounter.m` and `ekf_update_covariance.m` — detect marker-range
  entry and apply the EKF measurement update.
- `reconstruct_prm_path.m` — reconstructs the selected source-to-goal PRM
  path, unchanged from the PRM branch.
- `error_ellipse.m` and `psuedo_obs_check_line2_oct.m` — unchanged confidence
  ellipse and chance-constrained collision checking.
- `plot_for_paper_multipleObs.m` — unchanged PRM plotting script.

## Existing source files retained for comparison

`sample_x_P_randomly.m`, `sample_polyshape_check.m`, `check_lossless.m`,
`dijkstra_prm.m`, and `dist_ig_mat.m` are retained from the PRM branch but are
not called by this EKF-propagated version. They remain useful for direct
comparison with the random-covariance PRM implementation.
