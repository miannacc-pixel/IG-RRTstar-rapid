# IG-RRTstar-rapid — PRM with EKF-Propagated Covariance

This `PRM-stigmergy` branch is rebuilt from the `PRM` branch. It preserves the
PRM code structure, environment, output format, plotting script, original
cost-function interface, and collision checker.

The one algorithmic change is that PRM vertices sample position only. The
covariance `P` is no longer randomly sampled at every vertex. It starts from
the initial covariance and is EKF-predicted each time the shortest-path search
extends a roadmap edge.

## Running the planner

1. Create a `data` folder in the repository root if it does not already
   exist.
2. Open MATLAB in this repository folder.
3. Run `main.m`.
4. Run `plot_for_paper_multipleObs.m` after changing its `load(...)` line to
   the name of the saved result, if necessary.

Results retain the PRM save format, so the existing plotter continues to use
`saver.path` and `node`.

## EKF covariance propagation

For an accepted edge of length `Dtravel`, covariance is propagated as

```
P_next = F * P_current * F' + R * Dtravel
```

where the current configuration uses `F = I` and `R = 1e-4 * I` as process
noise per traveled meter. This is a temporary prediction-only value selected
so that the confidence ellipse can fit within the current goal region; it
should be calibrated to the robot/process model before reporting final
results. The initial node remains the only node with a directly specified
covariance.

Because `P_next` equals the predicted covariance used in the original
information-geometric distance, the original information term is zero under
this prediction-only model. The current path cost is consequently travel
distance, while `dist_ig_mat.m` is retained as the cost-function interface for
the planned later modification.

## Files used by this branch

- `main.m` — PRM parameters, environment, spatial sampling, neighborhood
  construction, search, goal selection, and data saving.
- `sample_free_position.m` — samples a collision-free spatial PRM vertex;
  does not sample covariance.
- `dijkstra_ekf_prm.m` — performs Dijkstra relaxation and EKF-predicts the
  covariance of each accepted candidate edge.
- `reconstruct_prm_path.m` — reconstructs the selected source-to-goal PRM
  path, unchanged from the PRM branch.
- `dist_ig_mat.m` — retains the existing travel-plus-information cost
  interface.
- `error_ellipse.m` and `psuedo_obs_check_line2_oct.m` — unchanged confidence
  ellipse and chance-constrained collision checking.
- `plot_for_paper_multipleObs.m` — unchanged PRM plotting script.

## Existing source files retained for comparison

`sample_x_P_randomly.m`, `sample_polyshape_check.m`, `check_lossless.m`, and
`dijkstra_prm.m` are retained from the PRM branch but are not called by this
EKF-propagated version. They remain useful for direct comparison with the
random-covariance PRM implementation.
