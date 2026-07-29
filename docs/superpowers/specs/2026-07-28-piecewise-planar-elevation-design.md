# Piecewise-Planar Elevation Design

## Goal

Reduce small bumps on floors, platforms, and stair treads while preserving
sharp height discontinuities. Complete thin unobserved strips between visible
horizontal surfaces without inventing intermediate ramp heights. Support
multiple platforms and stair levels.

## Representation Limit

The elevation map returns one height for each `(x, y)`. It cannot represent a
vertical wall containing multiple heights at the same horizontal coordinate.
An inferred platform side is therefore represented as a sharp transition
between adjacent high and low cells, not as a set of intermediate heights.

TF supplies pose, not unseen geometry. Existing point-cloud transforms and the
rolling fused map already align observations in the `world` frame. The new
algorithm uses that TF-aligned history to fit surfaces and infer a boundary;
it does not claim that TF directly measures the hidden platform side.

## Processing Scope

Operate only on the fused publication copy immediately before
`/elevation_map` is published. Do not change the persistent raw map, fused map,
Kalman fusion, scan-cell selector, or sensor variance model.

The existing small-hole filler remains an earlier publication-copy stage.
Piecewise-planar processing runs after it and before `uncertainty_range` is
computed.

## Surface Segmentation

1. Use finite `elevation`, `upper_bound`, and `lower_bound` cells as observed
   support.
2. Form eight-connected regions. Neighboring cells may join when their height
   difference is at most `0.06 m`.
3. Require at least 12 cells before fitting a region.
4. Fit `z = ax + by + c` robustly with iteratively reweighted least squares.
5. Accept a fitted region only when its residual median absolute deviation is
   at most `0.03 m`.

The algorithm may produce any number of local planes. Plane height is evaluated
at each cell position, so gently sloped floors and treads remain sloped rather
than being forced horizontal.

## Surface Regularization

For an observed cell belonging to an accepted plane, replace its published
height with the plane prediction only when its absolute residual is at most
`0.05 m`. Keep cells with larger residuals unchanged so real obstacles are not
flattened into the floor.

Shift `upper_bound` and `lower_bound` by the same height correction. This
preserves the cell's existing uncertainty width.

## Thin Occlusion Completion

An invalid cell is eligible only when:

- it lies within `0.12 m` of observed support from an accepted plane;
- its connected unknown band borders at least two accepted plane regions whose
  predicted heights differ by at least `0.06 m`;
- each bordering candidate plane supplies at least four nearby support cells;
- the nearest accepted plane prediction is locally consistent with its support
  within `0.05 m`; and
- the cell belongs to a thin unknown band, meaning every filled cell is within
  `0.12 m` of observed support. Large unseen interiors remain unknown.

Single-surface holes remain the responsibility of the existing small-hole
filler. Piecewise-planar completion only handles an occluded band bounded by
distinct height levels.

Choose the bordering plane whose observed support is nearest in XY. This
creates a discrete Voronoi boundary between high and low surfaces. Never
average plane heights.

For an inferred cell, set:

- `elevation` to the selected plane prediction;
- `lower_bound` to prediction minus `0.05 m`;
- `upper_bound` to prediction plus `0.05 m`.

Inference reads one immutable snapshot. Newly inferred cells cannot become
support for other inferred cells in the same publication.

## Parameters

The feature is opt-in and enabled only in `fastlio_mid360.yaml`:

```yaml
enable_piecewise_planar_regularization: true
planar_neighbor_height_tolerance: 0.06
planar_min_region_size: 12
planar_max_region_mad: 0.03
planar_max_regularization_residual: 0.05
planar_max_occlusion_distance: 0.12
planar_min_occlusion_support: 4
planar_inferred_half_range: 0.05
```

All other robot profiles default to disabled.

## Verification

Unit tests verify:

- noisy samples on one plane become planar;
- a gently sloped plane retains its slope;
- two or more levels remain separate;
- a thin gap between levels contains only one of the fitted plane heights;
- an unknown region adjacent to only one plane is not extrapolated;
- a wide unknown area remains invalid;
- large residual obstacles remain unchanged;
- disabled processing leaves the map unchanged; and
- processing a copy does not change the persistent source map.

Replay the unfiltered bag with a fresh mapper and `use_sim_time:=true`.
Compared with the current 0.04 m baseline:

- floor median absolute deviation must decrease by at least 20%;
- finite coverage must not decrease;
- inferred cells must not form intermediate-height ramps between platform
  levels; and
- RViz must retain visibly distinct platform and stair boundaries.
