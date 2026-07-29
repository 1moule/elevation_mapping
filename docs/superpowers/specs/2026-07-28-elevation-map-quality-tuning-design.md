# Elevation Map Quality Tuning Design

## Goal

Improve the downward-facing MID360 elevation map on the existing unfiltered
bag. Prioritize flat floor and platform surfaces, accurate heights, and sharp
platform edges. Fill only small holes whose surrounding elevations agree.
Large unobserved regions remain unknown.

## Evidence

With a 0.02 m grid, the raw map contained about 47% valid cells and the final
fused map about 72%. Global inpainting reached 100% coverage but interpolated
across platform edges and increased the high-region height dispersion.

Changing only the grid resolution to 0.04 m increased raw valid coverage to
about 82%, while preserving approximately 0.018 m floor median absolute
deviation. This is the initial baseline for further comparison.

## Approach

1. Compare 0.03 m, 0.04 m, and 0.05 m resolutions with the same unfiltered
   bag and otherwise identical parameters.
2. Keep the original single-height Kalman map, scan-cell representative
   selection, visibility cleanup, and fused-map interface.
3. On a copy of the fused map immediately before publication, find connected
   components of invalid elevation cells.
4. Fill only components no larger than four cells.
5. Require at least four valid neighboring cells and a neighboring elevation
   range no greater than 0.05 m.
6. Fill accepted holes with the median neighboring elevation. Do not modify
   the persistent raw or fused map state.
7. Do not use global inpainting as the authoritative height layer.

## Runtime

The launch file exposes `use_sim_time`. Bag playback uses
`use_sim_time:=true` for both elevation mapping and RViz. Live sensor operation
keeps the default `false`.

RViz continues to display `/elevation_map` with the `elevation` layer, so
downstream `(x, y)` height sampling keeps the existing topic and layer
contract.

## Verification

- Unit tests cover accepted and rejected hole components, edge rejection, and
  unchanged valid cells.
- The package builds in Release mode.
- Replay the unfiltered bag for each candidate resolution.
- Select the resolution with the best coverage while keeping floor median
  absolute deviation below 0.025 m and avoiding visibly rounded platform
  edges.
- Start the selected configuration with RViz and leave it running for visual
  review.
