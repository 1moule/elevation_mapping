# Multimodal Edge Elevation Design

## Context

The current elevation map stores one height estimate per XY cell. At a step or
platform edge, one 4 cm cell can receive returns from the upper horizontal
surface, lower horizontal surface, and vertical face. These measurements then
compete for one state and can make the published height switch between the
upper and lower surfaces.

The recorded bag
`/home/guanlin/catkin_ws/rosbag2_2026_07_25-09_22_56` contains 28.7 seconds of
body-frame point clouds, LIO odometry, and TF at approximately 10 Hz. An
isolated replay successfully drives the current elevation mapping node.

Transforming each cloud to `world` and measuring the per-scan height span inside
XY cells produced the following evidence:

| Resolution | Mean cells with span > 5 cm | 95th percentile |
| --- | ---: | ---: |
| 4 cm | 16.08% | 20.40% |
| 2 cm | 10.88% | 15.89% |
| 1 cm | 6.19% | 11.21% |

Reducing the global resolution mitigates but does not eliminate multimodal
cells. It also increases memory, computation, and map holes. The selected
design therefore keeps the 4 cm public grid and models two height modes inside
ambiguous cells.

## Goals

- Stop repeated upper/lower height switching at step and platform edges.
- Select the surface that covers the cell center or the largest XY area.
- Preserve the existing `grid_map` geometry and `elevation` layer interface.
- Keep flat-region height accuracy and finite-cell coverage at least as good as
  the current implementation.
- Make the recorded bag a deterministic regression test.

## Non-Goals

- Represent arbitrary overhangs or a complete 3D mesh.
- Replace `grid_map` with an octree or adaptive quadtree.
- Change FAST-LIO2, radar timing, or the public map coordinate frame.
- Infer a traversability policy such as always preferring the higher surface.

## Alternatives

### Global 1-2 cm Grid

This is simple and keeps one height per cell, but the bag still contains
multimodal cells at 1 cm. A 1 cm grid has sixteen times as many cells as a
4 cm grid over the same area and is more susceptible to holes.

### Adaptive Quadtree

Local spatial subdivision can represent edges accurately, but it is not
compatible with the existing `grid_map` message, RViz configuration, and
downstream consumers. It would require a broader mapping rewrite.

### Two Height Modes with In-Cell Spatial Support

This is the selected approach. It resolves the observed vertical switching
while retaining the existing public map and bounded per-cell storage.

## Architecture

### Scan Observation Construction

Before updating the persistent raw map, group all points in the scan by their
4 cm output cell. Build up to two height clusters per occupied cell. Clusters
are separated when their height difference exceeds a configurable threshold,
initially 5 cm.

For each cluster, compute:

- mean height and measurement variance;
- number of supporting points;
- occupied bins in a 3x3 XY micro-grid inside the cell;
- whether the center micro-bin is occupied;
- color and lowest-point metadata needed by the existing map.

The micro-grid is only an observation feature. It does not change the public
map resolution.

If more than two height groups are present, retain the two groups with the
strongest horizontal support. This suppresses thin vertical-face returns,
which occupy fewer distinct XY micro-bins than horizontal surfaces.

### Persistent Cell State

An ambiguous cell maintains at most two persistent surface modes. Each mode
stores:

- filtered elevation and variance;
- confidence and last observation time;
- recent XY coverage score;
- consecutive observation count.

Modes from the current scan are matched to persistent modes by height, not by
input order. New modes start with low confidence. Stale modes expire after a
configurable timeout.

The state must move with the existing `grid_map` circular buffer. Internal mode
state will therefore be represented by non-published, map-aligned layers so
that move and region-clear operations use the same indexing as `elevation`.

### Primary Surface Selection

The public `elevation` layer remains single-valued. Its primary mode is selected
in this order:

1. Prefer a sufficiently supported mode that occupies the center micro-bin.
2. Otherwise prefer the mode occupying more distinct XY micro-bins.
3. If spatial support is tied, retain the current primary mode.

A challenger must remain preferable for several consecutive scans and exceed
the incumbent by a configurable support margin before the primary mode
switches. This hysteresis prevents viewpoint-dependent upper/lower toggling.

The initial defaults will be calibrated with the recorded bag:

- height mode separation: 5 cm;
- minimum points per mode: three;
- minimum horizontal support: two micro-bins;
- challenger coverage margin: one micro-bin;
- primary switch confirmation: three scans;
- stale secondary mode timeout: 0.5 seconds.

### Integration with Existing Fusion

- Single-mode cells continue through the existing elevation and variance
  fusion path.
- Confirmed two-mode cells bypass the generic lower-point recovery and adaptive
  lower-surface path for that scan. Each mode is updated independently, then
  the selected primary mode is copied to `elevation`.
- Existing lower-surface recovery remains available for cells where only a
  coherent new lower surface is observed.
- Hole filling never overwrites a valid primary mode and must not bridge
  neighbors whose primary or secondary modes indicate a height discontinuity.
- Fused map publication remains at 20 Hz; input processing remains tied to the
  approximately 10 Hz point cloud.

### Diagnostics

Add optional diagnostic layers:

- `secondary_elevation`;
- `height_mode_count`;
- `height_mode_separation`;
- `primary_mode_confidence`.

These layers support bag analysis and RViz inspection. Existing consumers can
continue reading only `elevation`.

The feature is controlled by `enable_multimodal_cells`. It is enabled in the
MID360 configuration after validation, while the legacy path remains available
for A/B replay and rollback.

## Error Handling

- Reject non-finite point heights before clustering.
- Ignore clusters below minimum point and XY support thresholds.
- Never replace a finite, confident primary mode with an unsupported mode.
- Clear both modes when a circular-buffer region is newly allocated.
- Fall back to the existing single-height update when a cell has no valid
  multimodal observation.

## Testing

### Unit and Integration Tests

Add tests for:

- two height modes in both point orders;
- center-bin preference;
- larger XY coverage preference;
- tie behavior retaining the incumbent;
- switch confirmation and hysteresis;
- stale secondary mode expiration;
- rejection of narrow vertical-face clusters;
- circular-buffer map movement;
- interaction with existing lower-surface recovery and hole filling.

### Bag Replay

Replay `rosbag2_2026_07_25-09_22_56` in an isolated ROS domain and record
`/elevation_map_raw_post` and `/elevation_map`.

Define a vertical switch as a primary-height change greater than 8 cm in the
same world cell between consecutive raw-map updates, followed by a return to
the previous mode within one second.

Acceptance criteria:

- reduce vertical switches in multimodal edge cells by at least 80%;
- retain at least 95% of the baseline finite-cell coverage;
- do not increase the 95th-percentile temporal height variation in flat cells;
- keep the visible edge transition within one 4 cm output cell;
- produce deterministic metrics across three identical replays;
- increase elevation-mapping CPU usage by no more than 25% over the current
  replay baseline.

## Rollout

1. Add pure clustering and primary-selection tests.
2. Implement scan-local multimodal observations behind the feature flag.
3. Add persistent mode state and hysteresis.
4. Integrate with raw/fused map publication and diagnostics.
5. Run unit tests and the recorded bag A/B comparison.
6. Enable the feature in `fastlio_mid360.yaml` only after all acceptance
   criteria pass.
