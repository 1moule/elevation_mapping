# Yaw-Aligned Elevation Map Design

## Goal

Optionally publish a second elevation map centered on `body` and aligned with
the body's yaw, while preserving the existing world-aligned map and world
absolute elevation values.

## Interface

- Parameter: `publish_yaw_aligned_map` (`false` by default).
- Topic: `/elevation_map_body`.
- Frame: `elevation_map_body`.
- Existing `/elevation_map` behavior remains unchanged.

## Data Flow

The raw and fused maps remain world-aligned. On fused-map publication, when the
feature is enabled:

1. Look up the latest `world -> body` transform.
2. Extract body X, Y, and yaw; ignore roll and pitch.
3. Create an output grid with the same length and resolution, centered at
   `(0, 0)` in `elevation_map_body`.
4. Transform each output cell center into world XY and copy the nearest world
   map cell. Nearest-neighbor sampling preserves step edges.
5. Keep `elevation`, bounds, and other height layers in world absolute Z.
6. Publish `world -> elevation_map_body` with body X/Y translation, zero Z, and
   yaw-only rotation.

## Failure Handling

If the body transform is unavailable, skip only the yaw-aligned publication
for that cycle. The existing world map continues publishing normally.

## Verification

- Feature disabled: no additional topic or TF; existing output is unchanged.
- Zero yaw: body map matches a translated crop of the world map.
- Nonzero yaw: cell positions rotate with body yaw while elevation values stay
  unchanged.
- Roll and pitch do not rotate the output grid.
