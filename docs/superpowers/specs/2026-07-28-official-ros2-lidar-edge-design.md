# Official ROS2 LiDAR Edge Design

## Goal

Make a downward-facing MID360 produce the same qualitative platform edge as
the official ROS1 depth-camera demo: stable flat top and floor cells separated
by a narrow rendered transition.

## Scope

Start from ROS2 baseline `710fffc`. Keep the original single-height Kalman map,
horizontal uncertainty fusion, and postprocessor. Do not add multimodal cells,
learned completion, GP interpolation, lower-surface recovery, or persistent
per-cell state.

## Data Flow

1. Transform and filter the MID360 cloud using the existing laser processor.
2. Reject points above the body origin through configuration.
3. Rasterize each scan to the elevation grid before temporal fusion.
4. Keep one point per cell: the point nearest the cell center in XY.
5. Fuse that representative once with the original elevation state.
6. Publish the original fused map and optional inpainted raw map.

This removes point-order dependence and prevents top, side, and floor returns
inside one cell from repeatedly overwriting each other.

## Sensor Uncertainty

Use the official variance value without the ROS2 port's `1e-11` scale. Clamp it
to the configured minimum variance. Match the official laser sensor Jacobian
rotation expression.

## MID360 Defaults

- Map size: 3 m by 3 m.
- Resolution: 0.02 m.
- Voxel size: 0.01 m.
- Ignore points above body: 0.0 m.
- Ignore points below body: -3.0 m.
- Mahalanobis threshold: 2.5.
- Inpaint radius: 0.05 m.
- Visibility cleanup enabled.

## Verification

Unit tests verify deterministic one-point-per-cell selection, center proximity,
out-of-map rejection, and corrected variance handling. Build the package and
replay the existing unfiltered bag for visual evaluation in RViz.
