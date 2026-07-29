# Official ROS2 LiDAR Edge Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a minimal scan rasterization path and official uncertainty behavior for downward-facing MID360 elevation mapping.

**Architecture:** Preserve the original map and insert a stateless selector immediately before its point-fusion loop. Configuration performs terrain-height filtering and uses camera-like spatial resolution.

**Tech Stack:** ROS2 Humble, C++14, grid_map, PCL, GoogleTest, YAML launch parameters.

## Global Constraints

- Base commit is `710fffcba00dc4df3c3f4793194e19c38bb88307`.
- Do not add multimodal map state or a second mapping node.
- Keep the existing public ROS topics and map layers.

---

### Task 1: Scan Cell Representative Selector

**Files:**
- Create: `include/elevation_mapping/ScanCellSelector.hpp`
- Create: `src/ScanCellSelector.cpp`
- Create: `test/ScanCellSelectorTest.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: `grid_map::GridMap`, `PointCloudType`
- Produces: `std::vector<std::size_t> selectScanCellRepresentatives(...)`

- [ ] Write tests for one point per cell, nearest-center selection, deterministic ties, and out-of-map points.
- [ ] Run the test and verify it fails because the selector is missing.
- [ ] Implement the stateless selector.
- [ ] Run the selector tests and verify they pass.

### Task 2: Original Map Integration and Laser Variance

**Files:**
- Modify: `src/ElevationMap.cpp`
- Modify: `src/sensor_processors/LaserSensorProcessor.cpp`
- Test: `test/ScanCellSelectorTest.cpp`

**Interfaces:**
- Consumes: selector indices and point-cloud variance vector.
- Produces: at most one temporal fusion update per scan and grid cell.

- [ ] Add a failing test that verifies selected variances retain SI units and respect the minimum variance.
- [ ] Replace the all-point fusion loop with selector-index iteration.
- [ ] Remove the `1e-11` scale and clamp to `minVariance_`.
- [ ] Match the official ROS1 laser sensor Jacobian expression.
- [ ] Run focused tests.

### Task 3: MID360 Runtime Configuration

**Files:**
- Create: `config/robots/fastlio_mid360.yaml`
- Create: `launch/elevationMapping_fastlio_launch.py`
- Create: `rviz2/fastlio_elevation.rviz`

- [ ] Add the simplified 2 cm MID360 configuration.
- [ ] Add launch and RViz files using `/fastlio2/body_cloud`, `/fastlio2/lio_odom`, `world`, and `body`.
- [ ] Build `elevation_mapping`.
- [ ] Run all available package tests and record baseline failures separately.
- [ ] Replay the unfiltered bag and inspect the resulting elevation distribution and RViz edge.
