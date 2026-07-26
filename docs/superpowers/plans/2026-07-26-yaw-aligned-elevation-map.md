# Yaw-Aligned Elevation Map Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Publish an optional body-centered, yaw-aligned elevation map while preserving the existing world map and world absolute heights.

**Architecture:** Keep all mapping and fusion world-aligned. Add a pure nearest-neighbor resampler, then let `ElevationMapping` obtain body X/Y/yaw and request a second publication from `ElevationMap`; publish a matching yaw-only TF.

**Tech Stack:** ROS 2 Humble, grid_map, tf2, Eigen, GoogleTest.

## Global Constraints

- `publish_yaw_aligned_map` defaults to `false`.
- Publish the additional map on `/elevation_map_body` in frame `elevation_map_body`.
- Ignore body roll and pitch.
- Preserve world absolute Z values.
- Do not change `/elevation_map`.

---

### Task 1: Pure Yaw-Aligned Grid Resampling

**Files:**
- Create: `include/elevation_mapping/YawAlignedMap.hpp`
- Create: `src/YawAlignedMap.cpp`
- Create: `test/YawAlignedMapTest.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Produces: `grid_map::GridMap createYawAlignedMap(const grid_map::GridMap&, const grid_map::Position&, double, const std::string&)`.

- [ ] **Step 1: Write failing tests**

Add tests for zero yaw and 90-degree yaw. Populate a world map with deterministic elevation values, resample it, and assert that output cell positions select the expected world cells while copied elevation values remain unchanged.

- [ ] **Step 2: Verify tests fail**

Run:
```bash
cmake --build /home/guanlin/catkin_ws/build/elevation_mapping --target test_elevation_mapping_yaw_aligned_map -j2
```
Expected: failure because `YawAlignedMap.hpp` and the function do not exist.

- [ ] **Step 3: Implement nearest-neighbor resampling**

Create an output map with the source layers, length, resolution, and timestamp, centered at `(0, 0)`. For every output cell center `p_local`, calculate:
```cpp
p_world = bodyWorldPosition + Rotation2D(yaw) * p_local;
```
Use `source.getIndex(p_world, sourceIndex)` and copy every finite or non-finite layer value at that index. Set the output frame ID to `elevation_map_body`.

- [ ] **Step 4: Run tests**

Run the new test target and require all cases to pass.

- [ ] **Step 5: Commit**

```bash
git add CMakeLists.txt include/elevation_mapping/YawAlignedMap.hpp \
  src/YawAlignedMap.cpp test/YawAlignedMapTest.cpp
git commit -m "feat: add yaw-aligned grid resampling"
```

### Task 2: Parameter, Publisher, and Yaw-Only TF

**Files:**
- Modify: `include/elevation_mapping/ElevationMap.hpp`
- Modify: `src/ElevationMap.cpp`
- Modify: `include/elevation_mapping/ElevationMapping.hpp`
- Modify: `src/ElevationMapping.cpp`
- Modify: `config/robots/fastlio_mid360.yaml`
- Modify: `README.md`

**Interfaces:**
- Consumes: `createYawAlignedMap(...)` from Task 1.
- Produces: parameter `publish_yaw_aligned_map`, topic `/elevation_map_body`, and TF `world -> elevation_map_body`.

- [ ] **Step 1: Add publication method**

Add:
```cpp
bool ElevationMap::publishYawAlignedElevationMap(
    const grid_map::Position& bodyWorldPosition, double bodyYaw,
    const std::string& frameId);
```
It copies the fused map under its mutex, applies existing publication hole filling, resamples it, adds `uncertainty_range`, and publishes through a new `elevation_map_body` publisher.

- [ ] **Step 2: Add parameter and TF flow**

Declare/get `publish_yaw_aligned_map` in `ElevationMapping`. When enabled after each fused publication, look up the latest map-frame-to-body transform, extract yaw with tf2, publish the yaw-aligned map, and broadcast a transform with body X/Y, zero Z, yaw rotation, parent `map_frame_id`, and child `elevation_map_body`.

- [ ] **Step 3: Add configuration and documentation**

Set `publish_yaw_aligned_map: true` in `fastlio_mid360.yaml`. Document the additional topic, yaw-only frame, nearest-neighbor sampling, and world absolute height semantics.

- [ ] **Step 4: Build and verify**

Run:
```bash
source /opt/ros/humble/setup.bash
cd /home/guanlin/catkin_ws
colcon build --packages-select elevation_mapping --cmake-args -DCMAKE_BUILD_TYPE=Release
./build/elevation_mapping/test_elevation_mapping_yaw_aligned_map
```
Then run the filtered Bag and verify both `/elevation_map` and `/elevation_map_body` exist and `world -> elevation_map_body` contains body yaw with zero roll/pitch.

- [ ] **Step 5: Commit**

```bash
git add include/elevation_mapping/ElevationMap.hpp \
  include/elevation_mapping/ElevationMapping.hpp src/ElevationMap.cpp \
  src/ElevationMapping.cpp config/robots/fastlio_mid360.yaml README.md
git commit -m "feat: publish body-centered yaw-aligned elevation map"
```
