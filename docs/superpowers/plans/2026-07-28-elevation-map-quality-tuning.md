# Elevation Map Quality Tuning Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Produce the best measured MID360 elevation map from the unfiltered bag while preserving sharp height discontinuities and filling only small, locally consistent holes.

**Architecture:** Keep the existing scan rasterization and single-height fusion unchanged. Apply a bounded connected-component hole filler only to the fused map publication copy, then select the best of 0.03 m, 0.04 m, and 0.05 m resolutions using the same bag and metrics.

**Tech Stack:** ROS 2 Humble, C++14, grid_map, GoogleTest, Python ROS 2 diagnostics, RViz2.

## Global Constraints

- Fill only invalid connected components no larger than four cells.
- Require at least four valid neighboring cells.
- Reject filling when neighboring elevation range exceeds 0.05 m.
- Do not modify persistent raw or fused map state.
- Keep `/elevation_map` and its `elevation` layer as the downstream interface.
- Use `use_sim_time:=true` for bag playback and `false` by default for live sensors.

---

### Task 1: Finalize MID360 Runtime Baseline

**Files:**
- Modify: `config/robots/fastlio_mid360.yaml`
- Modify: `launch/elevationMapping_fastlio_launch.py`

**Interfaces:**
- Produces: launch argument `use_sim_time` and an initial `resolution: 0.04`.

- [ ] **Step 1: Verify the current installed launch does not expose the required behavior**

Run:

```bash
ros2 launch elevation_mapping elevationMapping_fastlio_launch.py --show-args
```

Expected before the runtime edit: no `use_sim_time` launch argument.

- [ ] **Step 2: Set the initial resolution and launch argument**

Use `resolution: 0.04` in the MID360 YAML. Declare:

```python
use_sim_time = launch.substitutions.LaunchConfiguration("use_sim_time")
launch.actions.DeclareLaunchArgument(
    "use_sim_time", default_value="false",
    description="Use the clock published by rosbag playback.",
)
```

Pass `{"use_sim_time": use_sim_time}` to both the mapping and RViz nodes.

- [ ] **Step 3: Build and verify the runtime parameters**

Run:

```bash
colcon build --base-paths src/elevation_mapping \
  --packages-select elevation_mapping \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
ros2 launch elevation_mapping elevationMapping_fastlio_launch.py --show-args
```

Expected: build succeeds and `use_sim_time` is listed with default `false`.

- [ ] **Step 4: Commit the runtime baseline**

```bash
git add config/robots/fastlio_mid360.yaml \
  launch/elevationMapping_fastlio_launch.py
git commit -m "fix: tune MID360 runtime for bag playback"
```

### Task 2: Add Edge-Constrained Small-Hole Filling

**Files:**
- Create: `include/elevation_mapping/SmallHoleFiller.hpp`
- Create: `src/SmallHoleFiller.cpp`
- Create: `test/SmallHoleFillerTest.cpp`
- Modify: `src/ElevationMap.cpp`
- Modify: `include/elevation_mapping/ElevationMap.hpp`
- Modify: `CMakeLists.txt`
- Modify: `config/robots/fastlio_mid360.yaml`

**Interfaces:**
- Produces:

```cpp
struct SmallHoleFillingParameters {
  std::size_t maxHoleSize;
  std::size_t minSupport;
  float maxHeightRange;
};

std::size_t fillSmallElevationHoles(
    grid_map::GridMap& map,
    const SmallHoleFillingParameters& parameters);
```

- [ ] **Step 1: Write failing unit tests**

Cover these real map behaviors:

```cpp
TEST(SmallHoleFillerTest, FillsSingleCellHoleWithMedianSupport);
TEST(SmallHoleFillerTest, RejectsComponentLargerThanMaximum);
TEST(SmallHoleFillerTest, RejectsHoleAcrossHeightDiscontinuity);
TEST(SmallHoleFillerTest, LeavesExistingElevationsUnchanged);
```

Each accepted fill must populate `elevation`, `upper_bound`, and
`lower_bound`; rejection must leave all three invalid.

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```bash
colcon build --base-paths src/elevation_mapping \
  --packages-select elevation_mapping \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
./build/elevation_mapping/test_small_hole_filler
```

Expected: build fails because `SmallHoleFiller.hpp` or
`fillSmallElevationHoles()` does not exist.

- [ ] **Step 3: Implement the minimal filler**

Use four-connectivity to discover invalid `elevation` components. For
components at or below `maxHoleSize`, gather unique valid eight-neighbors.
Require `minSupport`, reject if `max(elevation) - min(elevation)` exceeds
`maxHeightRange`, then assign the median neighbor value independently for
`elevation`, `upper_bound`, and `lower_bound`.

- [ ] **Step 4: Apply the filler only to the publication copy**

In `ElevationMap::publishFusedElevationMap()`, after copying `fusedMap_` and
before adding `uncertainty_range`, call:

```cpp
if (enableSmallHoleFilling_) {
  fillSmallElevationHoles(fusedMapCopy, smallHoleFillingParameters_);
}
```

Declare and read these parameters in `ElevationMap`:

```yaml
enable_small_hole_filling: true
small_hole_max_size: 4
small_hole_min_support: 4
small_hole_max_height_range: 0.05
```

- [ ] **Step 5: Run focused and existing tests**

Run:

```bash
./build/elevation_mapping/test_small_hole_filler
./build/elevation_mapping/test_scan_cell_selector
```

Expected: all focused tests pass. Record the unchanged known failures from
`test_elevation_mapping_input_sources` separately.

- [ ] **Step 6: Commit the filler**

```bash
git add CMakeLists.txt include/elevation_mapping/SmallHoleFiller.hpp \
  include/elevation_mapping/ElevationMap.hpp src/SmallHoleFiller.cpp \
  src/ElevationMap.cpp test/SmallHoleFillerTest.cpp \
  config/robots/fastlio_mid360.yaml
git commit -m "feat: fill small edge-consistent elevation holes"
```

### Task 3: Select and Run the Best Resolution

**Files:**
- Modify only if selected value changes: `config/robots/fastlio_mid360.yaml`

**Interfaces:**
- Consumes: `/fastlio2/body_cloud`, `/fastlio2/lio_odom`, `/tf`.
- Produces: `/elevation_map` for RViz and downstream height sampling.

- [ ] **Step 1: Replay the same bag at 0.03 m**

Run mapping with `use_sim_time:=true`, replay
`rosbag2_2026_07_25-09_22_56 --clock 100`, and record final valid coverage,
floor median absolute deviation, high-region median absolute deviation, and
neighbor height differences.

- [ ] **Step 2: Repeat at 0.04 m and 0.05 m**

Change no parameter except `resolution`. Use a fresh mapping process for every
replay so map state and simulated time start clean.

- [ ] **Step 3: Select the best candidate**

Reject candidates with floor median absolute deviation above 0.025 m. Among
the remaining candidates, prefer the one with greatest valid coverage unless
RViz shows visibly rounded platform boundaries.

- [ ] **Step 4: Save and build the selected resolution**

Update `fastlio_mid360.yaml` only when the winner differs from 0.04 m, then
run the Release build and focused tests again.

- [ ] **Step 5: Start final RViz validation**

Run:

```bash
ros2 launch elevation_mapping elevationMapping_fastlio_launch.py \
  use_sim_time:=true
ros2 bag play rosbag2_2026_07_25-09_22_56 --clock 100
```

Leave elevation mapping and RViz running after the bag completes.

- [ ] **Step 6: Commit the selected parameter if changed**

```bash
git add config/robots/fastlio_mid360.yaml
git commit -m "tune: select MID360 elevation map resolution"
```
