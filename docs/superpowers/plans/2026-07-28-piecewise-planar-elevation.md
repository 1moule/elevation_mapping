# Piecewise-Planar Elevation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Flatten LiDAR noise on floors, platforms, and stair treads while filling only thin occluded bands between distinct height levels with discrete plane heights.

**Architecture:** Add one publication-only `PiecewisePlanarProcessor` beside the existing `SmallHoleFiller`. It segments valid fused cells into local height-continuous regions, robustly fits one plane per accepted region, regularizes inliers, then assigns eligible unknown-band cells to the nearest bordering plane without averaging heights. `ElevationMap` keeps an immutable pre-fill support copy so neither small-hole results nor newly inferred cells can recursively become planar support.

**Tech Stack:** C++17, Eigen, grid_map_core, ROS 2 Humble parameters, GoogleTest, colcon.

## Global Constraints

- Do not change the persistent raw map, fused map, Kalman fusion, scan-cell selector, or sensor variance model.
- Run planar processing on the fused publication copy after small-hole filling and before `uncertainty_range`.
- Use only finite `elevation`, `upper_bound`, and `lower_bound` cells from the pre-fill fused snapshot as support.
- Segment with 8-connectivity and `0.06 m` maximum neighboring height difference.
- Require 12 cells and residual MAD at most `0.03 m` for an accepted plane.
- Regularize only cells whose absolute plane residual is at most `0.05 m`.
- Fill only candidate bands within `0.12 m` of support that border at least two accepted planes differing by at least `0.06 m`.
- Require at least four locally consistent support cells per bordering plane.
- Select the nearest plane in XY and never average plane heights.
- Set inferred bounds to the selected height plus or minus `0.05 m`.
- Keep the feature disabled by default and enable it only in `fastlio_mid360.yaml`.

---

### Task 1: Robust Plane Segmentation And Inlier Regularization

**Files:**
- Create: `include/elevation_mapping/PiecewisePlanarProcessor.hpp`
- Create: `src/PiecewisePlanarProcessor.cpp`
- Create: `test/PiecewisePlanarProcessorTest.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: a read-only support `grid_map::GridMap` and a writable publication copy with identical geometry.
- Produces:

```cpp
struct PiecewisePlanarParameters {
  float neighborHeightTolerance;
  std::size_t minRegionSize;
  float maxRegionMad;
  float maxRegularizationResidual;
  float maxOcclusionDistance;
  std::size_t minOcclusionSupport;
  float inferredHalfRange;
};

struct PiecewisePlanarResult {
  std::size_t acceptedPlanes;
  std::size_t regularizedCells;
  std::size_t inferredCells;
};

constexpr bool defaultPiecewisePlanarRegularizationEnabled();

PiecewisePlanarResult processPiecewisePlanarElevationIfEnabled(
    const grid_map::GridMap& supportMap,
    grid_map::GridMap& outputMap,
    bool enabled,
    const PiecewisePlanarParameters& parameters);

PiecewisePlanarResult processPiecewisePlanarElevation(
    const grid_map::GridMap& supportMap,
    grid_map::GridMap& outputMap,
    const PiecewisePlanarParameters& parameters);
```

- [ ] **Step 1: Register the new source and focused test target**

Add `src/PiecewisePlanarProcessor.cpp` to `${PROJECT_NAME}_library`, then add:

```cmake
ament_add_gtest(test_piecewise_planar_processor
  test/PiecewisePlanarProcessorTest.cpp
)
target_link_libraries(test_piecewise_planar_processor
  ${PROJECT_NAME}_library
)
target_include_directories(test_piecewise_planar_processor
  PRIVATE
    include
)
```

- [ ] **Step 2: Write failing tests for disabled behavior, noisy-plane flattening, and bounds**

Create a `0.04 m` resolution map fixture with the three required layers. Add:

```cpp
TEST(PiecewisePlanarProcessorTest, DefaultConfigurationIsDisabled)
TEST(PiecewisePlanarProcessorTest, DisabledLeavesOutputUnchanged)
TEST(PiecewisePlanarProcessorTest, FlattensNoisyPlaneAndPreservesBoundWidth)
TEST(PiecewisePlanarProcessorTest, RetainsSlopeInsteadOfForcingHorizontal)
TEST(PiecewisePlanarProcessorTest, DoesNotModifyLargeResidualObstacle)
TEST(PiecewisePlanarProcessorTest, DoesNotModifySupportMap)
```

Assert `defaultPiecewisePlanarRegularizationEnabled()` is `false`.
For the noisy-plane case, populate a `5 x 5` region from
`z = 0.02 * x - 0.01 * y + 0.4` with deterministic alternating noise
`{-0.02, 0.02}`. Set bounds to `z +/- 0.03`, run processing, and assert:

```cpp
EXPECT_EQ(result.acceptedPlanes, 1u);
EXPECT_EQ(result.regularizedCells, 25u);
EXPECT_NEAR(output.at("elevation", center), 0.4, 1.0e-3);
EXPECT_NEAR(
    output.at("upper_bound", center) -
        output.at("lower_bound", center),
    0.06, 1.0e-5);
```

For the obstacle case, place one extra cell `0.20 m` above an otherwise valid
plane and assert its elevation and both bounds are bitwise unchanged.

- [ ] **Step 3: Build the test target and confirm it fails**

Run:

```bash
source /opt/ros/humble/setup.bash
source /home/guanlin/catkin_ws/install/setup.bash
colcon build --packages-select elevation_mapping \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select elevation_mapping \
  --ctest-args -R test_piecewise_planar_processor --output-on-failure
```

Expected: compilation fails because `PiecewisePlanarProcessor` is not defined.

- [ ] **Step 4: Implement validation, segmentation, and robust plane fitting**

In `PiecewisePlanarProcessor.cpp`:

1. Return zero counts when required layers are absent, map geometry differs, or
   any positive threshold is non-finite/non-positive.
2. Snapshot support validity before writing `outputMap`.
3. Build 8-connected regions using physical neighbor positions so circular
   grid-map storage does not break adjacency.
4. Join valid neighbors only when `abs(z0 - z1) <= neighborHeightTolerance`.
5. Reject regions smaller than `minRegionSize`.
6. Fit `z = ax + by + c` by weighted least squares. Start with unit weights,
   then perform five IRLS iterations with Huber weights:

```cpp
const double scale = std::max(1.4826 * medianAbsoluteDeviation, 1.0e-6);
const double cutoff = 1.345 * scale;
weight = absoluteResidual <= cutoff ? 1.0 : cutoff / absoluteResidual;
```

7. Reject a fit when Eigen's LDLT solve fails, coefficients are non-finite, or
   final residual MAD exceeds `maxRegionMad`.
8. Store accepted plane coefficients, region ID, and original support indices.

- [ ] **Step 5: Implement inlier-only publication regularization**

For every support cell in an accepted region, compute predicted height. When
`abs(originalHeight - prediction) <= maxRegularizationResidual`, apply:

```cpp
const float correction = predictedHeight - originalHeight;
outputMap.at("elevation", index) = predictedHeight;
outputMap.at("upper_bound", index) =
    supportMap.at("upper_bound", index) + correction;
outputMap.at("lower_bound", index) =
    supportMap.at("lower_bound", index) + correction;
```

Do not write to `supportMap`, and do not modify rejected regions or outliers.
Leave `inferredCells` at zero until Task 2.

- [ ] **Step 6: Run the focused tests**

Run:

```bash
colcon build --packages-select elevation_mapping \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select elevation_mapping \
  --ctest-args -R test_piecewise_planar_processor --output-on-failure
colcon test-result --verbose
```

Expected: all new Task 1 tests pass.

- [ ] **Step 7: Commit Task 1**

```bash
git add CMakeLists.txt \
  include/elevation_mapping/PiecewisePlanarProcessor.hpp \
  src/PiecewisePlanarProcessor.cpp \
  test/PiecewisePlanarProcessorTest.cpp
git commit -m "feat: regularize elevation map with local planes"
```

---

### Task 2: Multi-Level Thin Occlusion Completion

**Files:**
- Modify: `src/PiecewisePlanarProcessor.cpp`
- Modify: `test/PiecewisePlanarProcessorTest.cpp`

**Interfaces:**
- Consumes: accepted plane records and the immutable support validity snapshot produced in Task 1.
- Produces: discrete inferred cells in `outputMap`; increments `PiecewisePlanarResult::inferredCells`.

- [ ] **Step 1: Write failing tests for discrete multi-level completion**

Add:

```cpp
TEST(PiecewisePlanarProcessorTest, FillsThinBandWithOnlyBorderingPlaneHeights)
TEST(PiecewisePlanarProcessorTest, SupportsThreeIndependentHeightLevels)
TEST(PiecewisePlanarProcessorTest, DoesNotExtrapolateFromOnePlane)
TEST(PiecewisePlanarProcessorTest, LeavesWideUnknownInteriorInvalid)
TEST(PiecewisePlanarProcessorTest, InferredCellsDoNotPropagate)
```

Build low and high `4 x 5` support patches separated by a two-cell-wide invalid
band. Use plane heights `0.0 m` and `0.30 m`. Assert every newly finite band
cell is within `1e-4` of one of those heights:

```cpp
const float z = output.at("elevation", index);
EXPECT_TRUE(std::abs(z - 0.0f) < 1.0e-4f ||
            std::abs(z - 0.30f) < 1.0e-4f);
EXPECT_FALSE(z > 1.0e-4f && z < 0.2999f);
EXPECT_NEAR(output.at("upper_bound", index) - z, 0.05, 1.0e-5);
EXPECT_NEAR(z - output.at("lower_bound", index), 0.05, 1.0e-5);
```

For three levels, create separate `0.0`, `0.20`, and `0.45 m` patches and
verify each intervening narrow band uses only its two bordering plane
predictions. For the propagation test, make the unknown area extend more than
`0.12 m`; assert only cells within the original-support distance are filled.

- [ ] **Step 2: Run the focused test and confirm the new cases fail**

Run:

```bash
colcon build --packages-select elevation_mapping \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select elevation_mapping \
  --ctest-args -R test_piecewise_planar_processor --output-on-failure
```

Expected: new band cells remain invalid because completion is not implemented.

- [ ] **Step 3: Build the bounded candidate mask**

For each originally invalid cell, query accepted-plane support cells inside
`maxOcclusionDistance`. Record per plane:

```cpp
struct PlaneCandidate {
  std::size_t planeId;
  std::size_t consistentSupportCount;
  double nearestSquaredDistance;
  float predictedHeight;
};
```

Count only support cells whose absolute fit residual is at most
`maxRegularizationResidual`. A plane is eligible for the cell only when
`consistentSupportCount >= minOcclusionSupport`.

Implement the query by precomputing integer cell offsets inside
`ceil(maxOcclusionDistance / resolution)` and reading a region-ID matrix.
Reject offsets whose physical squared distance exceeds the configured radius.
Do not scan every plane or every support point for each invalid cell.

Create a candidate mask from cells having at least one eligible plane. Find
4-connected components in that mask, not in the complete invalid area. This
prevents a narrow band connected to the map's unknown exterior from being
rejected as one huge component.

- [ ] **Step 4: Require two distinct bordering height levels**

For each candidate-mask component, collect all eligible plane IDs. Keep it only
when at least one pair has predicted height separation at the component cell
of at least `neighborHeightTolerance`. A component adjacent only to one plane
remains invalid; same-plane holes stay the responsibility of
`SmallHoleFiller`.

- [ ] **Step 5: Assign cells without height averaging**

For each retained component cell:

1. Discard candidates that do not participate in any distinct-height pair.
2. Select the candidate with minimum `nearestSquaredDistance`.
3. Break equal-distance ties by lower stable plane ID.
4. Write only to `outputMap`:

```cpp
outputMap.at("elevation", index) = candidate.predictedHeight;
outputMap.at("lower_bound", index) =
    candidate.predictedHeight - parameters.inferredHalfRange;
outputMap.at("upper_bound", index) =
    candidate.predictedHeight + parameters.inferredHalfRange;
```

All candidate searches must read the Task 1 support snapshot, never
`outputMap`, so inferred cells cannot propagate.

- [ ] **Step 6: Run all processor and existing hole-filler tests**

Run:

```bash
colcon build --packages-select elevation_mapping \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select elevation_mapping \
  --ctest-args \
  -R 'test_(piecewise_planar_processor|small_hole_filler)' \
  --output-on-failure
colcon test-result --verbose
```

Expected: the new processor suite and all 8 existing small-hole tests pass.

- [ ] **Step 7: Commit Task 2**

```bash
git add src/PiecewisePlanarProcessor.cpp \
  test/PiecewisePlanarProcessorTest.cpp
git commit -m "feat: complete occluded bands between elevation planes"
```

---

### Task 3: ROS Parameter And Publication-Pipeline Integration

**Files:**
- Modify: `include/elevation_mapping/ElevationMap.hpp`
- Modify: `src/ElevationMap.cpp`
- Modify: `config/robots/fastlio_mid360.yaml`
- Modify: `test/PiecewisePlanarProcessorTest.cpp`

**Interfaces:**
- Consumes: Task 1's public processor API.
- Produces: opt-in ROS parameters and publication-copy invocation.

- [ ] **Step 1: Add processor state to `ElevationMap`**

Include `PiecewisePlanarProcessor.hpp` and add:

```cpp
bool enablePiecewisePlanarRegularization_;
PiecewisePlanarParameters piecewisePlanarParameters_;
```

Initialize exact defaults in the constructor:

```cpp
enablePiecewisePlanarRegularization_(
    defaultPiecewisePlanarRegularizationEnabled()),
piecewisePlanarParameters_{
    0.06f, 12u, 0.03f, 0.05f, 0.12f, 4u, 0.05f}
```

- [ ] **Step 2: Declare and read the seven parameters**

Declare/read:

```text
enable_piecewise_planar_regularization
planar_neighbor_height_tolerance
planar_min_region_size
planar_max_region_mad
planar_max_regularization_residual
planar_max_occlusion_distance
planar_min_occlusion_support
planar_inferred_half_range
```

Read integer counts into temporary `int` values, clamp negative values to zero
before converting to `std::size_t`, and let the processor reject zero or
non-finite runtime settings without changing the map.

- [ ] **Step 3: Integrate the immutable support snapshot**

Update `publishFusedElevationMap()` in this order:

```cpp
grid_map::GridMap fusedMapCopy = fusedMap_;
const grid_map::GridMap planarSupportMap = fusedMapCopy;
scopedLock.unlock();

fillSmallElevationHolesIfEnabled(
    fusedMapCopy, enableSmallHoleFilling_, smallHoleFillingParameters_);
processPiecewisePlanarElevationIfEnabled(
    planarSupportMap, fusedMapCopy,
    enablePiecewisePlanarRegularization_,
    piecewisePlanarParameters_);
fusedMapCopy.add(
    "uncertainty_range",
    fusedMapCopy.get("upper_bound") - fusedMapCopy.get("lower_bound"));
```

This keeps both the persistent fused map and planar support immutable while
allowing small-hole output to remain visible.

- [ ] **Step 4: Enable the tuned profile only for MID360**

Append to `config/robots/fastlio_mid360.yaml`:

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

Do not modify other robot profiles.

- [ ] **Step 5: Compile and run focused regression tests**

Run:

```bash
colcon build --packages-select elevation_mapping \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select elevation_mapping \
  --ctest-args \
  -R 'test_(piecewise_planar_processor|small_hole_filler|scan_cell_selector)' \
  --output-on-failure
colcon test-result --verbose
```

Expected: processor tests, 8 small-hole tests, and 6 scan-cell-selector tests
pass. Record the known unrelated input-source baseline failures separately if
the full package suite is also run.

- [ ] **Step 6: Commit Task 3**

```bash
git add include/elevation_mapping/ElevationMap.hpp \
  src/ElevationMap.cpp \
  config/robots/fastlio_mid360.yaml \
  test/PiecewisePlanarProcessorTest.cpp
git commit -m "feat: enable planar elevation processing for mid360"
```

---

### Task 4: Unfiltered Bag Quality Gate And RViz

**Files:**
- Modify only if measurements require parameter tuning: `config/robots/fastlio_mid360.yaml`

**Interfaces:**
- Consumes: `/fastlio2/body_cloud`, `/fastlio2/lio_odom`, and `/tf` from `/home/guanlin/catkin_ws/rosbag2_2026_07_25-09_22_56`.
- Produces: measured comparison against the current `0.04 m` baseline and a running RViz view on `/elevation_map` in `world`.

- [ ] **Step 1: Stop stale replay/mapping processes and rebuild**

Stop only the ROS 2 bag, elevation-mapping, and RViz processes started for
this task. Then run:

```bash
source /opt/ros/humble/setup.bash
cd /home/guanlin/catkin_ws
colcon build --packages-select elevation_mapping \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Expected: build succeeds.

- [ ] **Step 2: Capture a disabled-feature baseline from the unfiltered bag**

Launch mapping with:

```bash
ros2 launch elevation_mapping elevationMapping_fastlio_launch.py \
  use_sim_time:=true \
  enable_piecewise_planar_regularization:=false
```

Replay:

```bash
ros2 bag play /home/guanlin/catkin_ws/rosbag2_2026_07_25-09_22_56 \
  --clock
```

Using the existing `/tmp/elevation_map_metrics.py`, record finite coverage and
floor median absolute deviation after the mapper reaches steady state.

- [ ] **Step 3: Capture the enabled-feature result from a fresh mapper**

Restart mapping and replay from the beginning with the MID360 profile defaults.
Record the same metrics and inspect inferred band heights. Accept only when:

```text
enabled_floor_MAD <= 0.80 * baseline_floor_MAD
enabled_finite_coverage >= baseline_finite_coverage
```

For each detected platform/stair transition, confirm inferred values cluster
at fitted plane heights and do not create a populated intermediate ramp.

- [ ] **Step 4: Tune only within the approved parameter surface if needed**

If the quality gate fails, change only these profile parameters:

```text
planar_neighbor_height_tolerance
planar_min_region_size
planar_max_region_mad
planar_max_regularization_residual
planar_max_occlusion_distance
planar_min_occlusion_support
planar_inferred_half_range
```

Rebuild only when YAML installation requires it, restart from a fresh mapper,
and replay the same bag segment. Do not change fusion, variance, or scan-cell
selection code during tuning.

- [ ] **Step 5: Launch final RViz visualization**

Leave the accepted mapper running and launch the repository RViz
configuration with fixed frame `world` and map topic `/elevation_map`.
Verify that:

- ground and platform tops are visibly flatter;
- every stair/platform level stays distinct;
- platform edges remain discrete rather than rounded ramps;
- only thin unobserved strips between levels are completed; and
- broad never-observed areas remain unknown.

- [ ] **Step 6: Commit any accepted parameter adjustment**

If Task 4 changed YAML:

```bash
git add config/robots/fastlio_mid360.yaml
git commit -m "tune: refine mid360 planar elevation parameters"
```

If no parameter changed, do not create an empty commit.
