# Multimodal Edge Elevation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a two-mode, spatially aware cell model that prevents upper/lower height switching at step edges while preserving the existing 4 cm `grid_map` interface.

**Architecture:** Build scan-local height modes from all points in each output cell, using a 3x3 in-cell XY occupancy mask to distinguish horizontal surfaces from narrow vertical returns. Match those observations to two persistent modes stored in a private map-aligned state grid, select the public primary height with center/coverage preference and hysteresis, and retain the legacy single-height path behind a feature flag.

**Tech Stack:** ROS 2 Humble, C++14, `grid_map`, PCL, Eigen, GTest, `rosbag2_py`, Python 3.

## Global Constraints

- Keep the public map geometry at 3 m by 3 m with 4 cm resolution.
- Keep `map_frame_id: world`, `robot_base_frame_id: body`, and the existing `elevation` layer contract.
- Store at most two persistent height modes per cell.
- Use a 5 cm initial height-mode separation threshold.
- Require at least three points and two occupied 3x3 micro-bins per accepted mode.
- Require a challenger advantage of one micro-bin for three consecutive scans before switching the primary mode.
- Expire an unobserved secondary mode after 0.5 seconds.
- Do not modify FAST-LIO2, Livox timing, or radar configuration.
- Preserve the current adaptive lower-surface and hole-filling behavior for non-multimodal cells.
- Keep the legacy path available through `enable_multimodal_cells`.
- Use `/home/guanlin/catkin_ws/rosbag2_2026_07_25-09_22_56` as the fixed replay input.
- Acceptance requires at least 80% fewer vertical switches, at least 95% baseline finite-cell coverage, no worse flat-cell 95th-percentile variation, an edge transition no wider than one 4 cm cell, deterministic results across three replays, and no more than 25% CPU increase.

---

## File Structure

- `include/elevation_mapping/MultimodalElevation.hpp`: Pure C++14 observation, persistent-mode, selection, and configuration interfaces.
- `src/MultimodalElevation.cpp`: Point clustering, 3x3 occupancy, mode matching, filtering, expiration, and primary-selection implementation.
- `test/MultimodalElevationTest.cpp`: Pure algorithm tests without ROS publishers or TF.
- `test/ElevationMapMultimodalTest.cpp`: `ElevationMap::add()` integration, circular-buffer movement, legacy-path, and diagnostics tests.
- `include/elevation_mapping/ElevationMap.hpp`: Private state grid, parameters, layer names, and integration helpers.
- `src/ElevationMap.cpp`: Per-scan cell grouping, state-grid encode/decode, multimodal updates, public primary application, diagnostics, and fusion integration.
- `src/ElevationMapping.cpp`: ROS parameter declaration, loading, and validation.
- `config/robots/fastlio_mid360.yaml`: Validated MID360 defaults and final feature enablement.
- `include/elevation_mapping/EdgeAwareMapUtils.hpp`: Hole-fill discontinuity helper declaration.
- `src/EdgeAwareMapUtils.cpp`: Hole-fill rejection around multimodal discontinuities.
- `test/EdgeAwareMapUtilsTest.cpp`: Hole-fill regression tests.
- `scripts/analyze_elevation_bag.py`: Deterministic raw-map switch, coverage, flat-region variation, and edge-width metrics.
- `scripts/run_elevation_bag_regression.sh`: Isolated-domain replay/record orchestration for legacy and candidate runs.
- `CMakeLists.txt`: New source/test targets and installed diagnostic scripts.
- `package.xml`: Runtime dependencies for the installed Python bag analyzer.

---

### Task 1: Preserve the Current Edge-Aware Baseline

**Files:**
- Modify and commit only the already changed baseline files:
  `CMakeLists.txt`,
  `config/robots/fastlio_mid360.yaml`,
  `include/elevation_mapping/ElevationMap.hpp`,
  `include/elevation_mapping/EdgeAwareMapUtils.hpp`,
  `src/ElevationMap.cpp`,
  `src/ElevationMapping.cpp`,
  `src/EdgeAwareMapUtils.cpp`,
  `test/EdgeAwareMapUtilsTest.cpp`,
  `test/ElevationMapAdaptiveTest.cpp`

**Interfaces:**
- Consumes: Current uncommitted adaptive lower-surface and edge-safe hole-fill implementation.
- Produces: A clean, tested baseline commit on which the multimodal feature can be reviewed independently.

- [ ] **Step 1: Verify the existing regression target**

Run:

```bash
cd /home/guanlin/catkin_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR=/tmp/ros_log
colcon build --packages-select elevation_mapping \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
./build/elevation_mapping/test_elevation_mapping_edge_aware_map --gtest_color=no
```

Expected: build exits 0 and all 11 current tests pass, including both mixed-return point orders.

- [ ] **Step 2: Verify the baseline diff is clean**

Run:

```bash
cd /home/guanlin/catkin_ws/src/elevation_mapping
git diff --check
git status --short
```

Expected: no whitespace errors; only the listed baseline files are modified or untracked.

- [ ] **Step 3: Commit the baseline without the plan or unrelated files**

```bash
git add CMakeLists.txt \
  config/robots/fastlio_mid360.yaml \
  include/elevation_mapping/ElevationMap.hpp \
  include/elevation_mapping/EdgeAwareMapUtils.hpp \
  src/ElevationMap.cpp \
  src/ElevationMapping.cpp \
  src/EdgeAwareMapUtils.cpp \
  test/EdgeAwareMapUtilsTest.cpp \
  test/ElevationMapAdaptiveTest.cpp
git commit -m "feat: improve edge-aware elevation recovery"
```

Expected: one commit containing the previously verified edge-aware baseline.

---

### Task 2: Build Scan-Local Height Modes

**Files:**
- Create: `include/elevation_mapping/MultimodalElevation.hpp`
- Create: `src/MultimodalElevation.cpp`
- Create: `test/MultimodalElevationTest.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: C++14 standard library only.
- Produces:

```cpp
namespace elevation_mapping {

struct MultimodalConfig {
  float modeSeparation{0.05f};
  std::size_t minPoints{3};
  std::size_t minBins{2};
  std::size_t switchMarginBins{1};
  int switchConfirmations{3};
  double staleTimeout{0.5};
};

struct MultimodalPoint {
  float x;
  float y;
  float height;
  float variance;
  float color;
};

struct SurfaceObservation {
  float height;
  float variance;
  float color;
  float lowestPointHeight;
  std::size_t pointCount;
  std::uint16_t xyMask;
  bool centerOccupied;
};

struct CellObservation {
  std::array<SurfaceObservation, 2> modes;
  std::size_t modeCount;
};

bool isValidMultimodalConfig(const MultimodalConfig& config);
std::size_t countOccupiedBins(std::uint16_t mask);
CellObservation buildCellObservation(
    const std::vector<MultimodalPoint>& points, float cellCenterX,
    float cellCenterY, float cellResolution, const MultimodalConfig& config);

}  // namespace elevation_mapping
```

- [ ] **Step 1: Add failing order, center, coverage, and vertical-face tests**

Add these test cases to `test/MultimodalElevationTest.cpp`:

```cpp
TEST(MultimodalObservation, IsIndependentOfPointOrder) {
  const auto highFirst = buildCellObservation(
      makeTwoSurfacePoints(true), 0.0f, 0.0f, 0.04f, config());
  const auto lowFirst = buildCellObservation(
      makeTwoSurfacePoints(false), 0.0f, 0.0f, 0.04f, config());
  ASSERT_EQ(highFirst.modeCount, 2u);
  ASSERT_EQ(lowFirst.modeCount, 2u);
  EXPECT_NEAR(highFirst.modes[0].height, lowFirst.modes[0].height, 1e-5f);
  EXPECT_NEAR(highFirst.modes[1].height, lowFirst.modes[1].height, 1e-5f);
}

TEST(MultimodalObservation, MarksModeOccupyingCenterBin) {
  const auto observation = buildCellObservation(
      makeCenterSupportedUpperSurface(), 0.0f, 0.0f, 0.04f, config());
  ASSERT_EQ(observation.modeCount, 2u);
  EXPECT_FALSE(observation.modes[0].centerOccupied);
  EXPECT_TRUE(observation.modes[1].centerOccupied);
}

TEST(MultimodalObservation, KeepsTwoModesWithLargestHorizontalCoverage) {
  const auto observation = buildCellObservation(
      makeUpperLowerAndVerticalFacePoints(), 0.0f, 0.0f, 0.04f, config());
  ASSERT_EQ(observation.modeCount, 2u);
  EXPECT_NEAR(observation.modes[0].height, -0.30f, 0.01f);
  EXPECT_NEAR(observation.modes[1].height, 0.20f, 0.01f);
}
```

Define the test-local helpers used above in the same file:

```cpp
MultimodalConfig config();  // Returns the defaults shown in the interface.
MultimodalPoint point(float x, float y, float z);
std::vector<MultimodalPoint> makeTwoSurfacePoints(bool highFirst);
std::vector<MultimodalPoint> makeCenterSupportedUpperSurface();
std::vector<MultimodalPoint> makeUpperLowerAndVerticalFacePoints();
```

`point()` sets variance to `1e-4f` and color to `0.0f`.
`makeTwoSurfacePoints()` uses lower XY positions
`(-0.018, -0.018)`, `(-0.006, -0.018)`, `(-0.018, -0.006)` at `-0.30f`
and upper positions `(0.006, 0.006)`, `(0.018, 0.006)`,
`(0.006, 0.018)` at `0.20f`; `highFirst` changes only insertion order.
The center-support fixture uses lower corner bins 0, 2, and 6, and upper
positions `(0, 0)`, `(0.018, 0)`, `(0, 0.018)`, so only the upper group owns
bit 4. The vertical-face fixture adds points
`(0.018, -0.018, -0.15)`, `(0.018, -0.018, -0.14)`, and
`(0.018, -0.018, -0.13)`, which form a one-bin group rejected for insufficient
horizontal support. Assert every helper point remains inside the target 4 cm
cell.

Also test that invalid config values, non-finite points, fewer than three points,
and fewer than two occupied micro-bins produce no accepted mode.

- [ ] **Step 2: Register the new source and test, then verify RED**

Add `src/MultimodalElevation.cpp` to `${PROJECT_NAME}_library` and
`test/MultimodalElevationTest.cpp` to
`test_${PROJECT_NAME}_edge_aware_map`.

Run:

```bash
cd /home/guanlin/catkin_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select elevation_mapping \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
```

Expected: compilation fails because the declared multimodal functions are not implemented.

- [ ] **Step 3: Implement deterministic clustering and 3x3 occupancy**

Implement these rules in `src/MultimodalElevation.cpp`:

```cpp
// 1. Reject non-finite points.
// 2. Sort accepted point indices by height.
// 3. Start a new provisional group when the current height minus the first
//    height in the group exceeds config.modeSeparation. Bounding total group
//    span prevents a continuous vertical-face return from chaining the lower
//    and upper horizontal surfaces into one group.
// 4. Convert local x/y offsets to clamped [0, 2] micro-bin coordinates:
//    bin = floor((offset / cellResolution + 0.5f) * 3.0f).
// 5. Set bit (row * 3 + column) in a uint16_t mask.
// 6. Reject groups below config.minPoints or config.minBins.
// 7. Rank remaining groups by occupied-bin count, then point count.
// 8. Break remaining rank ties by lower weighted variance, then lower height.
// 9. Keep at most two and return them sorted by height.
```

Compute observation height and variance using inverse-variance weighting with
`variance >= 1e-9f`. Set `centerOccupied` from mask bit 4. Choose color from the
point nearest the weighted mean height. Set `lowestPointHeight` to the minimum
`height + 3 * sqrt(variance)` in the accepted group so visibility cleanup keeps
the same conservative metadata as the legacy point loop.

- [ ] **Step 4: Run the pure tests**

```bash
cd /home/guanlin/catkin_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR=/tmp/ros_log
colcon build --packages-select elevation_mapping \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
./build/elevation_mapping/test_elevation_mapping_edge_aware_map \
  --gtest_filter='MultimodalObservation.*' --gtest_color=no
```

Expected: every `MultimodalObservation` test passes.

- [ ] **Step 5: Commit the scan-local observation component**

```bash
cd /home/guanlin/catkin_ws/src/elevation_mapping
git add CMakeLists.txt \
  include/elevation_mapping/MultimodalElevation.hpp \
  src/MultimodalElevation.cpp \
  test/MultimodalElevationTest.cpp
git commit -m "feat: build spatially supported height modes"
```

---

### Task 3: Add Persistent Modes and Primary-Selection Hysteresis

**Files:**
- Modify: `include/elevation_mapping/MultimodalElevation.hpp`
- Modify: `src/MultimodalElevation.cpp`
- Modify: `test/MultimodalElevationTest.cpp`

**Interfaces:**
- Consumes: `CellObservation` and `MultimodalConfig` from Task 2.
- Produces:

```cpp
struct SurfaceModeState {
  bool valid{false};
  float height{std::numeric_limits<float>::quiet_NaN()};
  float variance{std::numeric_limits<float>::quiet_NaN()};
  float color{std::numeric_limits<float>::quiet_NaN()};
  float lowestPointHeight{std::numeric_limits<float>::quiet_NaN()};
  float confidence{0.0f};
  double lastSeen{std::numeric_limits<double>::quiet_NaN()};
  std::size_t coverageBins{0};
  std::size_t consecutiveObservations{0};
  bool centerOccupied{false};
};

struct MultimodalCellState {
  std::array<SurfaceModeState, 2> modes;
  int primaryIndex{-1};
  int challengerIndex{-1};
  int challengerCount{0};
};

struct MultimodalUpdateResult {
  bool handled{false};
  bool primarySwitched{false};
  SurfaceModeState primary;
  SurfaceModeState secondary;
  std::size_t modeCount{0};
};

MultimodalUpdateResult updateMultimodalCell(
    MultimodalCellState& state, const CellObservation& observation,
    double timestamp, const MultimodalConfig& config);
std::size_t countValidModes(const MultimodalCellState& state);
bool expireStaleSecondaryMode(
    MultimodalCellState& state, double timestamp,
    const MultimodalConfig& config);
```

- [ ] **Step 1: Add failing persistent-state tests**

Add:

```cpp
TEST(MultimodalSelection, PrefersCenterSupportedMode) {
  MultimodalCellState state;
  const auto result = updateMultimodalCell(
      state, observationWithCenterOnUpper(), 0.0, config());
  ASSERT_TRUE(result.handled);
  EXPECT_NEAR(result.primary.height, 0.20f, 0.01f);
}

TEST(MultimodalSelection, RequiresThreeConsecutiveWinsToSwitch) {
  MultimodalCellState state = lowerPrimaryState();
  const auto challenger = upperCoverageWinner();
  EXPECT_FALSE(updateMultimodalCell(state, challenger, 0.1, config()).primarySwitched);
  EXPECT_FALSE(updateMultimodalCell(state, challenger, 0.2, config()).primarySwitched);
  const auto third = updateMultimodalCell(state, challenger, 0.3, config());
  EXPECT_TRUE(third.primarySwitched);
  EXPECT_NEAR(third.primary.height, 0.20f, 0.01f);
}

TEST(MultimodalSelection, RetainsIncumbentWhenSpatialSupportTies) {
  MultimodalCellState state = lowerPrimaryState();
  const auto result = updateMultimodalCell(
      state, equalCoverageObservation(), 0.1, config());
  EXPECT_NEAR(result.primary.height, -0.30f, 0.01f);
}

TEST(MultimodalSelection, ExpiresUnobservedSecondaryMode) {
  MultimodalCellState state = twoModeState();
  EXPECT_TRUE(expireStaleSecondaryMode(state, 0.7, config()));
  EXPECT_TRUE(state.modes[state.primaryIndex].valid);
  EXPECT_EQ(countValidModes(state), 1u);
}
```

Also add tests proving mode matching is height-based when observation order changes
and that a single-mode observation with no existing multimodal state returns
`handled == false`. Add `DoesNotExpireUnobservedPrimary` to prove the stale
sweep preserves the published primary mode.

Define all selection fixtures in the same test file using these constructors:

```cpp
SurfaceObservation surface(float height, std::uint16_t mask,
                           bool centerOccupied);
CellObservation observation(
    std::initializer_list<SurfaceObservation> surfaces);
MultimodalCellState lowerPrimaryState();
MultimodalCellState twoModeState();
CellObservation observationWithCenterOnUpper();
CellObservation upperCoverageWinner();
CellObservation equalCoverageObservation();
CellObservation lowerOnlyObservation();
```

`surface()` sets variance to `1e-4f`, point count to three, `xyMask` to `mask`,
color to zero, and lowest point to `height + 0.03f`.
`lowerPrimaryState()` contains valid modes at `-0.30f` and `0.20f`, selects the
lower slot, and gives both modes a finite last-seen time. Center and coverage
fixtures differ only in their masks: use bits 0 and 1 for the lower mode;
use bits 4 and 5 for center-supported upper; use bits 2, 4, 5, and 8 for the
four-bin upper coverage winner; and use bits 7 and 8 for the equal-coverage,
non-center upper mode. `lowerOnlyObservation()` contains only the lower mode.

- [ ] **Step 2: Run tests to verify RED**

```bash
cd /home/guanlin/catkin_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select elevation_mapping \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
```

Expected: link failures because persistent updates are declared and tested but
not implemented.

- [ ] **Step 3: Implement mode matching and selection**

Implement:

```cpp
// Match observations one-to-one to valid state modes by the smallest absolute
// height difference not exceeding config.modeSeparation. Unmatched accepted
// observations fill an invalid slot or replace only an expired secondary.
//
// Fuse matched height/variance with a scalar Kalman update.
// Copy color and lowestPointHeight from the matched current observation.
// confidence = min(1.0f, confidence + 0.25f).
// coverageBins and centerOccupied come from the current observation.
// Increment consecutiveObservations for a matched mode and reset it to one
// when filling a new or expired slot; reset it to zero when unobserved.
// Decay unobserved confidence by 0.25f and expire when
// timestamp - lastSeen > config.staleTimeout.
//
// Ranking: centerOccupied first, then coverageBins.
// A challenger wins only when its coverage exceeds the incumbent by
// config.switchMarginBins, or it alone occupies the center bin.
// Increment challengerCount once per distinct timestamp.
// Switch at config.switchConfirmations; reset on a tie/incumbent win.
```

Keep mode slots stable by height matching; do not sort persistent slots after update.
`expireStaleSecondaryMode()` clears only a valid non-primary slot older than
`staleTimeout`, and also clears challenger state if it referenced that slot.

- [ ] **Step 4: Run all pure multimodal tests**

```bash
cd /home/guanlin/catkin_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select elevation_mapping \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
./build/elevation_mapping/test_elevation_mapping_edge_aware_map \
  --gtest_filter='Multimodal*.*' --gtest_color=no
```

Expected: all observation and selection tests pass.

- [ ] **Step 5: Commit persistent selection**

```bash
git add include/elevation_mapping/MultimodalElevation.hpp \
  src/MultimodalElevation.cpp \
  test/MultimodalElevationTest.cpp
git commit -m "feat: stabilize multimodal primary height"
```

---

### Task 4: Integrate Multimodal State into `ElevationMap`

**Files:**
- Create: `test/ElevationMapMultimodalTest.cpp`
- Modify: `include/elevation_mapping/ElevationMap.hpp`
- Modify: `src/ElevationMap.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: `buildCellObservation()` and `updateMultimodalCell()` from Tasks 2-3.
- Produces:
  - private `grid_map::GridMap multimodalStateMap_`;
  - legacy behavior when `enableMultimodalCells_ == false`;
  - raw diagnostic layers `secondary_elevation`, `height_mode_count`,
    `height_mode_separation`, and `primary_mode_confidence`.

- [ ] **Step 1: Add failing `ElevationMap::add()` tests**

Create a fixture patterned after `ElevationMapMixedReturnTest` and configure a
0.6 m map with 4 cm resolution.

Declare the fixture as `friend class ElevationMapMultimodalTest` in
`ElevationMap.hpp`. Define these helpers in the fixture:

```cpp
void configureMultimodal(bool enabled);
void addScan(const std::vector<TestPoint>& points, double seconds);
void initializeUpperAndLowerSurfaces();
void initializeTwoModesAt(double x, double y);
void addAmbiguousScan(bool upperCenterSupported, double seconds);
void addAmbiguousScanAt(double x, double y, double seconds);
void addSingleSurfaceScan(float height, double seconds);
float rawValueAt(const std::string& layer, double x, double y) const;
float elevationAtTarget() const;
float secondaryElevationAtTarget() const;
float elevationAt(double x, double y) const;
```

`addScan()` uses the fixture's node-clock base time, variance `1e-4f`, black
points, and `Eigen::Affine3d::Identity()`. The two-surface helpers use the same
4 cm-local point layouts from Task 2. Implement `elevationAtTarget()`,
`secondaryElevationAtTarget()`, and `elevationAt(x, y)` as thin calls to
`rawValueAt()`.

Add:

```cpp
TEST_F(ElevationMapMultimodalTest, KeepsCenterSupportedHeightAcrossViewChanges) {
  initializeUpperAndLowerSurfaces();
  addAmbiguousScan(/* upper_center_supported = */ true, 0.1);
  addAmbiguousScan(/* upper_center_supported = */ false, 0.2);
  addAmbiguousScan(/* upper_center_supported = */ true, 0.3);
  EXPECT_NEAR(elevationAtTarget(), 0.20f, 0.02f);
  EXPECT_NEAR(secondaryElevationAtTarget(), -0.30f, 0.02f);
}

TEST_F(ElevationMapMultimodalTest, UsesLegacyPathWhenFeatureDisabled) {
  configureMultimodal(false);
  addSingleSurfaceScan(-0.10f, 0.1);
  EXPECT_NEAR(elevationAtTarget(), -0.10f, 0.02f);
  EXPECT_TRUE(std::isnan(secondaryElevationAtTarget()));
}

TEST_F(ElevationMapMultimodalTest, MovesModeStateWithCircularBuffer) {
  initializeTwoModesAt(0.0, 0.0);
  map_.move(Eigen::Vector2d(0.08, 0.08));
  addAmbiguousScanAt(0.0, 0.0, 0.2);
  EXPECT_NEAR(elevationAt(0.0, 0.0), 0.20f, 0.02f);
}
```

Also assert `height_mode_count == 2`, separation is approximately 0.5 m, and
`primary_mode_confidence` is finite.

- [ ] **Step 2: Register the integration test and verify RED**

Add `test/ElevationMapMultimodalTest.cpp` to
`test_${PROJECT_NAME}_edge_aware_map`, then rebuild:

```bash
cd /home/guanlin/catkin_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select elevation_mapping \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
```

Expected: compilation fails because `ElevationMap` has no multimodal state or parameters.

- [ ] **Step 3: Add the private state grid and lifecycle handling**

Initialize `multimodalStateMap_` with these non-published layers:

```text
mode0_elevation mode0_variance mode0_color mode0_lowest mode0_confidence mode0_time
mode0_coverage mode0_observations mode0_center
mode1_elevation mode1_variance mode1_color mode1_lowest mode1_confidence mode1_time
mode1_coverage mode1_observations mode1_center
primary_mode_index challenger_mode_index challenger_count
```

Add the four diagnostic layers to `rawMap_`. In `setGeometry()`, `clear()`, and
`move()`, apply the same geometry, clear, and position operation to
`multimodalStateMap_`. Newly exposed circular-buffer regions must contain NaN
heights/times, zero confidence/coverage/count, and `-1` selection indices.
At the start of every enabled scan, sweep the bounded private state map with
`expireStaleSecondaryMode()` so a cell does not need a new point for its stale
secondary to disappear.

- [ ] **Step 4: Group scan points by cell and apply handled results once**

At the beginning of `ElevationMap::add()`:

```cpp
std::vector<std::vector<MultimodalPoint>> pointsByCell(
    static_cast<std::size_t>(rawMap_.getSize().prod()));

// For each point inside rawMap_:
// key = index(0) * rawMap_.getSize()(1) + index(1)
// append x, y, z, clamped variance, and packed color.
```

Guard allocation, state sweeping, and point grouping with
`enableMultimodalCells_`; when false, enter the existing point/adaptive path
without any multimodal per-scan work.

For each occupied cell:

1. Build `CellObservation`.
2. Decode `MultimodalCellState` from `multimodalStateMap_`.
3. Call `updateMultimodalCell()`.
4. Encode updated state.
5. Mark handled cells in a scan-local mask.
6. After legacy point processing, write each handled primary exactly once to
   elevation, variance, time, horizontal variance, color, and lowest-scan metadata.
7. Populate the four diagnostic layers.

For lowest-scan metadata, write `primary.lowestPointHeight` and the current
`transformationSensorToMap.translation()`. Reset lower-point and adaptive-lower
candidate layers exactly as the existing accepted-lower-surface branch does.
Diagnostics use count `0.0f` with all other values NaN for no state, count
`1.0f` with secondary and separation NaN for one mode, and count `2.0f` with a
finite secondary and absolute separation for two modes. Refresh diagnostics
when the stale sweep changes a state as well as when an occupied cell updates.

The legacy point loop must skip handled cells. This ensures high/low point order
cannot overwrite the selected primary. Cells returning `handled == false`
continue through the current adaptive/legacy path unchanged.
All adaptive-lower preprocessing and confirmed-surface writeback loops must
also skip handled cells. Apply handled primary results after both the legacy
point loop and adaptive writeback so no later branch can overwrite them.

- [ ] **Step 5: Run integration and existing regressions**

```bash
cd /home/guanlin/catkin_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select elevation_mapping \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
./build/elevation_mapping/test_elevation_mapping_edge_aware_map \
  --gtest_color=no
```

Expected: all prior 11 tests plus the new multimodal tests pass.

- [ ] **Step 6: Commit the map integration**

```bash
git add CMakeLists.txt \
  include/elevation_mapping/ElevationMap.hpp \
  src/ElevationMap.cpp \
  test/ElevationMapMultimodalTest.cpp
git commit -m "feat: integrate multimodal elevation cells"
```

---

### Task 5: Add Parameters, Diagnostics Fusion, and Edge-Safe Hole Filling

**Files:**
- Modify: `src/ElevationMapping.cpp`
- Modify: `include/elevation_mapping/ElevationMap.hpp`
- Modify: `src/ElevationMap.cpp`
- Modify: `include/elevation_mapping/EdgeAwareMapUtils.hpp`
- Modify: `src/EdgeAwareMapUtils.cpp`
- Modify: `test/EdgeAwareMapUtilsTest.cpp`
- Modify: `test/ElevationMapMultimodalTest.cpp`
- Modify: `config/robots/fastlio_mid360.yaml`

**Interfaces:**
- Consumes: `MultimodalConfig` and raw diagnostic layers.
- Produces: Validated ROS parameters, fused diagnostic layers, and hole filling
  that refuses to bridge multimodal discontinuities.

- [ ] **Step 1: Add failing configuration and hole-fill tests**

Add tests:

```cpp
TEST(MultimodalConfig, RejectsUnsafeValues) {
  auto c = config();
  c.modeSeparation = 0.0f;
  EXPECT_FALSE(isValidMultimodalConfig(c));
  c = config();
  c.minPoints = 2;
  EXPECT_FALSE(isValidMultimodalConfig(c));
  c = config();
  c.switchConfirmations = 1;
  EXPECT_FALSE(isValidMultimodalConfig(c));
}

TEST(EdgeAwareMapUtils, HoleFillRejectsMultimodalEdgeNeighbors) {
  auto map = makeMapWithModeLayers();
  setSurroundingFlatHeights(map, 0.20f);
  markTwoModes(map, 0.04, 0.0, 0.20f, -0.30f);
  grid_map::Index source;
  EXPECT_FALSE(findEdgeSafeHoleFillSource(
      map, "elevation", "height_mode_count", "height_mode_separation",
      centerIndex(map), 0.12, 0.05, 4, source));
}
```

Extend the existing test-local `makeMap()` helper to optionally add
`height_mode_count` and `height_mode_separation`, or define
`makeMapWithModeLayers()` as a call to that helper. Define
`setSurroundingFlatHeights()` with the four axis-adjacent cells,
`markTwoModes()` by setting count to `2.0f` and separation to
`abs(primary - secondary)`, and `centerIndex()` as `indexAt(map, 0, 0)`.

In the integration test, call `fuseAll()` and assert the four diagnostic values
are copied to the fused map at the target cell.

- [ ] **Step 2: Run targeted tests to verify RED**

```bash
cd /home/guanlin/catkin_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select elevation_mapping \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
```

Expected: compile failures for the new hole-fill interface or test access to
diagnostic layers that are not implemented yet.

- [ ] **Step 3: Declare and validate ROS parameters**

Declare, read, and validate:

```yaml
enable_multimodal_cells: false
multimodal_height_separation: 0.05
multimodal_min_points: 3
multimodal_min_xy_bins: 2
multimodal_switch_margin_bins: 1
multimodal_switch_confirmations: 3
multimodal_stale_timeout: 0.5
```

Validation must require: positive separation, `min_points >= 3`,
`2 <= min_xy_bins <= 9`, `1 <= switch_margin_bins <= 9`,
`switch_confirmations >= 2`, and positive stale timeout. On invalid values,
log an error and disable multimodal cells rather than starting with partial
configuration.

Keep `enable_multimodal_cells: false` in `fastlio_mid360.yaml` until Task 7.

- [ ] **Step 4: Fuse diagnostics and protect hole filling**

Add the four diagnostics to `fusedMap_`. During `fuse()`, copy diagnostics from
the center raw cell selected for the fused estimate.

Extend `findEdgeSafeHoleFillSource()` with mode-count and separation layer names.
Reject filling when any supporting neighbor has `height_mode_count > 1.5f` and
`height_mode_separation > heightThreshold`. Update all call sites.

- [ ] **Step 5: Run the complete target and build**

```bash
cd /home/guanlin/catkin_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR=/tmp/ros_log
colcon build --packages-select elevation_mapping \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
./build/elevation_mapping/test_elevation_mapping_edge_aware_map --gtest_color=no
```

Expected: build exits 0; all edge-aware, adaptive, multimodal, integration, and
hole-fill tests pass.

- [ ] **Step 6: Commit parameters and fused diagnostics**

```bash
git add src/ElevationMapping.cpp \
  include/elevation_mapping/ElevationMap.hpp \
  src/ElevationMap.cpp \
  include/elevation_mapping/EdgeAwareMapUtils.hpp \
  src/EdgeAwareMapUtils.cpp \
  test/EdgeAwareMapUtilsTest.cpp \
  test/ElevationMapMultimodalTest.cpp \
  config/robots/fastlio_mid360.yaml
git commit -m "feat: configure multimodal edge diagnostics"
```

---

### Task 6: Add Deterministic Bag Replay Metrics

**Files:**
- Create: `scripts/analyze_elevation_bag.py`
- Create: `scripts/run_elevation_bag_regression.sh`
- Modify: `CMakeLists.txt`
- Modify: `package.xml`

**Interfaces:**
- Consumes: Input bag path, installed elevation mapping parameters, and recorded
  `/elevation_map_raw_post`.
- Produces:
  - one output bag per run;
  - one JSON metrics file per run;
  - mapper user-plus-system CPU seconds for the fixed-duration replay;
  - a `--compare` report for one baseline and three candidates;
  - exit status 0 only when the replay and analysis complete.

- [ ] **Step 1: Write analyzer self-tests first**

Create `scripts/analyze_elevation_bag.py` with argument parsing and a
`--self-test` branch that calls these not-yet-defined functions:

```python
times = [0.0, 0.1, 0.2, 0.3]
stable = [0.20, 0.21, 0.20, 0.20]
switching = [0.20, -0.30, 0.20, -0.30]
assert count_vertical_switches(times, stable, threshold=0.08,
                               return_window=1.0) == 0
assert count_vertical_switches(times, switching, threshold=0.08,
                               return_window=1.0) == 2
assert finite_coverage([1.0, float("nan"), 2.0]) == 2 / 3
```

Define a switch episode as a transition from mode A to mode B followed by a
return to A within `return_window`; overlapping A-B-A and B-A-B episodes count
separately. Add a self-test for a transition with no return and expect zero.

Run:

```bash
source /opt/ros/humble/setup.bash
source /home/guanlin/catkin_ws/install/setup.bash
python3 scripts/analyze_elevation_bag.py --self-test
```

Expected: failure because analyzer functions are absent. Implement them in
Step 2, then rerun this command in Step 4 and require exit 0.

- [ ] **Step 2: Implement raw-map decoding and metrics**

Use `rosbag2_py.SequentialReader` and
`rclpy.serialization.deserialize_message` for `grid_map_msgs/msg/GridMap`.

Decode each layer using its `Float32MultiArray` dimensions plus
`outer_start_index` and `inner_start_index`. Convert logical indices to world
cell coordinates using:

```python
x = center_x + 0.5 * length_x - (row + 0.5) * resolution
y = center_y + 0.5 * length_y - (column + 0.5) * resolution
world_key = (round(x / resolution), round(y / resolution))
```

For every world key, retain timestamped `elevation`, `secondary_elevation`,
`height_mode_count`, and `height_mode_separation`. Compute:

- edge keys as cells that, in any frame, have a finite 3x3-neighbor height range
  above 8 cm or a `height_mode_separation` above 8 cm;
- vertical return switches over 8 cm within one second, restricted to edge keys;
- mean finite-cell coverage per frame;
- 95th-percentile consecutive variation for cells whose local neighbor range
  remains below 5 cm;
- multimodal-cell count and separation distribution;
- edge transition width: for each two-mode cell, select x or y from the larger
  opposite-neighbor primary-height difference, then count contiguous cells on
  that axis with height strictly between `low + 0.02` and `high - 0.02`; report
  the 95th percentile in 4 cm cells.

Print JSON and support:

```text
analyze_elevation_bag.py BAG --cpu-seconds-file FILE --output METRICS
analyze_elevation_bag.py --compare BASE CANDIDATE1 CANDIDATE2 CANDIDATE3
```

The first form writes scalar metrics, sorted edge and flat key lists,
per-world-key switch counts, and per-world-key temporal variation. The
comparison form uses the union of all four edge-key lists when summing switches
and the baseline flat-key list when calculating candidate flat variation. It
computes candidate/baseline ratios, excludes CPU from the determinism
comparison, requires all non-CPU candidate metrics and sorted per-key data to
agree within `1e-6`, prints the six Task 7 acceptance fields, and exits nonzero
if any threshold fails.

Use the arithmetic mean of the three candidate scalar metrics for acceptance:

```python
vertical_switch_reduction = 1.0 - candidate_switches / baseline_switches
finite_coverage_ratio = candidate_coverage / baseline_coverage
flat_variation_ratio = candidate_flat_p95 / baseline_flat_p95
cpu_increase_ratio = candidate_cpu_seconds / baseline_cpu_seconds - 1.0
edge_transition_width_cells = max(candidate_edge_width_p95_values)
```

Treat a zero baseline denominator as passing only when the corresponding
candidate value is also zero; otherwise fail with an explicit diagnostic.
Parse `mapper_time.txt` as two whitespace-separated `%U %S` numbers from
`/usr/bin/time -f "%U %S"` and store their sum as `mapper_cpu_seconds`.

- [ ] **Step 3: Implement isolated replay orchestration**

`scripts/run_elevation_bag_regression.sh` must:

```bash
set -euo pipefail
# Arguments: BAG OUTPUT_DIR FEATURE_ENABLED ROS_DOMAIN_ID
# Refuse an existing OUTPUT_DIR.
# Source ROS and workspace setup.
# Create OUTPUT_DIR, and resolve the installed package prefix.
# Start elevation_mapping via /usr/bin/time without RViz:
# ros2 run elevation_mapping elevation_mapping --ros-args
#   --params-file <share>/config/robots/fastlio_mid360.yaml
#   --params-file <share>/config/postprocessing/postprocessor_pipeline.yaml
#   -p enable_multimodal_cells:=FEATURE_ENABLED
# Write time's %U + %S values to OUTPUT_DIR/mapper_time.txt.
# Record /elevation_map_raw_post and /elevation_map to OUTPUT_DIR/maps.
# Wait up to 15 seconds for the mapper's /fastlio2/body_cloud subscription and
# the recorder's /elevation_map_raw_post subscription; fail if either is absent.
# Play BAG at rate 1.0 and wait for completion.
# Stop recorder and mapper, accepting the mapper's known SIGINT shutdown code.
# Run analyze_elevation_bag.py on OUTPUT_DIR/maps with mapper_time.txt and write
# OUTPUT_DIR/metrics.json.
```

Use a caller-supplied ROS domain so live radar topics cannot enter the replay.
Install signal traps before starting children so an interrupted run stops the
player, recorder, and mapper and leaves no isolated-domain processes behind.

- [ ] **Step 4: Install scripts and run self-test**

Add:

```cmake
install(PROGRAMS
  scripts/analyze_elevation_bag.py
  scripts/run_elevation_bag_regression.sh
  DESTINATION lib/${PROJECT_NAME}
)
```

Add runtime dependencies for `rclpy`, `rosbag2_py`, and `rosidl_runtime_py` to
`package.xml`; `grid_map_msgs` is already declared.

Run the analyzer self-test and `bash -n` on the shell script. Expected: both exit 0.
Then build `elevation_mapping` once to verify the install rule and package
dependencies:

```bash
cd /home/guanlin/catkin_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select elevation_mapping \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
```

- [ ] **Step 5: Commit replay tooling**

```bash
git add CMakeLists.txt package.xml \
  scripts/analyze_elevation_bag.py \
  scripts/run_elevation_bag_regression.sh
git commit -m "test: add elevation bag regression metrics"
```

---

### Task 7: Run A/B Replays, Tune, and Enable MID360 Multimodal Cells

**Files:**
- Modify: `config/robots/fastlio_mid360.yaml`
- Modify only if metrics require a tested correction:
  `src/MultimodalElevation.cpp`,
  `test/MultimodalElevationTest.cpp`,
  `src/ElevationMap.cpp`,
  `test/ElevationMapMultimodalTest.cpp`

**Interfaces:**
- Consumes: Feature flag, replay scripts, and acceptance thresholds.
- Produces: Validated enabled MID360 configuration and final evidence.

- [ ] **Step 1: Build and install the candidate**

```bash
cd /home/guanlin/catkin_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR=/tmp/ros_log
colcon build --packages-select elevation_mapping \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
./build/elevation_mapping/test_elevation_mapping_edge_aware_map --gtest_color=no
```

Expected: build and complete target pass.

- [ ] **Step 2: Produce one baseline and three candidate runs**

```bash
cd /home/guanlin/catkin_ws/src/elevation_mapping
scripts/run_elevation_bag_regression.sh \
  /home/guanlin/catkin_ws/rosbag2_2026_07_25-09_22_56 \
  /tmp/elevation_baseline_092256 false 77
scripts/run_elevation_bag_regression.sh \
  /home/guanlin/catkin_ws/rosbag2_2026_07_25-09_22_56 \
  /tmp/elevation_candidate_092256_1 true 78
scripts/run_elevation_bag_regression.sh \
  /home/guanlin/catkin_ws/rosbag2_2026_07_25-09_22_56 \
  /tmp/elevation_candidate_092256_2 true 79
scripts/run_elevation_bag_regression.sh \
  /home/guanlin/catkin_ws/rosbag2_2026_07_25-09_22_56 \
  /tmp/elevation_candidate_092256_3 true 80
```

Expected: four metrics JSON files and no replay connected to the live ROS domain.

- [ ] **Step 3: Compare acceptance metrics**

Run:

```bash
python3 scripts/analyze_elevation_bag.py \
  --compare /tmp/elevation_baseline_092256/metrics.json \
  /tmp/elevation_candidate_092256_1/metrics.json \
  /tmp/elevation_candidate_092256_2/metrics.json \
  /tmp/elevation_candidate_092256_3/metrics.json
```

Expected:

```text
vertical_switch_reduction >= 0.80
finite_coverage_ratio >= 0.95
flat_variation_ratio <= 1.00
edge_transition_width_cells <= 1
candidate_runs_identical = true
cpu_increase_ratio <= 0.25
```

If a metric fails, add a failing unit or integration test that reproduces the
specific failure before changing one algorithm parameter or branch. Rebuild and
repeat all four replays; do not tune multiple variables in one iteration.

- [ ] **Step 4: Enable the validated MID360 feature**

Change only:

```yaml
enable_multimodal_cells: true
```

Keep the validated defaults from Task 5 unchanged unless the A/B results and a
new test justify a different value.

- [ ] **Step 5: Run final build, tests, diff, and one enabled replay**

```bash
cd /home/guanlin/catkin_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR=/tmp/ros_log
colcon build --packages-select elevation_mapping \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
./build/elevation_mapping/test_elevation_mapping_edge_aware_map --gtest_color=no
git -C src/elevation_mapping diff --check
```

Run one final enabled replay in a fresh output directory and confirm the same
acceptance result.

- [ ] **Step 6: Commit final enablement**

```bash
cd /home/guanlin/catkin_ws/src/elevation_mapping
git add config/robots/fastlio_mid360.yaml
git commit -m "feat: enable multimodal MID360 elevation cells"
```

- [ ] **Step 7: Restart the live mapping stack only after replay validation**

Keep the Livox driver running. Restart FAST-LIO2 only if its current state is
not publishing stable `/fastlio2/lio_odom`; otherwise restart only elevation
mapping and its RViz launch. Verify:

```bash
timeout 8 ros2 topic hz /fastlio2/body_cloud
timeout 8 ros2 topic hz /elevation_map
```

Expected: body cloud approximately 10 Hz, elevation map approximately 20 Hz,
no sustained TF errors, and finite `world`-frame map cells.
