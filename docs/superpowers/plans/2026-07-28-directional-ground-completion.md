# Directional Ground Completion Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fill observed high-to-low platform backside gaps with the local lower-plane height while preserving conservative behavior for ascending and ambiguous views.

**Architecture:** Extend the publication-only `PiecewisePlanarProcessor` with a second directional completion pass after existing conservative completion. The new pass uses the rolling map center as observer XY, nearest physical support positions for each accepted plane, and bounded high-to-low corridor geometry. Add two opt-in ROS parameters and enable them only for the MID360 profile.

**Tech Stack:** C++17, Eigen, grid_map_core, ROS 2 Humble parameters, GoogleTest, colcon.

## Global Constraints

- Do not change the persistent raw map, fused map, Kalman fusion, scan-cell selector, sensor variance, or point-cloud input.
- Use the immutable pre-small-hole fused snapshot as all planar support.
- Keep existing conservative completion at `0.10 m` unchanged.
- Ignore observer Z; use the rolling map center as observer XY.
- Directional order must be `observer -> high support -> unknown cell -> observed low support`.
- Require both high and low accepted planes; never invent an unobserved lower height.
- Limit high-to-low support width to `0.40 m`.
- Assign a directionally completed cell to the selected local lower-plane prediction, never an average or nearest high plane.
- Select local plane pairs independently so multiple platforms and stair levels remain separate.
- Keep directional completion disabled by default and enable it only in `fastlio_mid360.yaml`.
- Operate only on the publication copy; inferred cells cannot become support in the same pass.

---

### Task 1: Directional Corridor Geometry And Lower-Plane Completion

**Files:**
- Modify: `include/elevation_mapping/PiecewisePlanarProcessor.hpp`
- Modify: `src/PiecewisePlanarProcessor.cpp`
- Modify: `test/PiecewisePlanarProcessorTest.cpp`

**Interfaces:**
- Extend `PiecewisePlanarParameters` at the end to preserve the existing field order:

```cpp
bool enableDirectionalGroundCompletion;
float directionalGroundMaxGapWidth;
```

- Extend internal `PlaneCandidate` with:

```cpp
grid_map::Position nearestSupportPosition;
```

- Existing public processing function signatures remain unchanged. Observer XY is `supportMap.getPosition()`.

- [ ] **Step 1: Write failing descending and disabled tests**

Add:

```cpp
TEST(PiecewisePlanarProcessorTest,
     DirectionalCompletionFillsDescendingGapWithLowerPlane)
TEST(PiecewisePlanarProcessorTest,
     DisabledDirectionalCompletionPreservesConservativeResult)
```

Use a `0.04 m` map centered at observer XY `(0, 0)`. Put a sufficiently large
accepted high-plane patch at positive X near the observer, an accepted low
patch farther along positive X, and leave the cells between them invalid.
Disable conservative completion by making its search radius too small to span
the gap. Assert every between-support cell receives the low-plane prediction
when directional completion is enabled and remains invalid when disabled.

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```bash
source /opt/ros/humble/setup.bash
source /home/guanlin/catkin_ws/install/setup.bash
cd /home/guanlin/catkin_ws
colcon build --base-paths src/elevation_mapping \
  --packages-select elevation_mapping \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
colcon test --base-paths src/elevation_mapping \
  --packages-select elevation_mapping \
  --ctest-args -R test_piecewise_planar_processor --output-on-failure
```

Expected: new descending completion assertions fail because the processor has
no directional pass.

- [ ] **Step 3: Refactor bounded candidate search without changing conservative behavior**

Change the offset helper to accept an explicit radius:

```cpp
std::vector<CellOffset> buildSearchOffsets(
    const grid_map::GridMap& map, float maximumDistance);
```

The conservative pass calls it with `parameters.maxOcclusionDistance`. The
directional pass calls it with `parameters.directionalGroundMaxGapWidth`.

Do not add the directional width to validation that gates the whole
piecewise-planar processor. Existing regularization and conservative
completion remain valid when the directional feature is disabled or its
width is invalid. The directional pass itself returns without writing unless
its enable flag is true and its width is finite and positive.

When `findEligibleCandidates()` finds a nearer support cell, update both
`nearestSquaredDistance` and `nearestSupportPosition`. For exactly equal
distance, choose the lexicographically smaller physical `(x, y)` support
position so results are deterministic.

- [ ] **Step 4: Implement directional pair validation**

For each originally invalid cell `C`, enumerate distinct-height candidate
pairs. Label the higher prediction `H` and lower prediction `L`, then compute:

```cpp
const grid_map::Position delta =
    low.nearestSupportPosition - high.nearestSupportPosition;
const double squaredGap = delta.squaredNorm();
const double observerProjection =
    (observer - high.nearestSupportPosition).dot(delta) / squaredGap;
const double cellProjection =
    (cellPosition - high.nearestSupportPosition).dot(delta) / squaredGap;
const grid_map::Position projected =
    high.nearestSupportPosition + cellProjection * delta;
const double lateralDistance = (cellPosition - projected).norm();
```

Accept only when:

```cpp
squaredGap > 0.0
squaredGap <= directionalGroundMaxGapWidth^2
observerProjection < 0.0
cellProjection >= 0.0 && cellProjection <= 1.0
lateralDistance <= std::sqrt(2.0) * supportMap.getResolution()
```

Select among valid pairs by:

```text
distance(C, highSupport) + distance(C, lowSupport),
high stable plane ID,
low stable plane ID
```

Write the selected low prediction and `+/- inferredHalfRange` bounds to the
publication map using physical-position-to-output-index conversion.

- [ ] **Step 5: Preserve unique inferred-cell accounting and pass order**

Run conservative completion first and directional completion second. Track
original support indices written by either pass in one `std::set<IndexKey>`.
Directional completion may overwrite a conservative value, but
`PiecewisePlanarResult::inferredCells` must count that physical cell once.

Both passes read only `supportMap`; neither may read inferred values from
`outputMap` as support.

- [ ] **Step 6: Add direction, width, and missing-support tests**

Add:

```cpp
TEST(PiecewisePlanarProcessorTest,
     DirectionalCompletionDoesNotTriggerWhenAscending)
TEST(PiecewisePlanarProcessorTest,
     DirectionalCompletionUsesMapCenterAsObserverXY)
TEST(PiecewisePlanarProcessorTest,
     DirectionalCompletionRejectsGapWiderThanMaximum)
TEST(PiecewisePlanarProcessorTest,
     DirectionalCompletionRequiresObservedLowerPlane)
TEST(PiecewisePlanarProcessorTest,
     DirectionalCompletionRejectsCellsBesideSupportSegment)
TEST(PiecewisePlanarProcessorTest,
     DirectionalCompletionDoesNotPropagateInferredCells)
```

For ascending geometry, place low support between observer and high support.
For the observer test, move the map center across the same fixed high/low
ordering and verify that only the high-side XY position triggers completion;
there is no observer-Z input in the public interface.
For width rejection, make nearest high/low support separation greater than
`0.40 m`. For lateral rejection, place the invalid cell more than
`sqrt(2) * resolution` from the support segment.

- [ ] **Step 7: Add wide-view, stairs, tie, and circular-buffer tests**

Add:

```cpp
TEST(PiecewisePlanarProcessorTest,
     DirectionalCompletionHandlesTwoPlatformsInWideView)
TEST(PiecewisePlanarProcessorTest,
     DirectionalCompletionKeepsMultipleStairLevelsSeparate)
TEST(PiecewisePlanarProcessorTest,
     DirectionalCompletionPairTieIsDeterministic)
TEST(PiecewisePlanarProcessorTest,
     DirectionalCompletionWritesByPhysicalPosition)
```

The wide-view fixture contains two spatially separate high-to-low corridors
with different lower heights. The stair fixture verifies each gap uses its
adjacent lower level, not the globally lowest plane. The circular-buffer test
moves only `outputMap`, restores its map center, confirms different start
indices, and validates physical output positions.

- [ ] **Step 8: Run processor and existing regressions**

Run:

```bash
colcon build --base-paths src/elevation_mapping \
  --packages-select elevation_mapping \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
colcon test --base-paths src/elevation_mapping \
  --packages-select elevation_mapping \
  --ctest-args \
  -R 'test_(piecewise_planar_processor|small_hole_filler|scan_cell_selector)' \
  --output-on-failure
colcon test-result --test-result-base build/elevation_mapping --verbose
```

Expected: all processor tests, 8 small-hole tests, and 6 scan-cell-selector
tests pass. Record the known unrelated input-source XML warning separately.

- [ ] **Step 9: Commit Task 1**

```bash
git add include/elevation_mapping/PiecewisePlanarProcessor.hpp \
  src/PiecewisePlanarProcessor.cpp \
  test/PiecewisePlanarProcessorTest.cpp
git commit -m "feat: complete descending ground occlusions"
```

---

### Task 2: ROS Parameters And MID360 Enablement

**Files:**
- Modify: `include/elevation_mapping/ElevationMap.hpp`
- Modify: `src/ElevationMap.cpp`
- Modify: `config/robots/fastlio_mid360.yaml`
- Modify: `test/PiecewisePlanarProcessorTest.cpp`

**Interfaces:**
- Consumes Task 1's two new `PiecewisePlanarParameters` fields.
- Existing publication-copy call and support snapshot remain unchanged.

- [ ] **Step 1: Add failing default and invalid-configuration tests**

Add processor tests proving:

```cpp
parameters.enableDirectionalGroundCompletion = false;
```

preserves current behavior, and enabled processing with non-finite, zero, or
negative `directionalGroundMaxGapWidth` does not directionally modify output.
Conservative processing remains available when its own parameters are valid.

- [ ] **Step 2: Add exact constructor defaults**

Extend the `PiecewisePlanarParameters` initializer in `ElevationMap`:

```cpp
false,  // enableDirectionalGroundCompletion
0.40f   // directionalGroundMaxGapWidth
```

Do not change the existing tuned planar values.

- [ ] **Step 3: Declare and read ROS parameters**

Declare/read:

```text
enable_directional_ground_completion
directional_ground_max_gap_width
```

The default enable value is `false`; the default maximum width is `0.40`.
Invalid width disables only directional completion for that call and must not
disable regularization or existing conservative completion.

- [ ] **Step 4: Enable only the MID360 profile**

Add:

```yaml
enable_directional_ground_completion: true
directional_ground_max_gap_width: 0.40
```

to `config/robots/fastlio_mid360.yaml`. Do not modify another robot profile.
Keep:

```yaml
planar_max_occlusion_distance: 0.10
planar_min_occlusion_support: 6
planar_inferred_half_range: 0.05
```

unchanged.

- [ ] **Step 5: Build and run focused regression tests**

Run:

```bash
colcon build --base-paths src/elevation_mapping \
  --packages-select elevation_mapping \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
colcon test --base-paths src/elevation_mapping \
  --packages-select elevation_mapping \
  --ctest-args \
  -R 'test_(piecewise_planar_processor|small_hole_filler|scan_cell_selector)' \
  --output-on-failure
```

Expected: all selected targets pass with directional completion disabled by
default outside MID360 YAML.

- [ ] **Step 6: Commit Task 2**

```bash
git add include/elevation_mapping/ElevationMap.hpp \
  src/ElevationMap.cpp \
  config/robots/fastlio_mid360.yaml \
  test/PiecewisePlanarProcessorTest.cpp
git commit -m "feat: enable directional ground completion for mid360"
```

---

### Task 3: Unfiltered Bag Directional Acceptance And RViz

**Files:**
- Modify only if one accepted parameter adjustment is required:
  `config/robots/fastlio_mid360.yaml`

**Interfaces:**
- Consumes the unfiltered bag at
  `/home/guanlin/catkin_ws/rosbag2_2026_07_25-09_22_56`.
- Produces physical-coordinate disabled/enabled snapshots and leaves the
  accepted mapper plus RViz running.

- [ ] **Step 1: Rebuild and stop stale task processes**

Build Release using the explicit package base path. Stop only mapper, RViz,
bag, and temporary diagnostics started for this task.

- [ ] **Step 2: Capture directional-disabled baseline**

Start a fresh mapper with normal MID360 parameters but override only:

```text
enable_directional_ground_completion:=false
```

Because the launch file does not expose this as a launch argument, run the
mapper executable directly with both installed parameter files and the final
ROS parameter override. Start temporary snapshot/metrics subscribers, replay
the unfiltered bag from its beginning with `--clock`, then stop this mapper.

- [ ] **Step 3: Capture directional-enabled output**

Start a second fresh mapper using the committed MID360 YAML, use the same
subscribers, and replay the same bag from its beginning. Decode GridMap
column-major storage and circular-buffer indices into physical XY before
comparing snapshots.

- [ ] **Step 4: Quantitatively classify directional-only cells**

For each enabled-only finite cell:

1. identify nearby accepted high and low height clusters;
2. find the selected local high-to-low support corridor;
3. verify support width is at most `0.40 m`;
4. verify observer/high/cell/low projection order;
5. verify output height matches the local lower-plane prediction; and
6. count ascending-view, intermediate-height, over-width, lateral, and
   missing-low-support violations.

Acceptance requires zero violations in all categories. Report both final-frame
counts and aggregate counts over the final stable frames. If the bag contains
no descending directional-only samples, report insufficient evidence instead
of claiming success.

- [ ] **Step 5: Verify map quality does not regress**

Reuse the approved local residual metric:

- `0.12 m` local neighborhood;
- neighbor height difference at most `0.06 m`;
- center excluded;
- at least six neighbors; and
- center residual against the neighbor-fitted plane.

Directional-enabled finite coverage and local residual MAD must not be worse
than the directional-disabled run.

- [ ] **Step 6: Launch final mapper and RViz**

If all gates pass, leave a normal YAML launch running with:

```text
fixed frame: world
map topic: /elevation_map
directional completion: true
```

Do not leave a bag player or temporary diagnostic process running.

- [ ] **Step 7: Commit a parameter adjustment only when required**

If one approved directional parameter adjustment is necessary and passes a
fresh replay, commit only the YAML change. Otherwise create no commit.
