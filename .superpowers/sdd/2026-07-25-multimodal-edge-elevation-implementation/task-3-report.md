# Task 3 Report: Persistent Modes and Primary-Selection Hysteresis

## Scope

Implemented Task 3 in `MultimodalElevation.hpp/.cpp` and its pure tests only.
No `ElevationMap` integration was added.

## TDD Evidence

### RED

1. Added the persistent-state declarations and selection fixtures/tests before
   production implementations.
2. Built from `/tmp/elevation_mapping_sdd_ws` after sourcing
   `/home/guanlin/catkin_ws/install/setup.bash` as the dependency underlay.
3. The expected link failure occurred: the edge-aware test target reported
   undefined references to `updateMultimodalCell`,
   `expireStaleSecondaryMode`, and `countValidModes`.
4. Added the distinct-timestamp challenger regression test before production
   code and rebuilt. The target again failed at link time for the new API,
   confirming the test suite was RED for the missing implementation.

### GREEN

Implemented:

- Persistent two-slot mode state and update result interfaces.
- Greedy global one-to-one height matching within `modeSeparation`, preserving
  persistent slot positions.
- Scalar Kalman height/variance fusion, current-observation metadata refresh,
  confidence updates, consecutive-observation tracking, and unobserved decay.
- Primary ranking by center occupancy then coverage; challenger hysteresis with
  `switchConfirmations`, `switchMarginBins`, and one count per timestamp.
- Valid-mode counting and secondary-only stale expiration that clears any
  associated challenger state while preserving the selected primary.
- Pure tests for center preference, three-scan switching, same-timestamp
  deduplication, tied support, order-independent matching/filtering, initial
  single-mode rejection, and stale secondary/primary behavior.

## Verification

- `colcon build --event-handlers console_direct+ --packages-select elevation_mapping --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release`
  completed with `test_elevation_mapping_edge_aware_map` at 100%.
- `./build/elevation_mapping/test_elevation_mapping_edge_aware_map --gtest_filter='Multimodal*.*' --gtest_color=no` passed 16/16 tests.
- The complete `test_elevation_mapping_edge_aware_map` target passed 27/27
  tests. It was run with `ROS_LOG_DIR=/tmp/elevation_mapping_ros_log` and
  permission for local ROS middleware sockets, because those resources are not
  available inside the filesystem/network sandbox.

## Self-Review

- Confirmed no persistent-mode slots are sorted after updates.
- Confirmed only an observed, strictly better challenger can advance the
  hysteresis count; ties reset it.
- Confirmed stale expiration cannot remove the `primaryIndex` slot.
- Ran `git diff --check` before commit; no whitespace errors.

## Commit

`0eb8a01 feat: stabilize multimodal primary height`

---

## Fix Round 1

### Scope

Fixed all three follow-up findings in the Task 3 multimodal implementation
and tests:

- Replaced greedy matching with exhaustive, deterministic two-slot assignment.
  It maximizes the number of valid matches, then minimizes total height
  distance, with a canonical observation ordering for equal-cost ties.
- Empty observations now decay confidence and reset consecutive-observation
  counts before stale secondary expiration; the primary is preserved.
- `switchMarginBins` must be in the inclusive range `[1, 9]`.

### RED

Command:

```bash
cd /tmp/elevation_mapping_sdd_ws
source /opt/ros/humble/setup.bash
source /home/guanlin/catkin_ws/install/setup.bash
colcon build --event-handlers console_direct+ --packages-select elevation_mapping \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
./build/elevation_mapping/test_elevation_mapping_edge_aware_map \
  --gtest_filter='Multimodal*.*' --gtest_color=no
```

Relevant output: the suite ran 19 tests, with 15 passing and four expected
failures: `RejectsInvalidConfiguration`,
`RejectsZeroSwitchMarginRatherThanSwitchingOnTie`,
`GloballyMatchesAmbiguousObservationsIndependentOfOrder`, and
`EmptyObservationAgesPrimaryAndExpiresSecondary`. The assignment regression
showed the ascending input left the zero-height slot unobserved, while the
reversed input matched both slots.

### GREEN

Command:

```bash
cd /tmp/elevation_mapping_sdd_ws
source /opt/ros/humble/setup.bash
source /home/guanlin/catkin_ws/install/setup.bash
colcon build --event-handlers console_direct+ --packages-select elevation_mapping \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
./build/elevation_mapping/test_elevation_mapping_edge_aware_map \
  --gtest_filter='Multimodal*.*' --gtest_color=no
```

Relevant output: build completed successfully and all 19 multimodal tests
passed.

### Full Edge-Aware Target

Command:

```bash
cd /tmp/elevation_mapping_sdd_ws
source /opt/ros/humble/setup.bash
source /home/guanlin/catkin_ws/install/setup.bash
source install/setup.bash
export ROS_LOG_DIR=/tmp/elevation_mapping_ros_log
./build/elevation_mapping/test_elevation_mapping_edge_aware_map --gtest_color=no
```

Relevant output: all 30 tests from four suites passed. This command requires
local ROS middleware sockets, so it was run outside the filesystem/network
sandbox with the workspace overlay and `/tmp` log directory.

### Self-Review

- Verified the exact state slots `[0.0625, 0.0]` and observation heights
  `[0.03125, 0.09375]` match both modes in either input order.
- Verified empty updates retain `handled == false` while decaying the primary
  and expiring only a stale secondary.
- Verified zero margin is rejected and tied valid support retains the
  incumbent.
- Ran `git diff --check`; no whitespace errors.
