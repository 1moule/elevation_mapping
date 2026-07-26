# Fast Edge Hole Filling Design

## Goal

Eliminate the visible delay when filling small holes at observed platform and
step edges while preserving crisp, discrete high/low elevations.

The feature must fill enclosed holes in the already observed terrain on the
next fused-map publication. Terrain outside the observed region must remain
invalid.

## Current Behavior And Root Cause

`ElevationMap::fillHolesForPublication()` runs for every fused-map publication,
currently configured at 20 Hz. The delay is therefore not caused by a timer.

The current `findEdgeSafeHoleFillSource()` rejects a hole when:

- valid support is present on only one side;
- all valid neighbors do not fit within one 0.05 m height range; or
- any neighbor reports two height modes separated by more than 0.05 m.

A platform edge naturally contains both the upper and lower surfaces. The
current function rejects that mixed neighborhood until later LiDAR scans happen
to produce a single-height neighborhood, which appears as slow filling.

## Scope

Modify only the fused-map publication hole-filling selection and its focused
tests and replay metrics.

Do not change:

- Livox or FAST-LIO2 timing, topics, frames, or parameters;
- the raw elevation map;
- persistent multimodal cell state;
- the 3 m by 3 m map geometry or 0.04 m resolution;
- `fused_map_hole_filling_radius: 0.12`;
- the public single-valued 2.5D map contract;
- unknown terrain outside the observed region.

Persistent `3x3` subcells and coordinate-level split-height queries remain
future work.

## Algorithm

For each invalid elevation cell in the fused-map publication copy:

1. Collect finite neighboring elevation samples within
   `fused_map_hole_filling_radius`.
2. Require at least `fused_map_hole_filling_min_support` finite samples.
3. Apply the existing angular enclosure test to the union of all finite
   samples. The largest angular gap must not exceed 180 degrees.
4. Sort samples deterministically by elevation, distance, and grid index.
5. Split samples into height modes. Every mode's maximum elevation minus its
   minimum elevation must not exceed
   `fused_map_hole_filling_height_threshold`.
   The mode elevation used for ranking is the median member elevation.
6. Rank modes by:
   - shortest member distance to the hole;
   - then larger sample count;
   - then higher mode elevation.
7. Select the nearest sample in the winning mode. Resolve any remaining tie by
   stable grid-index order.
8. Copy the selected sample's `elevation`, `upper_bound`, `lower_bound`,
   `color`, `secondary_elevation`, `height_mode_count`,
   `height_mode_separation`, and `primary_mode_confidence` when the layers
   exist.

The selected elevation is always copied from an observed neighbor. Heights from
different modes are never averaged.

## Unknown-Region Protection

The angular enclosure check is evaluated before height clustering and uses all
finite neighbors. A hole between an observed upper surface and an observed
lower surface can therefore pass even though each height mode is spatially
one-sided.

A cell immediately outside the observed map boundary still has one-sided
support as a whole and fails the enclosure check. This prevents filling from
growing into never-observed terrain.

Hole filling continues to operate only on a temporary publication copy. Filled
cells are not written into `fusedMap_`, so they cannot become support for
outward propagation in later publications.

## Parameter Semantics

No new parameter is required.

- `fused_map_hole_filling_radius: 0.12` remains the maximum search radius.
- `fused_map_hole_filling_min_support: 4` remains the minimum total finite
  support.
- `fused_map_hole_filling_height_threshold: 0.05` becomes the maximum
  within-mode height spread instead of the maximum spread across the entire
  mixed neighborhood.

Invalid radius, support, or threshold values retain the existing behavior of
disabling publication hole filling during initialization.

## Determinism And Performance

Mode construction and ranking must not depend on `CircleIterator` traversal
order. Sorting and explicit tie-breaks make repeated publications and Bag
replays deterministic.

The search radius is unchanged. Sorting is limited to the small local
neighborhood of each candidate hole, so the asymptotic map traversal remains
unchanged and the 20 Hz fused-map publication target is retained.

## Unit Tests

Extend `test/EdgeAwareMapUtilsTest.cpp` with tests that prove:

1. An enclosed mixed upper/lower neighborhood returns a valid source in one
   call.
2. The selected elevation equals one observed mode and is never an intermediate
   height.
3. The closest height mode wins.
4. Equal-distance modes prefer more support.
5. A complete tie prefers the higher mode.
6. Reordered sample insertion produces the same source height.
7. One-sided support outside the observed region remains rejected.
8. Existing flat-hole and circular-buffer tests continue to pass.

Add or extend an `ElevationMap` publication test to verify all available
coupled layers are copied from the selected source.

## Bag Replay Validation

Extend `scripts/analyze_elevation_bag.py` with
`enclosed_hole_count`. A cell contributes to this metric when:

- its published `elevation` is invalid;
- at least four finite neighbors exist within 0.12 m; and
- the union of those neighbors passes the 180-degree angular enclosure test.

The metric is evaluated in the existing deterministic target ROI covering the
dominant flat ground and raised platform.

Use the fixed Bag:

```text
/home/guanlin/catkin_ws/rosbag2_2026_07_25-09_22_56
```

Each replay uses its existing isolated ROS domain.

The validation sequence is:

1. Add and self-test the analyzer metric without changing the hole-filling
   algorithm.
2. Run one feature-enabled reference with the current hole-filling algorithm
   and retain its metrics.
3. Implement the new selection algorithm.
4. Run the existing feature-disabled baseline and three feature-enabled
   candidates for the established multimodal acceptance comparison.
5. Compare all three candidates with the pre-change feature-enabled reference
   for enclosed holes, finite coverage, flat variation, and CPU time.

## Acceptance Criteria

- `enclosed_hole_count == 0` in each candidate target ROI.
- Mixed-height edge holes are filled by one function call.
- A filled edge cell equals an observed high or low mode, never their average.
- One-sided unknown-region cells remain invalid.
- `edge_transition_width_cells <= 1`.
- `candidate_runs_identical == true`.
- Candidate finite coverage is not lower than the pre-change feature-enabled
  reference.
- Candidate flat-region 95th-percentile variation is not higher than the
  pre-change feature-enabled reference.
- CPU increase is no more than 25% relative to the pre-change feature-enabled
  reference.
- Existing edge-aware, multimodal, lifecycle, analyzer, and replay-runner tests
  continue to pass.

## Documentation

After validation, update the MID360 customization section in `README.md`:

- explain mode-aware edge hole filling;
- retain the distinction between enclosed observed holes and unknown terrain;
- report the new replay metric without replacing the previously validated
  metrics.
