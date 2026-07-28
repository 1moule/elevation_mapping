# Directional Ground Completion Design

## Goal

Fill platform and stair backside holes when the robot observes from a higher
surface toward an already observed lower surface. Assign the hidden band to
the lower plane instead of splitting it between the nearest high and low
planes.

The rule must work independently for multiple platforms and stair levels in a
wide LiDAR field of view. It must not infer a lower surface that has never
been observed.

## Existing Behavior

The piecewise-planar processor currently searches at most `0.10 m` from an
unknown cell. If distinct high and low plane candidates are available, it
selects the candidate with the nearest XY support.

Consequently:

- wider backside gaps remain unknown; and
- cells near the high edge may be assigned to the high plane even when the
  hidden region should continue as lower ground.

The current conservative completion remains useful when the robot observes
from a low surface toward a higher platform and must remain unchanged.

## Observer Position

Use the rolling grid map center as the observer XY position. The MID360 profile
tracks `body` at `(0, 0, 0)`, so the map center is the `body` XY projection in
`world`.

Do not compare `body.z` with plane heights. The sensor and body may be above
every mapped surface.

The directional feature defaults to disabled. Enabling it on another robot
profile requires that its map tracking point represent the desired observer
XY position.

## Directional Plane Pair

For an originally invalid cell at position `C`, collect accepted plane
candidates using the immutable pre-fill support map. Each candidate records:

- stable plane ID;
- predicted plane height at `C`;
- locally consistent support count;
- nearest support distance; and
- physical XY position of its nearest support cell.

The directional candidate search uses bounded cell offsets within
`directional_ground_max_gap_width`. It is separate from the existing
`planar_max_occlusion_distance` search so increasing directional coverage
does not enlarge ascending or ambiguous conservative completion.

Consider every candidate pair whose predicted heights differ by at least
`planar_neighbor_height_tolerance`. Label the higher candidate `H` and the
lower candidate `L`, using their nearest support positions `p_H` and `p_L`.

The pair is directionally valid only when all conditions hold:

1. Both candidates have at least `planar_min_occlusion_support` locally
   consistent support cells.
2. `|p_L - p_H|` is no greater than
   `directional_ground_max_gap_width`.
3. Let `d = p_L - p_H` and
   `t(P) = dot(P - p_H, d) / dot(d, d)`.
   The observer is on the high side: `t(observer) < 0`.
4. The unknown cell lies between the supports: `0 <= t(C) <= 1`.
5. The perpendicular XY distance from `C` to segment `p_H--p_L` is no greater
   than `sqrt(2) * map_resolution`.

These tests implement the ordering:

```text
observer -> high edge -> unknown cell -> observed lower plane
```

They reject the reverse, ascending ordering:

```text
observer -> lower plane -> high platform
```

They also reject lateral cells near a platform corner that are not inside the
high-to-low corridor.

If more than one directional pair is valid at a cell, select the pair by the
following deterministic key:

1. smallest `distance(C, p_H) + distance(C, p_L)`;
2. lower high-plane stable ID; then
3. lower low-plane stable ID.

This associates each cell with the nearest local step transition instead of
the globally lowest plane, which preserves multiple stair levels.

## Completion Order

Operate only on the fused publication copy.

1. Preserve the immutable pre-small-hole support snapshot.
2. Run the existing small-hole filler on the publication copy.
3. Run existing `0.10 m` conservative piecewise-planar completion.
4. Run directional ground completion using the same immutable support
   snapshot.
5. Directional completion may overwrite an earlier conservative inferred
   value for an originally invalid cell with the selected lower-plane
   prediction.
6. Compute `uncertainty_range`.

Directional completion sets:

- `elevation` to the selected lower-plane prediction;
- `lower_bound` to prediction minus `planar_inferred_half_range`; and
- `upper_bound` to prediction plus `planar_inferred_half_range`.

Neither small-hole output, conservative inferred cells, nor directional
inferred cells may become support in the same publication pass.

## Parameters

Add:

```yaml
enable_directional_ground_completion: true
directional_ground_max_gap_width: 0.40
```

The feature defaults to disabled and is enabled only in
`config/robots/fastlio_mid360.yaml`.

Existing conservative parameters remain unchanged, including:

```yaml
planar_max_occlusion_distance: 0.10
planar_min_occlusion_support: 6
planar_inferred_half_range: 0.05
```

Reject directional configuration without modifying the map when the maximum
gap width is non-finite or non-positive.

## Tests

Focused unit tests verify:

- descending geometry fills every eligible blind-band cell with the lower
  plane;
- observer height is not an input to direction classification;
- ascending geometry does not trigger directional completion;
- a wide field of view containing two independent platforms associates each
  blind band with its local lower plane;
- three or more stair levels retain separate lower-plane assignments;
- a gap wider than `0.40 m` remains unknown outside existing conservative
  completion;
- a high plane without an observed lower plane is not extrapolated;
- cells beside, rather than between, high and low supports remain unknown;
- equal-distance pair selection is deterministic;
- inferred cells cannot propagate during the same pass;
- disabled directional completion preserves existing processor behavior; and
- differing support/output circular-buffer start indices map writes by
  physical position.

## Bag Acceptance

Replay the unfiltered bag from a fresh mapper with `use_sim_time:=true`.
Compared with directional completion disabled:

- finite coverage in the descending platform/stair backside region increases
  or remains unchanged;
- every directional-only cell is within `0.40 m` high-to-low support width;
- directional-only heights match the selected local lower-plane prediction
  and do not form intermediate ramps;
- ascending platform views do not gain directional-only cells; and
- broad regions without an observed lower plane remain unknown.

RViz remains an auxiliary visual check using fixed frame `world` and topic
`/elevation_map`.
