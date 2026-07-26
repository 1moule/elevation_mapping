# MID360 Customization README Design

## Goal

Document the repository's current MID360 and FAST-LIO2 elevation-mapping
customizations in the main `README.md` without replacing or rewriting the
upstream ROS 2 package documentation.

The new documentation must distinguish implemented behavior from current
limitations and proposed future work.

## Location And Language

Add a new top-level section after the existing Overview:

```text
## MID360 / FAST-LIO2 定制修改
```

Write explanatory text in Chinese. Keep ROS topic names, frame names,
parameters, commands, file paths, and code identifiers in English.

## Required Content

### Integration Contract

Document the validated live configuration:

- input point cloud: `/fastlio2/body_cloud`;
- odometry: `/fastlio2/lio_odom`;
- map frame: `world`;
- robot frame: `body`;
- map geometry: 3 m by 3 m at 4 cm resolution;
- fused map publication: `/elevation_map` at approximately 20 Hz;
- Livox point cloud, IMU, and FAST-LIO2 rates observed during live validation.

State that FAST-LIO2 and Livox timing/configuration are not modified by the
elevation-mapping feature.

### Implemented Mapping Changes

Summarize:

- edge-aware fusion and edge-safe publication hole filling;
- adaptive lower-surface recovery for descending steps;
- scan-local spatially supported height-mode extraction;
- at most two persistent height modes per cell;
- primary-mode hysteresis requiring a one-bin advantage for three consecutive
  scans;
- 0.5 second stale-mode expiry;
- full state reset/alignment on map movement, clear, geometry changes, and raw
  map replacement;
- healthy node shutdown when optional timers are disabled;
- deterministic fixed-ROI replay analysis.

Explain that the feature is controlled by `enable_multimodal_cells` and that
setting it to `false` preserves the legacy single-height path.

### Parameters

List the validated MID360 parameters from
`config/robots/fastlio_mid360.yaml`, including mode separation, minimum points,
minimum occupied bins, switch hysteresis, stale timeout, hole-filling controls,
and visibility-cleanup state.

Do not duplicate every package parameter already documented later in the
README.

### Build And Run

Document commands from the workspace root:

```bash
./build_elevation_mapping.sh
./run_fastlio_elevation_mapping.sh --with-elevation-rviz
```

State that the Livox host interface must own `192.168.1.1/24` when using the
repository's current `MID360_config.json`, and that the configured LiDAR address
is `192.168.1.123`.

Include the focused package build and test commands for developers.

### Replay Validation

Document the fixed input bag, isolated ROS domains, baseline plus three
candidate workflow, and the analyzer comparison command.

Report the final validated acceptance values:

- vertical switch reduction: `0.9864864864864865`;
- finite coverage ratio: `1.0`;
- flat variation ratio: `0.9987039357152114`;
- edge transition width: `0.0` cells in the fixed ROI;
- deterministic candidate signatures: `true`;
- CPU increase ratio: `0.006007751937984418`.

State that the primary ROI covers the dominant flat ground and raised platform;
isolated surrounding obstacles are secondary diagnostics.

### Current Semantics And Limitations

Explicitly document:

- TF aligns point clouds and moves the rolling map; it does not infer unseen
  terrain height.
- Previously observed cells may remain visible after they become occluded.
- Small publication holes may be filled from consistent nearby support.
- Visibility cleanup is currently disabled in the MID360 configuration.
- The public map remains a single-valued 2.5D elevation layer. Internal
  multimodal state reduces height switching but cannot fully represent a
  vertical wall inside one 4 cm output cell.
- Persistent `3x3` subcell ownership and coordinate-level split-height queries
  are proposed future work and must not be described as implemented.

## Presentation

Use short subsections, tables for topic/parameter summaries, and fenced shell
blocks for commands. Link to the existing detailed design and implementation
plan instead of copying their internal task history.

Keep the existing upstream authorship, citation, installation, node, service,
and parameter documentation unchanged.

## Verification

Before committing the README change:

1. Verify every documented topic, frame, parameter, command, and metric against
   the current branch.
2. Confirm future subcell work is clearly labeled as not implemented.
3. Run a Markdown structure check for duplicate top-level headings and broken
   local paths.
4. Run `git diff --check`.
