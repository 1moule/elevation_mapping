# MID360 Customization README Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an accurate Chinese summary of the repository's implemented MID360 and FAST-LIO2 elevation-mapping customizations to the main README.

**Architecture:** Insert one self-contained top-level section after the upstream Overview and before Citing. Preserve all upstream documentation, link to the detailed design and implementation plan, and clearly separate implemented behavior, measured results, current limitations, and unimplemented future work.

**Tech Stack:** Markdown, ROS 2 Humble, Livox MID360, FAST-LIO2, elevation_mapping, Bash validation commands.

## Global Constraints

- Modify documentation only; do not change FAST-LIO2, Livox, elevation-mapping code, launch files, or runtime parameters.
- Write explanations in Chinese while keeping topic names, frame names, parameters, commands, paths, and identifiers in English.
- Preserve the existing upstream authorship, citation, installation, node, service, and parameter documentation.
- Describe persistent `3x3` subcell ownership and coordinate-level split-height queries only as unimplemented future work.
- Use the validated input topics `/fastlio2/body_cloud` and `/fastlio2/lio_odom`, frames `world` and `body`, and output topic `/elevation_map`.

---

### Task 1: Document And Verify The MID360 Customizations

**Files:**
- Modify: `README.md`
- Reference: `config/robots/fastlio_mid360.yaml`
- Reference: `scripts/run_elevation_bag_regression.sh`
- Reference: `scripts/analyze_elevation_bag.py`
- Reference: `docs/superpowers/specs/2026-07-25-multimodal-edge-elevation-design.md`
- Reference: `docs/superpowers/plans/2026-07-25-multimodal-edge-elevation-implementation.md`

**Interfaces:**
- Consumes: Current branch configuration, scripts, accepted replay metrics, and the approved README design.
- Produces: A new `## MID360 / FAST-LIO2 定制修改` section in `README.md`; no runtime interface changes.

- [ ] **Step 1: Verify all facts that will be documented**

Run from `/home/guanlin/catkin_ws/src/elevation_mapping`:

```bash
rg -n "map_frame_id|robot_base_frame_id|length_in_[xy]|resolution|edge_aware_fusion|fusion_height_difference_threshold|enable_multimodal_cells|multimodal_|fused_map_publishing_rate|enable_fused_map_hole_filling|fused_map_hole_filling_|enable_visibility_cleanup|enable_continuous_cleanup" \
  config/robots/fastlio_mid360.yaml
rg -n "compare|candidate_runs_identical|vertical_switch_reduction|finite_coverage_ratio|flat_variation_ratio|cpu_increase_ratio|edge_transition_width" \
  scripts/analyze_elevation_bag.py
test -x /home/guanlin/catkin_ws/build_elevation_mapping.sh
test -x /home/guanlin/catkin_ws/run_fastlio_elevation_mapping.sh
test -d /home/guanlin/catkin_ws/rosbag2_2026_07_25-09_22_56
```

Expected: The config contains the exact validated values from the design; both workspace scripts are executable; the fixed bag exists; the analyzer exposes the documented comparison metrics.

- [ ] **Step 2: Add the customization section**

Use `apply_patch` to insert `## MID360 / FAST-LIO2 定制修改` after the final Overview paragraph and before `## Citing`. Add these concise subsections:

1. `### 当前接入关系`: tables for topics, frames, 3 m by 3 m map geometry, 0.04 m resolution, and approximately 20 Hz fused-map output; include the observed live rates of approximately 10 Hz LiDAR, 200 Hz IMU, and 10 Hz FAST-LIO2 odometry.
2. `### 已实现的建图修改`: edge-aware fusion, edge-safe hole filling, adaptive lower-surface recovery, scan-local mode extraction, two persistent modes per cell, one-bin/three-scan switch hysteresis, 0.5 s expiry, state lifecycle alignment, optional-timer shutdown handling, deterministic ROI replay analysis, and the `enable_multimodal_cells: false` legacy path.
3. `### MID360 已验证参数`: document the exact values in `config/robots/fastlio_mid360.yaml` without duplicating the package-wide parameter reference.
4. `### 编译与运行`: show `./build_elevation_mapping.sh` and `./run_fastlio_elevation_mapping.sh --with-elevation-rviz` from `/home/guanlin/catkin_ws`; state host address `192.168.1.1/24` and LiDAR address `192.168.1.123`; include focused package build and test commands.
5. `### 固定 Bag 回放验证`: show one feature-disabled baseline and three feature-enabled candidates in isolated ROS domains, followed by the exact analyzer `--compare` command; report the six accepted metric values from the approved design.
6. `### 语义、限制与后续工作`: state that TF aligns data but does not estimate unseen height, occluded observations may remain, consistent small holes may be filled, visibility cleanup is disabled, and the public map is still single-valued 2.5D; label persistent `3x3` subcells and coordinate-level split-height queries as not implemented.
7. `### 设计文档`: add working relative links to the existing detailed feature design and implementation plan.

- [ ] **Step 3: Validate Markdown structure and local links**

Run:

```bash
test "$(rg -c '^## MID360 / FAST-LIO2 定制修改$' README.md)" -eq 1
test -z "$(awk '/^## / {print}' README.md | sort | uniq -d)"
test -f docs/superpowers/specs/2026-07-25-multimodal-edge-elevation-design.md
test -f docs/superpowers/plans/2026-07-25-multimodal-edge-elevation-implementation.md
rg -n "未实现|3x3|192\\.168\\.1\\.1/24|192\\.168\\.1\\.123|0\\.9864864864864865|0\\.006007751937984418" README.md
git diff --check
```

Expected: one customization heading, no duplicate level-two headings, both linked local documents exist, future subcell work is marked unimplemented, addresses and accepted metrics are present, and no whitespace errors are reported.

- [ ] **Step 4: Review scope and commit**

Run:

```bash
git diff -- README.md
git status --short
git add README.md
git commit -m "docs: document MID360 elevation mapping changes"
```

Expected: The README is the only implementation file changed by this task, upstream sections remain unchanged, and the commit succeeds.
