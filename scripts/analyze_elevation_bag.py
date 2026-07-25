#!/usr/bin/env python3

import argparse
import json
import math
import os
import sys
import tempfile
from pathlib import Path
from types import SimpleNamespace


RAW_MAP_TOPIC = "/elevation_map_raw_post"
GRID_MAP_TYPE = "grid_map_msgs/msg/GridMap"
REQUIRED_LAYERS = (
    "elevation",
    "secondary_elevation",
    "height_mode_count",
    "height_mode_separation",
)
EDGE_HEIGHT_THRESHOLD = 0.08
FLAT_HEIGHT_THRESHOLD = 0.05
RETURN_WINDOW_SECONDS = 1.0
TRANSITION_MARGIN_METERS = 0.02
REFERENCE_CELL_SIZE_METERS = 0.04
COMPARISON_TOLERANCE = 1e-6


def count_vertical_switches(times, heights, threshold, return_window):
    if len(times) != len(heights):
        raise ValueError("times and heights must have equal lengths")
    if threshold < 0.0 or return_window < 0.0:
        raise ValueError("threshold and return_window must be non-negative")

    samples = [
        (float(timestamp), float(height))
        for timestamp, height in zip(times, heights)
        if math.isfinite(timestamp) and math.isfinite(height)
    ]
    if any(samples[index][0] < samples[index - 1][0] for index in range(1, len(samples))):
        raise ValueError("timestamps must be nondecreasing")

    count = 0
    for index in range(len(samples) - 2):
        mode_a_time, mode_a = samples[index]
        mode_b_time, mode_b = samples[index + 1]
        del mode_a_time
        if abs(mode_b - mode_a) <= threshold:
            continue

        deadline = mode_b_time + return_window
        for return_time, returned_height in samples[index + 2:]:
            if return_time > deadline:
                break
            if (
                abs(returned_height - mode_a) <= threshold
                and abs(returned_height - mode_b) > threshold
            ):
                count += 1
                break
    return count


def finite_coverage(values):
    values = list(values)
    if not values:
        return 0.0
    return sum(math.isfinite(float(value)) for value in values) / len(values)


def percentile(values, quantile):
    if not 0.0 <= quantile <= 1.0:
        raise ValueError("quantile must be between zero and one")
    finite_values = sorted(float(value) for value in values if math.isfinite(value))
    if not finite_values:
        return 0.0
    if len(finite_values) == 1:
        return finite_values[0]

    position = (len(finite_values) - 1) * quantile
    lower_index = math.floor(position)
    upper_index = math.ceil(position)
    if lower_index == upper_index:
        return finite_values[lower_index]
    fraction = position - lower_index
    return (
        finite_values[lower_index] * (1.0 - fraction)
        + finite_values[upper_index] * fraction
    )


def decode_layer(array, outer_start_index, inner_start_index):
    dimensions = list(array.layout.dim)
    if len(dimensions) != 2:
        raise ValueError("GridMap layers must have exactly two dimensions")

    labels = [dimension.label for dimension in dimensions]
    if set(labels) != {"row_index", "column_index"}:
        raise ValueError(
            "GridMap layer dimensions must be labeled row_index and column_index"
        )

    row_dimension = dimensions[labels.index("row_index")]
    column_dimension = dimensions[labels.index("column_index")]
    rows = int(row_dimension.size)
    columns = int(column_dimension.size)
    if rows <= 0 or columns <= 0:
        raise ValueError("GridMap layer dimensions must be positive")

    data_offset = int(array.layout.data_offset)
    outer_dimension, inner_dimension = dimensions
    outer_size = int(outer_dimension.size)
    outer_stride = int(outer_dimension.stride)
    inner_stride = int(inner_dimension.stride)
    if outer_stride <= 0 or inner_stride <= 0:
        raise ValueError("GridMap layer dimension strides must be positive")
    if outer_stride % outer_size != 0:
        raise ValueError("GridMap outer dimension stride is not indexable")
    outer_index_stride = outer_stride // outer_size
    if outer_index_stride != inner_stride or inner_stride < int(inner_dimension.size):
        raise ValueError("GridMap layer dimension strides are inconsistent")

    greatest_index = (
        data_offset
        + (outer_size - 1) * outer_index_stride
        + int(inner_dimension.size) - 1
    )
    if data_offset < 0 or greatest_index >= len(array.data):
        raise ValueError("GridMap layer data is shorter than its dimensions")

    row_major = labels[0] == "row_index"
    decoded = []
    for logical_row in range(rows):
        physical_row = (int(outer_start_index) + logical_row) % rows
        row_values = []
        for logical_column in range(columns):
            physical_column = (int(inner_start_index) + logical_column) % columns
            if row_major:
                outer_index, inner_index = physical_row, physical_column
            else:
                outer_index, inner_index = physical_column, physical_row
            flat_index = outer_index * outer_index_stride + inner_index
            row_values.append(float(array.data[data_offset + flat_index]))
        decoded.append(row_values)
    return decoded


def parse_cpu_seconds(path):
    cpu_path = Path(path)
    fields = cpu_path.read_text(encoding="ascii").split()
    if len(fields) != 2:
        raise ValueError(
            f"{cpu_path} must contain exactly two whitespace-separated %U %S values"
        )
    user_seconds, system_seconds = (float(field) for field in fields)
    if not all(
        math.isfinite(value) and value >= 0.0
        for value in (user_seconds, system_seconds)
    ):
        raise ValueError(f"{cpu_path} contains invalid CPU seconds")
    return user_seconds + system_seconds


def local_height_range(world_key, heights):
    row_key, column_key = world_key
    neighbors = [
        heights[(row_key + row_offset, column_key + column_offset)]
        for row_offset in (-1, 0, 1)
        for column_offset in (-1, 0, 1)
        if (row_key + row_offset, column_key + column_offset) in heights
    ]
    if not neighbors:
        return 0.0
    return max(neighbors) - min(neighbors)


def is_edge_cell(world_key, finite_heights, separation):
    return (
        local_height_range(world_key, finite_heights) > EDGE_HEIGHT_THRESHOLD
        or (math.isfinite(separation) and separation > EDGE_HEIGHT_THRESHOLD)
    )


def multimodal_separation_for_cell(mode_count, separation):
    if (
        math.isfinite(mode_count)
        and mode_count >= 2.0
        and math.isfinite(separation)
    ):
        return separation
    return None


def transition_width_cells(world_key, heights, resolution):
    x_key, y_key = world_key
    axis_options = []
    for step in ((1, 0), (0, 1)):
        negative_key = (x_key - step[0], y_key - step[1])
        positive_key = (x_key + step[0], y_key + step[1])
        if negative_key in heights and positive_key in heights:
            difference = abs(heights[positive_key] - heights[negative_key])
            axis_options.append((difference, step, negative_key, positive_key))
    if not axis_options:
        return 0.0

    _, step, negative_key, positive_key = max(
        axis_options, key=lambda option: (option[0], option[1] == (1, 0))
    )
    low = min(heights[negative_key], heights[positive_key])
    high = max(heights[negative_key], heights[positive_key])
    lower_bound = low + TRANSITION_MARGIN_METERS
    upper_bound = high - TRANSITION_MARGIN_METERS
    if lower_bound >= upper_bound:
        return 0.0

    def is_transition_cell(key):
        return key in heights and lower_bound < heights[key] < upper_bound

    if not is_transition_cell(world_key):
        return 0.0

    count = 1
    for direction in (-1, 1):
        distance = 1
        while True:
            key = (
                x_key + direction * distance * step[0],
                y_key + direction * distance * step[1],
            )
            if not is_transition_cell(key):
                break
            count += 1
            distance += 1
    return count * float(resolution) / REFERENCE_CELL_SIZE_METERS


def world_key_to_string(world_key):
    return f"{world_key[0]},{world_key[1]}"


def key_list(world_keys):
    return [[world_key[0], world_key[1]] for world_key in sorted(world_keys)]


def message_timestamp(message, bag_timestamp_nanoseconds):
    stamp = message.header.stamp
    timestamp = float(stamp.sec) + float(stamp.nanosec) * 1e-9
    if timestamp == 0.0:
        timestamp = float(bag_timestamp_nanoseconds) * 1e-9
    return timestamp


def decode_frame(message, bag_timestamp_nanoseconds):
    if len(message.layers) != len(message.data):
        raise ValueError("GridMap layer names and data have different lengths")

    layer_messages = dict(zip(message.layers, message.data))
    missing_layers = [
        layer for layer in REQUIRED_LAYERS if layer not in layer_messages
    ]
    if missing_layers:
        raise ValueError(f"GridMap is missing required layers: {missing_layers}")

    layers = {
        layer: decode_layer(
            layer_messages[layer],
            message.outer_start_index,
            message.inner_start_index,
        )
        for layer in REQUIRED_LAYERS
    }
    rows = len(layers["elevation"])
    columns = len(layers["elevation"][0])
    for layer_name, layer in layers.items():
        if len(layer) != rows or any(len(row) != columns for row in layer):
            raise ValueError(f"GridMap layer {layer_name} has inconsistent dimensions")

    resolution = float(message.info.resolution)
    length_x = float(message.info.length_x)
    length_y = float(message.info.length_y)
    center_x = float(message.info.pose.position.x)
    center_y = float(message.info.pose.position.y)
    if not math.isfinite(resolution) or resolution <= 0.0:
        raise ValueError("GridMap resolution must be finite and positive")

    timestamp = message_timestamp(message, bag_timestamp_nanoseconds)
    values_by_key = {}
    for row in range(rows):
        x = center_x + 0.5 * length_x - (row + 0.5) * resolution
        x_key = round(x / resolution)
        for column in range(columns):
            y = center_y + 0.5 * length_y - (column + 0.5) * resolution
            world_key = (x_key, round(y / resolution))
            if world_key in values_by_key:
                raise ValueError(
                    f"GridMap geometry produced duplicate world key {world_key}"
                )
            values_by_key[world_key] = tuple(
                layers[layer][row][column] for layer in REQUIRED_LAYERS
            )
    return timestamp, resolution, rows * columns, values_by_key


def read_grid_map_messages(bag_path):
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:
        raise RuntimeError(
            "ROS 2 Python packages are unavailable; source the ROS workspace"
        ) from error

    bag_path = Path(bag_path)
    if not bag_path.exists():
        raise FileNotFoundError(f"input bag does not exist: {bag_path}")

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(
        uri=str(bag_path), storage_id=""
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_options, converter_options)

    topic_types = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    if RAW_MAP_TOPIC not in topic_types:
        raise ValueError(f"bag does not contain required topic {RAW_MAP_TOPIC}")
    if topic_types[RAW_MAP_TOPIC] != GRID_MAP_TYPE:
        raise ValueError(
            f"{RAW_MAP_TOPIC} has type {topic_types[RAW_MAP_TOPIC]}, "
            f"expected {GRID_MAP_TYPE}"
        )
    grid_map_message = get_message(GRID_MAP_TYPE)

    while reader.has_next():
        topic, serialized_data, bag_timestamp = reader.read_next()
        if topic == RAW_MAP_TOPIC:
            yield deserialize_message(serialized_data, grid_map_message), bag_timestamp


def consecutive_variations(samples):
    heights = [sample[1] for sample in samples if math.isfinite(sample[1])]
    return [
        abs(heights[index] - heights[index - 1])
        for index in range(1, len(heights))
    ]


def analyze_bag(bag_path, mapper_cpu_seconds):
    timelines = {}
    edge_keys = set()
    flat_seen_keys = set()
    nonflat_keys = set()
    multimodal_keys = set()
    multimodal_cell_count = 0
    multimodal_separations = []
    transition_widths = []
    frame_coverages = []
    frame_count = 0

    for message, bag_timestamp in read_grid_map_messages(bag_path):
        timestamp, resolution, cell_count, values_by_key = decode_frame(
            message, bag_timestamp
        )
        frame_count += 1
        heights = {
            key: values[0]
            for key, values in values_by_key.items()
            if math.isfinite(values[0])
        }
        frame_coverages.append(len(heights) / cell_count)

        for key, values in values_by_key.items():
            elevation, secondary, mode_count, separation = values
            if is_edge_cell(key, heights, separation):
                edge_keys.add(key)
            valid_multimodal_separation = multimodal_separation_for_cell(
                mode_count, separation
            )
            if valid_multimodal_separation is not None:
                multimodal_cell_count += 1
                multimodal_keys.add(key)
                multimodal_separations.append(valid_multimodal_separation)
            if not math.isfinite(elevation):
                continue
            timelines.setdefault(key, []).append(
                (timestamp, elevation, secondary, mode_count, separation)
            )

            neighbor_range = local_height_range(key, heights)
            flat_seen_keys.add(key)
            if neighbor_range >= FLAT_HEIGHT_THRESHOLD:
                nonflat_keys.add(key)

            if valid_multimodal_separation is not None:
                transition_widths.append(
                    transition_width_cells(key, heights, resolution)
                )

    if frame_count == 0:
        raise ValueError(f"bag contains no messages on {RAW_MAP_TOPIC}")

    switch_counts = {}
    variations = {}
    for key in sorted(timelines):
        samples = timelines[key]
        times = [sample[0] for sample in samples]
        heights = [sample[1] for sample in samples]
        switch_counts[key] = count_vertical_switches(
            times,
            heights,
            threshold=EDGE_HEIGHT_THRESHOLD,
            return_window=RETURN_WINDOW_SECONDS,
        )
        variations[key] = consecutive_variations(samples)

    flat_keys = flat_seen_keys - nonflat_keys
    flat_variations = [
        variation
        for key in flat_keys
        for variation in variations.get(key, [])
    ]
    serialized_switch_counts = {
        world_key_to_string(key): switch_counts[key] for key in sorted(switch_counts)
    }
    serialized_variations = {
        world_key_to_string(key): variations[key] for key in sorted(variations)
    }

    metrics = {
        "edge_keys": key_list(edge_keys),
        "edge_transition_width_p95_cells": percentile(
            transition_widths, 0.95
        ),
        "flat_keys": key_list(flat_keys),
        "flat_variation_p95": percentile(flat_variations, 0.95),
        "frame_count": frame_count,
        "mapper_cpu_seconds": float(mapper_cpu_seconds),
        "mean_finite_cell_coverage": sum(frame_coverages) / len(frame_coverages),
        "multimodal_cell_count": multimodal_cell_count,
        "multimodal_separation_count": len(multimodal_separations),
        "multimodal_separation_max": max(multimodal_separations, default=0.0),
        "multimodal_separation_mean": (
            sum(multimodal_separations) / len(multimodal_separations)
            if multimodal_separations
            else 0.0
        ),
        "multimodal_separation_p50": percentile(
            multimodal_separations, 0.50
        ),
        "multimodal_separation_p95": percentile(
            multimodal_separations, 0.95
        ),
        "multimodal_world_key_count": len(multimodal_keys),
        "switch_counts_by_key": serialized_switch_counts,
        "temporal_variation_by_key": serialized_variations,
        "vertical_switch_count": sum(
            switch_counts.get(key, 0) for key in edge_keys
        ),
    }
    return metrics


def load_metrics(path):
    metrics_path = Path(path)
    try:
        metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as error:
        raise ValueError(f"invalid metrics JSON {metrics_path}: {error}") from error
    if not isinstance(metrics, dict):
        raise ValueError(f"metrics file must contain a JSON object: {metrics_path}")
    return metrics


def metric_world_keys(metrics, field):
    try:
        keys = {
            (int(world_key[0]), int(world_key[1]))
            for world_key in metrics[field]
        }
    except (KeyError, TypeError, ValueError, IndexError) as error:
        raise ValueError(f"invalid {field} in metrics file") from error
    return keys


def numeric_metric(metrics, field):
    try:
        value = float(metrics[field])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"invalid numeric metric {field}") from error
    if not math.isfinite(value):
        raise ValueError(f"metric {field} must be finite")
    return value


def sum_switches(metrics, world_keys):
    try:
        per_key = metrics["switch_counts_by_key"]
        return sum(
            float(per_key.get(world_key_to_string(key), 0.0))
            for key in world_keys
        )
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError("invalid switch_counts_by_key in metrics file") from error


def variation_for_keys(metrics, world_keys):
    try:
        per_key = metrics["temporal_variation_by_key"]
        values = []
        for key in world_keys:
            serialized_key = world_key_to_string(key)
            if serialized_key not in per_key:
                continue
            key_variations = per_key[serialized_key]
            if not isinstance(key_variations, list):
                key_variations = [key_variations]
            values.extend(float(value) for value in key_variations)
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(
            "invalid temporal_variation_by_key in metrics file"
        ) from error
    return percentile(values, 0.95)


def missing_variation_keys(metrics, world_keys):
    try:
        per_key = metrics["temporal_variation_by_key"]
        if not isinstance(per_key, dict):
            raise TypeError("temporal_variation_by_key must be an object")
    except (KeyError, TypeError) as error:
        raise ValueError(
            "invalid temporal_variation_by_key in metrics file"
        ) from error
    return sorted(
        key for key in world_keys if world_key_to_string(key) not in per_key
    )


def mean(values):
    values = list(values)
    if not values:
        raise ValueError("cannot calculate a mean without values")
    return sum(values) / len(values)


def compare_values(reference, candidate, path="$"):
    if isinstance(reference, bool) or isinstance(candidate, bool):
        if reference is candidate:
            return None
        return f"{path}: {candidate!r} != {reference!r}"
    if isinstance(reference, (int, float)) and isinstance(
        candidate, (int, float)
    ):
        if math.isfinite(float(reference)) and math.isfinite(float(candidate)):
            if abs(float(reference) - float(candidate)) <= COMPARISON_TOLERANCE:
                return None
        return f"{path}: {candidate!r} != {reference!r}"
    if type(reference) is not type(candidate):
        return (
            f"{path}: type {type(candidate).__name__} != "
            f"{type(reference).__name__}"
        )
    if isinstance(reference, dict):
        if set(reference) != set(candidate):
            return f"{path}: object keys differ"
        for key in sorted(reference):
            if key == "mapper_cpu_seconds" and path == "$":
                continue
            mismatch = compare_values(
                reference[key], candidate[key], f"{path}.{key}"
            )
            if mismatch:
                return mismatch
        return None
    if isinstance(reference, list):
        if len(reference) != len(candidate):
            return f"{path}: list lengths differ"
        for index, (reference_item, candidate_item) in enumerate(
            zip(reference, candidate)
        ):
            mismatch = compare_values(
                reference_item, candidate_item, f"{path}[{index}]"
            )
            if mismatch:
                return mismatch
        return None
    if reference != candidate:
        return f"{path}: {candidate!r} != {reference!r}"
    return None


def denominator_ratio(
    candidate, baseline, metric_name, zero_value, diagnostics
):
    if baseline == 0.0:
        if candidate == 0.0:
            return zero_value
        diagnostics.append(
            f"{metric_name}: baseline is zero but candidate is {candidate}"
        )
        return None
    return candidate / baseline


def compare_metrics(baseline, candidates):
    if len(candidates) != 3:
        raise ValueError("comparison requires exactly three candidate metrics")

    union_edge_keys = metric_world_keys(baseline, "edge_keys")
    for candidate in candidates:
        union_edge_keys.update(metric_world_keys(candidate, "edge_keys"))
    baseline_flat_keys = metric_world_keys(baseline, "flat_keys")
    diagnostics = []
    missing_baseline_flat_variation = False
    for candidate_index, candidate in enumerate(candidates, start=1):
        missing_keys = missing_variation_keys(candidate, baseline_flat_keys)
        if missing_keys:
            missing_baseline_flat_variation = True
        for key in missing_keys:
            diagnostics.append(
                "candidate "
                f"{candidate_index} missing temporal variation for baseline flat key "
                f"{world_key_to_string(key)}"
            )

    baseline_switches = sum_switches(baseline, union_edge_keys)
    candidate_switches = mean(
        sum_switches(candidate, union_edge_keys) for candidate in candidates
    )
    baseline_coverage = numeric_metric(
        baseline, "mean_finite_cell_coverage"
    )
    candidate_coverage = mean(
        numeric_metric(candidate, "mean_finite_cell_coverage")
        for candidate in candidates
    )
    baseline_flat_p95 = variation_for_keys(baseline, baseline_flat_keys)
    candidate_flat_p95 = mean(
        variation_for_keys(candidate, baseline_flat_keys)
        for candidate in candidates
    )
    if missing_baseline_flat_variation:
        candidate_flat_p95 = max(candidate_flat_p95, baseline_flat_p95)
    baseline_cpu_seconds = numeric_metric(baseline, "mapper_cpu_seconds")
    candidate_cpu_seconds = mean(
        numeric_metric(candidate, "mapper_cpu_seconds")
        for candidate in candidates
    )
    edge_transition_width_cells = max(
        numeric_metric(candidate, "edge_transition_width_p95_cells")
        for candidate in candidates
    )

    switch_ratio = denominator_ratio(
        candidate_switches,
        baseline_switches,
        "vertical_switch_reduction",
        0.0,
        diagnostics,
    )
    vertical_switch_reduction = (
        None if switch_ratio is None else 1.0 - switch_ratio
    )
    finite_coverage_ratio = denominator_ratio(
        candidate_coverage,
        baseline_coverage,
        "finite_coverage_ratio",
        1.0,
        diagnostics,
    )
    flat_variation_ratio = denominator_ratio(
        candidate_flat_p95,
        baseline_flat_p95,
        "flat_variation_ratio",
        1.0,
        diagnostics,
    )
    cpu_ratio = denominator_ratio(
        candidate_cpu_seconds,
        baseline_cpu_seconds,
        "cpu_increase_ratio",
        1.0,
        diagnostics,
    )
    cpu_increase_ratio = None if cpu_ratio is None else cpu_ratio - 1.0

    determinism_mismatches = [
        mismatch
        for candidate in candidates[1:]
        if (mismatch := compare_values(candidates[0], candidate)) is not None
    ]
    diagnostics.extend(
        f"candidate determinism mismatch: {mismatch}"
        for mismatch in determinism_mismatches
    )
    candidate_runs_identical = not determinism_mismatches

    report = {
        "candidate_runs_identical": candidate_runs_identical,
        "cpu_increase_ratio": cpu_increase_ratio,
        "edge_transition_width_cells": edge_transition_width_cells,
        "finite_coverage_ratio": finite_coverage_ratio,
        "flat_variation_ratio": flat_variation_ratio,
        "vertical_switch_reduction": vertical_switch_reduction,
    }

    checks = (
        (
            vertical_switch_reduction is not None
            and vertical_switch_reduction
            >= 0.80 - COMPARISON_TOLERANCE,
            "vertical_switch_reduction must be >= 0.80",
        ),
        (
            finite_coverage_ratio is not None
            and finite_coverage_ratio
            >= 0.95 - COMPARISON_TOLERANCE,
            "finite_coverage_ratio must be >= 0.95",
        ),
        (
            flat_variation_ratio is not None
            and flat_variation_ratio
            <= 1.00 + COMPARISON_TOLERANCE,
            "flat_variation_ratio must be <= 1.00",
        ),
        (
            edge_transition_width_cells <= 1.0 + COMPARISON_TOLERANCE,
            "edge_transition_width_cells must be <= 1",
        ),
        (
            candidate_runs_identical,
            "candidate_runs_identical must be true",
        ),
        (
            cpu_increase_ratio is not None
            and cpu_increase_ratio <= 0.25 + COMPARISON_TOLERANCE,
            "cpu_increase_ratio must be <= 0.25",
        ),
    )
    diagnostics.extend(message for passed, message in checks if not passed)
    return report, diagnostics


def json_text(value):
    return json.dumps(
        value, allow_nan=False, indent=2, sort_keys=True
    ) + "\n"


def write_json(path, value):
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = output_path.with_name(f".{output_path.name}.tmp")
    temporary_path.write_text(json_text(value), encoding="utf-8")
    os.replace(temporary_path, output_path)


def run_self_test():
    times = [0.0, 0.1, 0.2, 0.3]
    stable = [0.20, 0.21, 0.20, 0.20]
    switching = [0.20, -0.30, 0.20, -0.30]
    no_return = [0.20, -0.30, -0.31, -0.30]

    assert count_vertical_switches(
        times, stable, threshold=0.08, return_window=1.0
    ) == 0
    assert count_vertical_switches(
        times, switching, threshold=0.08, return_window=1.0
    ) == 2
    assert count_vertical_switches(
        times, no_return, threshold=0.08, return_window=1.0
    ) == 0
    assert finite_coverage([1.0, float("nan"), 2.0]) == 2 / 3
    assert percentile([0.0, 10.0], 0.95) == 9.5
    variation_samples = [
        (0.0, 0.0, float("nan"), 1.0, float("nan")),
        (0.1, 1.0, float("nan"), 1.0, float("nan")),
        (0.2, 3.0, float("nan"), 1.0, float("nan")),
    ]
    assert consecutive_variations(variation_samples) == [1.0, 2.0]

    array = SimpleNamespace(
        layout=SimpleNamespace(
            data_offset=0,
            dim=[
                SimpleNamespace(label="column_index", size=3, stride=6),
                SimpleNamespace(label="row_index", size=2, stride=2),
            ],
        ),
        data=[0.0, 10.0, 1.0, 11.0, 2.0, 12.0],
    )
    assert decode_layer(array, outer_start_index=1, inner_start_index=2) == [
        [12.0, 10.0, 11.0],
        [2.0, 0.0, 1.0],
    ]
    padded_array = SimpleNamespace(
        layout=SimpleNamespace(
            data_offset=2,
            dim=[
                SimpleNamespace(label="row_index", size=2, stride=8),
                SimpleNamespace(label="column_index", size=3, stride=4),
            ],
        ),
        data=[-1.0, -1.0, 10.0, 11.0, 12.0, -1.0, 20.0, 21.0, 22.0],
    )
    assert decode_layer(
        padded_array, outer_start_index=1, inner_start_index=2
    ) == [[22.0, 20.0, 21.0], [12.0, 10.0, 11.0]]

    with tempfile.TemporaryDirectory() as directory:
        cpu_file = Path(directory) / "mapper_time.txt"
        cpu_file.write_text("1.25 0.75\n", encoding="ascii")
        assert parse_cpu_seconds(cpu_file) == 2.0

    heights = {
        (-2, 0): -0.20,
        (-1, 0): -0.20,
        (0, 0): 0.00,
        (1, 0): 0.20,
        (2, 0): 0.20,
        (0, -1): 0.00,
        (0, 1): 0.01,
    }
    assert transition_width_cells((0, 0), heights, resolution=0.04) == 1.0
    assert is_edge_cell((0, 0), {}, 0.10)
    assert multimodal_separation_for_cell(2.0, 0.10) == 0.10
    assert multimodal_separation_for_cell(2.0, float("nan")) is None

    baseline_metrics = {
        "edge_keys": [],
        "edge_transition_width_p95_cells": 0.0,
        "flat_keys": [[1, 1]],
        "mapper_cpu_seconds": 1.0,
        "mean_finite_cell_coverage": 1.0,
        "switch_counts_by_key": {},
        "temporal_variation_by_key": {"1,1": [0.01]},
    }
    missing_flat_key_candidate = dict(baseline_metrics)
    missing_flat_key_candidate["temporal_variation_by_key"] = {}
    report, diagnostics = compare_metrics(
        baseline_metrics, [missing_flat_key_candidate] * 3
    )
    assert report["flat_variation_ratio"] == 1.0
    assert diagnostics == [
        "candidate 1 missing temporal variation for baseline flat key 1,1",
        "candidate 2 missing temporal variation for baseline flat key 1,1",
        "candidate 3 missing temporal variation for baseline flat key 1,1",
    ]


def parse_args():
    parser = argparse.ArgumentParser(
        description="Analyze deterministic elevation GridMap bag replays."
    )
    parser.add_argument("bag", nargs="?")
    parser.add_argument("--cpu-seconds-file")
    parser.add_argument("--output")
    parser.add_argument(
        "--compare",
        nargs=4,
        metavar=("BASE", "CANDIDATE1", "CANDIDATE2", "CANDIDATE3"),
    )
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()

    modes = sum(
        (
            args.self_test,
            args.compare is not None,
            args.bag is not None,
        )
    )
    if modes != 1:
        parser.error("choose exactly one of BAG, --compare, or --self-test")
    if args.bag is not None and (
        args.cpu_seconds_file is None or args.output is None
    ):
        parser.error("BAG mode requires --cpu-seconds-file and --output")
    if args.bag is None and (
        args.cpu_seconds_file is not None or args.output is not None
    ):
        parser.error("--cpu-seconds-file and --output are valid only with BAG")
    return args


def main():
    args = parse_args()
    try:
        if args.self_test:
            run_self_test()
            print("analyze_elevation_bag self-test: PASS")
            return 0
        if args.compare is not None:
            baseline = load_metrics(args.compare[0])
            candidates = [
                load_metrics(path) for path in args.compare[1:]
            ]
            report, diagnostics = compare_metrics(baseline, candidates)
            print(json_text(report), end="")
            for diagnostic in diagnostics:
                print(f"FAIL: {diagnostic}", file=sys.stderr)
            return 1 if diagnostics else 0

        mapper_cpu_seconds = parse_cpu_seconds(args.cpu_seconds_file)
        metrics = analyze_bag(args.bag, mapper_cpu_seconds)
        write_json(args.output, metrics)
        print(json_text(metrics), end="")
        return 0
    except (OSError, RuntimeError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
