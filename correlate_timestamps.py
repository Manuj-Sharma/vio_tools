#!/usr/bin/env python3

"""
correlate_timestamps.py

Analyze timestamp correlation between a ROS 2 image topic and IMU topic
inside a rosbag2.

Designed for the VIO setup:

    /cam0/image_raw   sensor_msgs/Image
    /imu0             sensor_msgs/Imu

The script compares message HEADER timestamps, not rosbag recording times.

For every image:
    - find the nearest IMU timestamp
    - calculate IMU timestamp - image timestamp
    - calculate absolute time difference
    - determine whether it is within the requested threshold

It also analyzes:
    - image timestamp intervals
    - IMU timestamp intervals
    - timestamp monotonicity
    - timestamp correlation statistics

Output:
    timestamp_correlation.csv

Usage:

    python3 correlate_timestamps.py /path/to/rosbag

or:

    python3 correlate_timestamps.py /path/to/rosbag --threshold-ms 5

Optional topics:

    --image-topic /cam0/image_raw
    --imu-topic /imu0

Optional CSV output:

    --csv my_results.csv
"""

import argparse
import csv
import os
import sys

import numpy as np

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def stamp_to_seconds(stamp):
    """Convert ROS builtin_interfaces/Time to seconds."""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def get_timestamps(bag_path, topic_name, msg_type_name):
    """
    Read all header timestamps for one topic from a rosbag2.

    Returns:
        numpy array of timestamps in seconds.
    """

    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path,
        storage_id="sqlite3"
    )

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr"
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    target_type = get_message(msg_type_name)

    timestamps = []

    while reader.has_next():
        topic, data, bag_timestamp = reader.read_next()

        if topic != topic_name:
            continue

        msg = deserialize_message(data, target_type)

        if not hasattr(msg, "header"):
            raise RuntimeError(
                f"Topic {topic_name} does not contain a message header."
            )

        timestamps.append(stamp_to_seconds(msg.header.stamp))

    return np.asarray(timestamps, dtype=np.float64)


def print_interval_statistics(name, timestamps):
    """Print statistics for timestamp intervals."""

    if len(timestamps) < 2:
        print(f"\n{name}: not enough samples for interval analysis.")
        return

    dt = np.diff(timestamps)

    print(f"\n{name} timestamp intervals:")
    print(f"  Samples       : {len(timestamps):d}")
    print(f"  Mean          : {np.mean(dt) * 1000.0:.6f} ms")
    print(f"  Median        : {np.median(dt) * 1000.0:.6f} ms")
    print(f"  Std deviation : {np.std(dt) * 1000.0:.6f} ms")
    print(f"  Minimum       : {np.min(dt) * 1000.0:.6f} ms")
    print(f"  Maximum       : {np.max(dt) * 1000.0:.6f} ms")

    negative = np.sum(dt < 0.0)
    zero = np.sum(dt == 0.0)

    print(f"  Backwards     : {negative:d}")
    print(f"  Identical     : {zero:d}")


def print_basic_timestamp_info(name, timestamps):
    """Print first/last timestamp and duration."""

    if len(timestamps) == 0:
        print(f"\n{name}: NO SAMPLES")
        return

    print(f"\n{name}:")
    print(f"  Samples       : {len(timestamps):d}")
    print(f"  First stamp   : {timestamps[0]:.9f}")
    print(f"  Last stamp    : {timestamps[-1]:.9f}")
    print(f"  Duration      : {timestamps[-1] - timestamps[0]:.6f} s")


# ---------------------------------------------------------------------------
# Main correlation analysis
# ---------------------------------------------------------------------------

def correlate(image_ts, imu_ts, threshold_ms, csv_path):
    """
    Correlate every image timestamp with the nearest IMU timestamp.
    """

    if len(image_ts) == 0:
        raise RuntimeError("No image timestamps found.")

    if len(imu_ts) == 0:
        raise RuntimeError("No IMU timestamps found.")

    # Sort IMU timestamps for nearest-neighbour search.
    #
    # We retain the original order separately only for interval analysis.
    imu_sorted = np.sort(imu_ts)

    results = []

    for image_index, t_image in enumerate(image_ts):

        # Position where image timestamp would be inserted.
        idx = np.searchsorted(imu_sorted, t_image)

        candidates = []

        if idx > 0:
            candidates.append(idx - 1)

        if idx < len(imu_sorted):
            candidates.append(idx)

        if not candidates:
            continue

        # Select closest IMU timestamp.
        nearest_idx = min(
            candidates,
            key=lambda i: abs(imu_sorted[i] - t_image)
        )

        t_imu = imu_sorted[nearest_idx]

        signed_dt = t_imu - t_image
        abs_dt = abs(signed_dt)

        within = abs_dt <= threshold_ms * 1e-3

        results.append(
            (
                image_index,
                t_image,
                t_imu,
                signed_dt,
                abs_dt,
                within
            )
        )

    if not results:
        raise RuntimeError("Could not correlate image and IMU timestamps.")

    signed = np.asarray([r[3] for r in results])
    absolute = np.asarray([r[4] for r in results])

    threshold_sec = threshold_ms * 1e-3

    within_count = np.sum(absolute <= threshold_sec)

    print("\n" + "=" * 72)
    print("IMAGE / IMU TIMESTAMP CORRELATION")
    print("=" * 72)

    print(f"\nImage samples correlated : {len(results)}")
    print(f"IMU samples available    : {len(imu_ts)}")
    print(f"Threshold                : ±{threshold_ms:.3f} ms")

    print("\nNearest IMU - Image timestamp:")
    print(f"  Mean signed difference  : {np.mean(signed) * 1000.0:.6f} ms")
    print(f"  Median signed difference: {np.median(signed) * 1000.0:.6f} ms")
    print(f"  Std deviation           : {np.std(signed) * 1000.0:.6f} ms")
    print(f"  Minimum                 : {np.min(signed) * 1000.0:.6f} ms")
    print(f"  Maximum                 : {np.max(signed) * 1000.0:.6f} ms")

    print("\nAbsolute nearest-sample difference:")
    print(f"  Mean                    : {np.mean(absolute) * 1000.0:.6f} ms")
    print(f"  Median                  : {np.median(absolute) * 1000.0:.6f} ms")
    print(f"  Minimum                 : {np.min(absolute) * 1000.0:.6f} ms")
    print(f"  Maximum                 : {np.max(absolute) * 1000.0:.6f} ms")

    percentage = 100.0 * within_count / len(results)

    print("\nThreshold result:")
    print(f"  Within ±{threshold_ms:.3f} ms : "
          f"{within_count}/{len(results)} "
          f"({percentage:.2f} %)")

    # -----------------------------------------------------------------------
    # Percentiles
    # -----------------------------------------------------------------------

    print("\nAbsolute difference percentiles:")

    for p in [50, 75, 90, 95, 99, 99.9]:
        value = np.percentile(absolute, p) * 1000.0
        print(f"  {p:5.1f} %                 : {value:.6f} ms")

    # -----------------------------------------------------------------------
    # Positive / negative side
    # -----------------------------------------------------------------------

    before = np.sum(signed < 0.0)
    after = np.sum(signed > 0.0)
    exact = np.sum(signed == 0.0)

    print("\nNearest IMU position relative to image:")
    print(f"  IMU before image         : {before}")
    print(f"  IMU after image          : {after}")
    print(f"  Exactly equal            : {exact}")

    # -----------------------------------------------------------------------
    # Write CSV
    # -----------------------------------------------------------------------

    with open(csv_path, "w", newline="") as f:

        writer = csv.writer(f)

        writer.writerow([
            "image_index",
            "image_timestamp_sec",
            "nearest_imu_timestamp_sec",
            "signed_difference_sec",
            "absolute_difference_sec",
            "signed_difference_ms",
            "absolute_difference_ms",
            "within_threshold"
        ])

        for image_index, t_image, t_imu, signed_dt, abs_dt, within in results:

            writer.writerow([
                image_index,
                f"{t_image:.9f}",
                f"{t_imu:.9f}",
                f"{signed_dt:.9f}",
                f"{abs_dt:.9f}",
                f"{signed_dt * 1000.0:.6f}",
                f"{abs_dt * 1000.0:.6f}",
                within
            ])

    print(f"\nDetailed results written to:")
    print(f"  {os.path.abspath(csv_path)}")

    # -----------------------------------------------------------------------
    # Worst cases
    # -----------------------------------------------------------------------

    print("\nWorst 10 image/IMU correlations:")

    worst_indices = np.argsort(absolute)[-10:][::-1]

    for rank, result_index in enumerate(worst_indices, start=1):

        image_index, t_image, t_imu, signed_dt, abs_dt, within = results[result_index]

        print(
            f"  {rank:2d}: image {image_index:6d} | "
            f"image={t_image:.6f} | "
            f"imu={t_imu:.6f} | "
            f"dt={signed_dt * 1000.0:+.3f} ms"
        )

    print("\n" + "=" * 72)


# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------

def main():

    parser = argparse.ArgumentParser(
        description="Correlate ROS 2 image and IMU header timestamps."
    )

    parser.add_argument(
        "bag",
        help="Path to rosbag2 directory"
    )

    parser.add_argument(
        "--image-topic",
        default="/cam0/image_raw",
        help="Image topic (default: /cam0/image_raw)"
    )

    parser.add_argument(
        "--imu-topic",
        default="/imu0",
        help="IMU topic (default: /imu0)"
    )

    parser.add_argument(
        "--threshold-ms",
        type=float,
        default=5.0,
        help="Timestamp correlation threshold in milliseconds "
             "(default: 5 ms)"
    )

    parser.add_argument(
        "--csv",
        default="timestamp_correlation.csv",
        help="Output CSV filename"
    )

    args = parser.parse_args()

    if not os.path.exists(args.bag):
        print(f"ERROR: bag does not exist: {args.bag}")
        sys.exit(1)

    print("=" * 72)
    print("Reading ROS 2 bag")
    print("=" * 72)
    print(f"Bag          : {args.bag}")
    print(f"Image topic  : {args.image_topic}")
    print(f"IMU topic    : {args.imu_topic}")

    print("\nReading image timestamps...")

    image_ts = get_timestamps(
        args.bag,
        args.image_topic,
        "sensor_msgs/msg/Image"
    )

    print(f"Found {len(image_ts)} image messages.")

    print("\nReading IMU timestamps...")

    imu_ts = get_timestamps(
        args.bag,
        args.imu_topic,
        "sensor_msgs/msg/Imu"
    )

    print(f"Found {len(imu_ts)} IMU messages.")

    # Basic information.
    print_basic_timestamp_info("IMAGE", image_ts)
    print_basic_timestamp_info("IMU", imu_ts)

    # Interval analysis.
    print_interval_statistics("IMAGE", image_ts)
    print_interval_statistics("IMU", imu_ts)

    # Correlation.
    correlate(
        image_ts,
        imu_ts,
        args.threshold_ms,
        args.csv
    )


if __name__ == "__main__":
    main()

