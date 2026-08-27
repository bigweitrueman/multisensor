#!/usr/bin/env python3
"""Validate live PointCloud2 frames without running FAST-LIVO2."""

import argparse
import collections
import datetime
import json
import math
import time
import zlib

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


FIELD_DTYPES = {
    PointField.INT8: "i1",
    PointField.UINT8: "u1",
    PointField.INT16: "i2",
    PointField.UINT16: "u2",
    PointField.INT32: "i4",
    PointField.UINT32: "u4",
    PointField.FLOAT32: "f4",
    PointField.FLOAT64: "f8",
}


def stats(values):
    array = np.asarray(values, dtype=np.float64)
    if array.size == 0:
        return {}
    return {
        "min": float(array.min()),
        "p01": float(np.percentile(array, 1)),
        "median": float(np.median(array)),
        "mean": float(array.mean()),
        "p99": float(np.percentile(array, 99)),
        "max": float(array.max()),
        "stddev": float(array.std()),
    }


class FrameAnalyzer(Node):
    def __init__(self, topic, report_every):
        super().__init__("vanjee_frame_analyzer")
        self.report_every = report_every
        self.frames = 0
        self.first_arrival_ns = None
        self.last_arrival_ns = None
        self.last_stamp_ns = None
        self.last_crc = None
        self.layout = None
        self.field_names = []
        self.arrival_dt_ms = []
        self.header_dt_ms = []
        self.epoch_offsets_s = []
        self.total_points = []
        self.finite_points = []
        self.finite_ratios = []
        self.z_spans = []
        self.ring_counts = []
        self.point_time_spans_ms = []
        self.anomalies = collections.Counter()
        self.anomaly_frames = []

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(PointCloud2, topic, self.on_cloud, qos)

    def build_dtype(self, message):
        names = []
        formats = []
        offsets = []
        endian = ">" if message.is_bigendian else "<"
        for field in message.fields:
            if field.count != 1 or field.datatype not in FIELD_DTYPES:
                continue
            names.append(field.name)
            formats.append(np.dtype(endian + FIELD_DTYPES[field.datatype]))
            offsets.append(field.offset)
        return np.dtype(
            {
                "names": names,
                "formats": formats,
                "offsets": offsets,
                "itemsize": message.point_step,
            }
        )

    def points(self, message):
        dtype = self.build_dtype(message)
        packed_size = message.width * message.point_step
        rows = []
        raw = memoryview(message.data)
        for row in range(message.height):
            start = row * message.row_step
            rows.append(
                np.frombuffer(
                    raw[start : start + packed_size],
                    dtype=dtype,
                    count=message.width,
                )
            )
        return np.concatenate(rows) if len(rows) > 1 else rows[0]

    def record_anomaly(self, frame_index, name, detail):
        self.anomalies[name] += 1
        if len(self.anomaly_frames) < 100:
            self.anomaly_frames.append(
                {"frame": frame_index, "type": name, "detail": detail}
            )

    def on_cloud(self, message):
        arrival_ns = time.monotonic_ns()
        wall_ns = time.time_ns()
        stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        frame_index = self.frames
        self.frames += 1

        if self.first_arrival_ns is None:
            self.first_arrival_ns = arrival_ns
            self.layout = {
                "width": int(message.width),
                "height": int(message.height),
                "point_step": int(message.point_step),
                "row_step": int(message.row_step),
                "frame_id": message.header.frame_id,
            }
            self.field_names = [field.name for field in message.fields]
        else:
            arrival_dt = (arrival_ns - self.last_arrival_ns) / 1_000_000.0
            header_dt = (stamp_ns - self.last_stamp_ns) / 1_000_000.0
            self.arrival_dt_ms.append(arrival_dt)
            self.header_dt_ms.append(header_dt)
            if arrival_dt < 50.0 or arrival_dt > 150.0:
                self.record_anomaly(frame_index, "arrival_period", f"{arrival_dt:.3f} ms")
            if header_dt <= 0.0:
                self.record_anomaly(frame_index, "header_nonmonotonic", f"{header_dt:.3f} ms")
            elif header_dt < 50.0 or header_dt > 150.0:
                self.record_anomaly(frame_index, "header_period", f"{header_dt:.3f} ms")

        self.last_arrival_ns = arrival_ns
        self.last_stamp_ns = stamp_ns
        self.epoch_offsets_s.append((stamp_ns - wall_ns) / 1_000_000_000.0)

        points = self.points(message)
        total = int(points.size)
        self.total_points.append(total)
        if total == 0:
            self.record_anomaly(frame_index, "empty_frame", "zero points")
            return
        if total != self.layout["width"] * self.layout["height"]:
            self.record_anomaly(frame_index, "layout_size", f"{total} points")

        finite = (
            np.isfinite(points["x"])
            & np.isfinite(points["y"])
            & np.isfinite(points["z"])
        )
        valid = points[finite]
        finite_count = int(valid.size)
        finite_ratio = finite_count / total
        self.finite_points.append(finite_count)
        self.finite_ratios.append(finite_ratio)
        if finite_count < 1000:
            self.record_anomaly(frame_index, "too_few_finite_points", str(finite_count))
            return

        zero_xyz = (
            (valid["x"] == 0.0) & (valid["y"] == 0.0) & (valid["z"] == 0.0)
        )
        if bool(np.all(zero_xyz)):
            self.record_anomaly(frame_index, "all_zero_xyz", str(finite_count))

        z_span = float(valid["z"].max() - valid["z"].min())
        self.z_spans.append(z_span)
        if z_span < 0.1:
            self.record_anomaly(frame_index, "flat_z", f"{z_span:.6f} m")

        if "ring" in points.dtype.names:
            rings = np.unique(valid["ring"])
            ring_count = int(rings.size)
            self.ring_counts.append(ring_count)
            if ring_count < 32:
                self.record_anomaly(
                    frame_index,
                    "missing_rings",
                    ",".join(str(int(value)) for value in rings),
                )

        if "timestamp" in points.dtype.names:
            point_times = valid["timestamp"]
            finite_time = point_times[np.isfinite(point_times)]
            if finite_time.size != valid.size:
                self.record_anomaly(
                    frame_index,
                    "invalid_point_timestamps",
                    f"{valid.size - finite_time.size} non-finite",
                )
            if finite_time.size:
                self.point_time_spans_ms.append(
                    float((finite_time.max() - finite_time.min()) * 1000.0)
                )

        crc = zlib.crc32(message.data)
        if self.last_crc is not None and crc == self.last_crc:
            self.record_anomaly(frame_index, "duplicate_payload", f"crc32={crc:08x}")
        self.last_crc = crc

        if self.frames == 1 or self.frames % self.report_every == 0:
            self.get_logger().info(
                f"frame={self.frames} total={total} finite={finite_count} "
                f"ratio={finite_ratio:.3f} z_span={z_span:.3f} "
                f"rings={self.ring_counts[-1] if self.ring_counts else 0}"
            )

    def summary(self):
        duration_s = 0.0
        if self.first_arrival_ns is not None and self.last_arrival_ns is not None:
            duration_s = (self.last_arrival_ns - self.first_arrival_ns) / 1e9
        first_stamp = None
        if self.last_stamp_ns is not None and self.header_dt_ms:
            first_stamp_ns = self.last_stamp_ns - int(sum(self.header_dt_ms) * 1_000_000)
            first_stamp = datetime.datetime.fromtimestamp(
                first_stamp_ns / 1e9, tz=datetime.timezone.utc
            ).isoformat()
        return {
            "frames": self.frames,
            "duration_s": duration_s,
            "arrival_rate_hz": (self.frames - 1) / duration_s if duration_s > 0 else 0.0,
            "layout": self.layout,
            "fields": self.field_names,
            "first_header_stamp_utc": first_stamp,
            "arrival_period_ms": stats(self.arrival_dt_ms),
            "header_period_ms": stats(self.header_dt_ms),
            "header_minus_system_time_s": stats(self.epoch_offsets_s),
            "total_points": stats(self.total_points),
            "finite_points": stats(self.finite_points),
            "finite_ratio": stats(self.finite_ratios),
            "z_span_m": stats(self.z_spans),
            "ring_count": stats(self.ring_counts),
            "point_timestamp_span_ms": stats(self.point_time_spans_ms),
            "anomaly_counts": dict(self.anomalies),
            "anomaly_frames": self.anomaly_frames,
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/vanjee_points722")
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument("--report-every", type=int, default=100)
    parser.add_argument("--output", default="/tmp/vanjee_frame_analysis.json")
    args = parser.parse_args()

    rclpy.init()
    node = FrameAnalyzer(args.topic, args.report_every)
    wait_deadline = time.monotonic() + args.timeout
    while rclpy.ok() and node.frames == 0 and time.monotonic() < wait_deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.frames == 0:
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(f"no frames received from {args.topic}")

    deadline = time.monotonic() + args.duration
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)

    summary = node.summary()
    with open(args.output, "w", encoding="ascii") as output:
        json.dump(summary, output, indent=2, sort_keys=True)
        output.write("\n")
    print(json.dumps(summary, indent=2, sort_keys=True))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
