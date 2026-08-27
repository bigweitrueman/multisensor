#!/usr/bin/env python3
import argparse
import statistics
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header


def stamp_ns(msg):
    header = getattr(msg, "header", msg)
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def period_stats(stamps):
    if len(stamps) < 2:
        return "insufficient"
    values = [(b - a) / 1e6 for a, b in zip(stamps, stamps[1:])]
    nonpositive = sum(v <= 0 for v in values)
    return (
        f"median={statistics.median(values):.3f} ms "
        f"min={min(values):.3f} ms max={max(values):.3f} ms "
        f"nonpositive={nonpositive}"
    )


def nearest_deltas_ms(source, reference):
    if not source or not reference:
        return []
    result = []
    j = 0
    for value in source:
        while j + 1 < len(reference) and abs(reference[j + 1] - value) < abs(reference[j] - value):
            j += 1
        result.append((value - reference[j]) / 1e6)
    return result


class SyncCheck(Node):
    def __init__(self, duration_s):
        super().__init__("atlas_sync_check")
        lidar_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )
        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.stamps = {"lidar": [], "cam1": [], "cam2": []}
        self.start = time.monotonic()
        self.duration_s = duration_s
        self.create_subscription(PointCloud2, "/vanjee_points722", self.on_lidar, lidar_qos)
        self.create_subscription(Header, "/imx586/cam1/header", lambda m: self.on_image("cam1", m), camera_qos)
        self.create_subscription(Header, "/imx586/cam2/header", lambda m: self.on_image("cam2", m), camera_qos)

    def on_image(self, name, msg):
        self.stamps[name].append(stamp_ns(msg))

    def on_lidar(self, msg):
        self.stamps["lidar"].append(stamp_ns(msg))


def describe(name, values):
    print(f"{name}: frames={len(values)} period {period_stats(values)}")


def describe_delta(name, values):
    if not values:
        print(f"{name}: no pairs")
        return
    mean = statistics.fmean(values)
    std = statistics.pstdev(values) if len(values) > 1 else 0.0
    print(
        f"{name}: pairs={len(values)} median={statistics.median(values):.3f} ms "
        f"mean={mean:.3f} ms std={std:.3f} ms "
        f"min={min(values):.3f} ms max={max(values):.3f} ms "
        f"abs_gt_20ms={sum(abs(v) > 20 for v in values)}"
    )


def main():
    parser = argparse.ArgumentParser(description="Check Atlas camera/LiDAR ROS timestamp synchronization.")
    parser.add_argument("--duration", type=float, default=60.0)
    args = parser.parse_args()
    rclpy.init()
    node = SyncCheck(args.duration)
    try:
        while rclpy.ok() and time.monotonic() - node.start < node.duration_s:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        lidar = node.stamps["lidar"]
        cam1 = node.stamps["cam1"]
        cam2 = node.stamps["cam2"]
        print(f"duration={time.monotonic() - node.start:.3f} s qos=lidar-reliable,camera-best-effort")
        describe("lidar", lidar)
        describe("cam1", cam1)
        describe("cam2", cam2)
        describe_delta("cam1-lidar", nearest_deltas_ms(cam1, lidar))
        describe_delta("cam2-lidar", nearest_deltas_ms(cam2, lidar))
        describe_delta("cam1-cam2", nearest_deltas_ms(cam1, cam2))
        common = set(cam1).intersection(cam2)
        print(
            f"cam1-cam2-exact: common={len(common)} "
            f"only_cam1={len(set(cam1) - common)} only_cam2={len(set(cam2) - common)}"
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
