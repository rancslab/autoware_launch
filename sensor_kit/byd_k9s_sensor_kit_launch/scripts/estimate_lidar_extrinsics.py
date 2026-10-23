#!/usr/bin/env python3
"""First-pass rotation estimate for the two K9S front-corner lidars (not a calibration).

For each lidar, read a few raw clouds (sensor frame) and:
  1. fit the ground plane -> lidar height and the roll/pitch that levels it. The lidars are mounted
     upside down, so the ground appears at +z and the rotation includes a ~180 deg roll;
  2. find the largest azimuth sector with no ground returns, which is the part of the view the bus
     body blocks. It should be centred on vehicle azimuth 225 deg for the front-left lidar (rear-right
     of it) and 135 deg for the front-right lidar. That gives the yaw.
Positions are not estimated; they come from the tape measurements in sensor_kit_calibration.yaml.

Run with the sensing stack (or the bare drivers) up:
  python3 estimate_lidar_extrinsics.py [--frames 10]
"""
import argparse

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

LIDARS = {
    # name: (raw topic, vehicle azimuth of the centre of the sector blocked by the bus, deg)
    "lidar_front_left": ("/sensing/lidar/front_left/pointcloud_raw_ex", 225.0),
    "lidar_front_right": ("/sensing/lidar/front_right/pointcloud_raw_ex", 135.0),
}


def grab(topics, frames):
    rclpy.init()
    node = rclpy.create_node("estimate_lidar_extrinsics")
    clouds = {name: [] for name in topics}

    def callback(msg, name):
        if len(clouds[name]) < frames:
            a = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            clouds[name].append(np.stack([a["x"], a["y"], a["z"]], axis=1))

    for name, topic in topics.items():
        node.create_subscription(
            PointCloud2, topic, lambda m, n=name: callback(m, n), qos_profile_sensor_data
        )
    while rclpy.ok() and any(len(c) < frames for c in clouds.values()):
        rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()
    return {name: np.concatenate(c) for name, c in clouds.items()}


def fit_ground(points, rng):
    """RANSAC + SVD plane on the ground rings. Returns (unit normal pointing at the ground, distance)."""
    r = np.linalg.norm(points[:, :2], axis=1)
    cand = points[(r > 3) & (r < 20) & (np.abs(points[:, 2]) > 0.5)]
    best_count, best = 0, None
    for _ in range(3000):
        s = cand[rng.choice(len(cand), 3, replace=False)]
        n = np.cross(s[1] - s[0], s[2] - s[0])
        if np.linalg.norm(n) < 1e-6:
            continue
        n /= np.linalg.norm(n)
        if abs(n[2]) < np.cos(np.radians(35)):  # ground must be within 35 deg of the sensor z axis
            continue
        count = (np.abs((cand - s[0]) @ n) < 0.04).sum()
        if count > best_count:
            best_count, best = count, (n, s[0])
    n, p0 = best
    inliers = cand[np.abs((cand - p0) @ n) < 0.04]
    centre = inliers.mean(axis=0)
    n = np.linalg.svd(inliers - centre, full_matrices=False)[2][2]
    if n @ centre < 0:  # orient the normal from the lidar towards the ground
        n = -n
    return n, n @ centre, len(inliers)


def blocked_sector(points, height):
    """Largest run of 1-deg azimuth bins without ground returns (levelled frame). Returns (start, width)."""
    r = np.linalg.norm(points[:, :2], axis=1)
    ground = points[(r > 3) & (r < 15) & (np.abs(points[:, 2] + height) < 0.3)]
    az = np.degrees(np.arctan2(ground[:, 1], ground[:, 0])) % 360
    empty = np.histogram(az, bins=360, range=(0, 360))[0] < 5
    best, run = (0, 0), 0
    for i, e in enumerate(np.r_[empty, empty]):
        run = run + 1 if e else 0
        if best[0] < run <= 360:
            best = (run, i)
    width, end = best
    return (end - width + 1) % 360, width


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--frames", type=int, default=10, help="clouds per lidar")
    args = parser.parse_args()
    rng = np.random.default_rng(0)
    clouds = grab({name: topic for name, (topic, _) in LIDARS.items()}, args.frames)

    for name, (_, blocked_centre) in LIDARS.items():
        n, height, count = fit_ground(clouds[name], rng)
        # level: 180 deg roll (upside-down mount), then the smallest rotation taking the ground normal to -z
        flip = Rotation.from_euler("x", 180, degrees=True)
        n1 = flip.apply(n)
        axis = np.cross(n1, [0, 0, -1])
        angle = np.arctan2(np.linalg.norm(axis), n1 @ [0, 0, -1])
        level = Rotation.from_rotvec(axis / np.linalg.norm(axis) * angle) * flip
        start, width = blocked_sector(level.apply(clouds[name]), height)
        yaw = (blocked_centre - (start + width / 2) + 180) % 360 - 180
        roll, pitch, yaw_out = (Rotation.from_euler("z", yaw, degrees=True) * level).as_euler("xyz")
        print(
            f"{name}: ground {height:.3f} m away ({count} pts), blocked sector {width} deg wide\n"
            f"    roll: {roll:.4f}\n    pitch: {pitch:.4f}\n    yaw: {yaw_out:.4f}"
        )


if __name__ == "__main__":
    main()
