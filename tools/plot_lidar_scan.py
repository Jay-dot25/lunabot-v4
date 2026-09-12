#!/usr/bin/env python3
"""
LunaBot V4 - Phase A - plot a single LiDAR scan (sensor_msgs/LaserScan)
saved as YAML by `ros2 topic echo /lunabot/lidar/scan --once`.

Usage:
    python3 tools/plot_lidar_scan.py lidar_scan_sample.yaml lidar_scan_preview.png

Writes a top-down polar view of the scan (the crater field around the rover).
"""
import math
import os
import sys


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        return 2
    yaml_path, out_path = sys.argv[1], sys.argv[2]

    import yaml
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    with open(yaml_path) as fh:
        scan = yaml.safe_load(fh)

    ranges = [float(r) for r in scan.get("ranges", [])]
    angles = [float(a) for a in scan.get("angles", [])]
    if not angles:
        n = len(ranges)
        a0 = float(scan.get("angle_min", -math.pi))
        a1 = float(scan.get("angle_max", math.pi))
        angles = [a0 + (a1 - a0) * i / max(1, n - 1) for i in range(n)]

    a_min = float(scan.get("angle_min", -math.pi))
    a_max = float(scan.get("angle_max", math.pi))
    r_max = min(60.0, float(scan.get("range_max", 60.0)))
    r_min = float(scan.get("range_min", 0.1))
    frame = scan.get("header", {}).get("frame_id", "lidar")
    stamp = scan.get("header", {}).get("stamp", {})
    t = float(stamp.get("sec", 0)) + float(stamp.get("nanosec", 0)) * 1e-9

    pts = []
    for r, a in zip(ranges, angles):
        if r_min < r < r_max and r > 0:
            pts.append((math.cos(a) * r, math.sin(a) * r))

    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]

    lim = min(60.0, max([abs(v) for v in xs + ys] + [20.0]))
    fig, ax = plt.subplots(figsize=(9, 9), dpi=140)
    ax.scatter(xs, ys, s=1.2, c="0.25", alpha=0.8)
    for ring in (10, 20, 30, 40, 50):
        if ring <= lim:
            ax.add_patch(plt.Circle((0, 0), ring, fill=False, ec="0.6", lw=0.6, ls="--"))
    ax.plot([0], [0], marker="^", ms=9, color="red")
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m] (rover frame)")
    ax.set_ylabel("y [m] (rover frame)")
    ax.set_title(f"LiDAR scan - {frame} - t={t:.1f} s - {len(pts)}/{len(ranges)} valid ranges")
    fig.tight_layout()
    fig.savefig(out_path)
    print(f"wrote {out_path} ({len(pts)} points)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
