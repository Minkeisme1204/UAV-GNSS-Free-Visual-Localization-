#!/usr/bin/env python3
"""agl_from_lidar.py — first-order AGL per Velodyne scan from `/velodyne_points`.

For every scan the points falling inside a nadir cone (half-angle `--cone-deg`,
default 20 deg, measured from the sensor's -Z axis) are collected and a robust
percentile of their downward distance `-z` is reported as the height above
ground.

LIMITATIONS — read before trusting a number
-------------------------------------------
* The result is **sensor-frame** height: it is the distance from the LiDAR to
  whatever the cone hits, along the sensor's own Z axis.
* The **IMU-to-LiDAR extrinsic is ignored** — no lever arm, no boresight.
* **Airframe attitude is ignored** — during a banked turn the cone points off
  nadir and the estimate reads long.
* Canopy, buildings and vehicles are ground for this estimate; only the
  percentile choice pushes back on that.

SENSOR GEOMETRY — MUN-FRL bell412 (measured)
--------------------------------------------
The Velodyne is mounted **horizontally** (spin axis up) and is a VLP-16-class
unit with a +/-15 deg vertical field of view, so the steepest return sits
**75 deg from nadir**. A literal 20 deg nadir cone therefore contains ZERO
points on this dataset — use `--cone-deg 80` (the script says so at run time).
With a ~100 m maximum range and a 15 deg depression, ground can only be seen
below roughly 100 * sin(15 deg) ~ 26 m AGL; above that the estimate saturates
on whatever nearby structure is still in view and must not be trusted.

Even so this is a better AGL proxy than the ellipsoidal PPK height, which is not
a height above anything physical.

Examples:
  ./agl_from_lidar.py /media/.../bell412_dataset6 --max-scans 20 --cone-deg 80
  ./agl_from_lidar.py /media/.../bell412_dataset6 --cone-deg 80 --percentile 20
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np

from munfrl import DEFAULT_OUTPUT_ROOT, bagio, timeutil

TOPIC_LIDAR = "/velodyne_points"
OUTPUT_NAME = "agl.csv"
OUTPUT_COLUMNS = ("timestamp_msec", "bag_msec", "agl_m", "num_points")

DEFAULT_CONE_DEG = 20.0
# Median of the cone hits: robust to a few canopy/vehicle returns without
# chasing the single lowest (possibly spurious) point.
DEFAULT_PERCENTILE = 50.0
DEFAULT_MIN_POINTS = 20
PROGRESS_EVERY = 200


def agl_from_scan(xyzi: np.ndarray, cone_deg: float, percentile: float,
                  min_points: int) -> tuple[float | None, int, float]:
    """Robust downward distance for one scan.

    Returns ``(agl_m | None, n_points_in_cone, min_angle_from_nadir_deg)``. The
    last value is the steepest downward return the sensor actually produced and
    is what the caller uses to tell the user a usable `--cone-deg`.
    """
    if xyzi.size == 0:
        return None, 0, float("nan")
    x, y, z = xyzi[:, 0], xyzi[:, 1], xyzi[:, 2]
    down = -z
    rng = np.sqrt(x * x + y * y + z * z)
    valid = (down > 0.0) & (rng > 0.0)
    if not np.any(valid):
        return None, 0, float("nan")
    # cos(angle from nadir) = down / range
    cos_ratio = down[valid] / rng[valid]
    min_angle = math.degrees(math.acos(min(1.0, float(cos_ratio.max()))))
    cos_limit = math.cos(math.radians(cone_deg))
    inside = valid & (down >= cos_limit * rng)
    n_used = int(np.count_nonzero(inside))
    if n_used < min_points:
        return None, n_used, min_angle
    return float(np.percentile(down[inside], percentile)), n_used, min_angle


def main() -> int:
    p = argparse.ArgumentParser(
        description="Estimate per-scan AGL from Velodyne nadir returns (first-order).",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("path", help="A .bag file, or a dataset/split-bag directory.")
    p.add_argument("--out", default=str(DEFAULT_OUTPUT_ROOT), help="Output directory.")
    p.add_argument("--name", default=OUTPUT_NAME, help="Output CSV file name.")
    p.add_argument("--topic", default=TOPIC_LIDAR, help="PointCloud2 topic.")
    p.add_argument("--cone-deg", type=float, default=DEFAULT_CONE_DEG,
                   help="Half-angle of the nadir cone, in degrees.")
    p.add_argument("--percentile", type=float, default=DEFAULT_PERCENTILE,
                   help="Percentile of -z taken inside the cone (50 = median).")
    p.add_argument("--min-points", type=int, default=DEFAULT_MIN_POINTS,
                   help="Scans with fewer cone hits than this are written as blank.")
    p.add_argument("--max-scans", type=int, default=0,
                   help="Stop after N scans (0 = all). Useful for smoke tests.")
    args = p.parse_args()

    if not 0.0 < args.cone_deg < 90.0:
        print("ERROR: --cone-deg must be in (0, 90)", file=sys.stderr)
        return 2
    if not 0.0 <= args.percentile <= 100.0:
        print("ERROR: --percentile must be in [0, 100]", file=sys.stderr)
        return 2

    out_path = Path(args.out) / args.name
    out_path.parent.mkdir(parents=True, exist_ok=True)

    print("NOTE: sensor-frame AGL. The IMU-to-LiDAR extrinsic and the airframe "
          "attitude are IGNORED — treat these values as a first-order estimate.")

    import csv

    n = 0
    n_valid = 0
    values: list[float] = []
    min_angle_seen = float("inf")
    warned_geometry = False
    with bagio.open_bags(args.path) as reader, \
            open(out_path, "w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(OUTPUT_COLUMNS)
        for t_ns, msg in bagio.iter_topic(reader, args.topic):
            if args.max_scans and n >= args.max_scans:
                break
            xyzi = bagio.pointcloud2_xyzi(msg)
            agl, n_used, min_angle = agl_from_scan(xyzi, args.cone_deg,
                                                   args.percentile, args.min_points)
            if np.isfinite(min_angle):
                min_angle_seen = min(min_angle_seen, min_angle)
                if not warned_geometry and min_angle > args.cone_deg:
                    warned_geometry = True
                    print(f"WARNING: the steepest downward return in this scan is "
                          f"{min_angle:.1f} deg from nadir, but --cone-deg is "
                          f"{args.cone_deg:.1f} — the cone is empty. This sensor is "
                          f"mounted horizontally; re-run with "
                          f"--cone-deg {math.ceil(min_angle) + 1}.", file=sys.stderr)
            t_hdr = timeutil.ros_stamp_to_epoch(msg.header.stamp)
            t_bag = timeutil.ns_to_epoch(t_ns)
            writer.writerow([
                timeutil.format_msec(t_hdr if t_hdr > 0 else t_bag),
                timeutil.format_msec(t_bag),
                "" if agl is None else f"{agl:.4f}",
                n_used,
            ])
            if agl is not None:
                values.append(agl)
                n_valid += 1
            n += 1
            if n % PROGRESS_EVERY == 0:
                print(f"  {n} scans", end="\r", flush=True)

    print(f"\nScans processed : {n}  (valid AGL: {n_valid})")
    if values:
        arr = np.asarray(values)
        print(f"AGL range       : [{arr.min():.3f}, {arr.max():.3f}] m, "
              f"median {np.median(arr):.3f} m")
    if np.isfinite(min_angle_seen):
        print(f"Steepest return : {min_angle_seen:.1f} deg from nadir "
              "(sensor vertical FOV limit)")
    print(f"Cone half-angle : {args.cone_deg} deg, percentile {args.percentile}")
    print(f"Written         : {out_path}")
    if n and not n_valid:
        print("ERROR: no scan produced a valid AGL — see the geometry warning above.",
              file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
