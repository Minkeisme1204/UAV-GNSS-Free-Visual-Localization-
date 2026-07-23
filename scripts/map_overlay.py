#!/usr/bin/env python3
"""map_overlay.py — draw the PPK track on the MUN-FRL map JPEG.

!! THE ALIGNMENT IS NOT SOLVED !!
---------------------------------
The PPK track is WGS84 lat/lon. The map JPEG (and the map point cloud) live in a
**local metric frame** whose tie to WGS84 is not published with the dataset:
there is no datum, no anchor lat/lon, and no rotation. This script therefore
produces a **manual fit**, not a georeferenced product:

  1. lat/lon are converted to flat-earth ENU anchored at the first PPK fix
     (or at `--anchor-lat/--anchor-lon` with `--anchor manual`), using the same
     formula as `src/debug_viewer/gps_to_enu.h`;
  2. `--rot DEG` rotates that ENU track about the anchor (counter-clockwise);
  3. `--offset X Y` translates it, in metres, in the map's frame;
  4. the result is drawn with `MapGeo.xy_to_pixel`, i.e. the `map_server`
     bottom-left origin convention.

Nothing here estimates the transform. Every printed coordinate is only as good
as the `--rot`/`--offset` you supplied. Solving the tie properly means
registering the PPK track (or the VO trajectory) against the map point cloud.

Empirical note (bell412_dataset6): with `--anchor first-fix --rot 0
--offset 0 0` the track already lands on the NRC Ottawa site and its start/end
markers sit on the helipad, and its extent (x [-262, 591] m, y [-57, 713] m)
sits inside both the raster extent and the map cloud bounds. That strongly
suggests the local frame is ENU anchored at (or very near) the first PPK fix —
but it is an eyeball observation, not a published or fitted tie.

The map geo-parameters come from the file name
(`..._3_602_pxpm_..._0_277624_-502_-86.jpg` -> 0.277624 m/px, origin
(-502, -86) m) and can be overridden with `--resolution` / `--origin`.

Examples:
  ./map_overlay.py /media/.../dataset6_..._-502_-86.jpg --ppk data/.../ppk.csv
  ./map_overlay.py map.jpg --ppk ppk.csv --rot 12.5 --offset 120 -45
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np

from munfrl import DEFAULT_OUTPUT_ROOT, geomap, pos

OUTPUT_NAME = "map_overlay.png"

TRACK_COLOR_BGR = (0, 0, 255)      # red polyline
START_COLOR_BGR = (0, 255, 0)      # green start marker
END_COLOR_BGR = (255, 0, 0)        # blue end marker
TRACK_THICKNESS_PX = 2
MARKER_RADIUS_PX = 8
ANCHOR_MODES = ("first-fix", "manual")


def rotate_2d(east: np.ndarray, north: np.ndarray, deg: float):
    """Rotate an ENU track counter-clockwise about its own origin."""
    if deg == 0.0:
        return east, north
    rad = np.deg2rad(deg)
    c, s = np.cos(rad), np.sin(rad)
    return east * c - north * s, east * s + north * c


def main() -> int:
    p = argparse.ArgumentParser(
        description="Overlay the PPK track on the MUN-FRL map raster (MANUAL fit).",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("map_image", help="Path to the map JPEG/PNG.")
    p.add_argument("--ppk", required=True, help="ppk.csv from pos_to_csv.py, or a .pos.")
    p.add_argument("--out", default=str(DEFAULT_OUTPUT_ROOT), help="Output directory.")
    p.add_argument("--name", default=OUTPUT_NAME, help="Output PNG file name.")
    p.add_argument("--anchor", choices=ANCHOR_MODES, default="first-fix",
                   help="ENU anchor: the first PPK fix, or an explicit lat/lon.")
    p.add_argument("--anchor-lat", type=float, help="Anchor latitude (--anchor manual).")
    p.add_argument("--anchor-lon", type=float, help="Anchor longitude (--anchor manual).")
    p.add_argument("--offset", nargs=2, type=float, default=(0.0, 0.0),
                   metavar=("X", "Y"), help="Manual translation in map metres.")
    p.add_argument("--rot", type=float, default=0.0,
                   help="Manual rotation of the ENU track, in degrees CCW.")
    p.add_argument("--resolution", type=float,
                   help="Override the map resolution, in metres per pixel.")
    p.add_argument("--origin", nargs=2, type=float, metavar=("X", "Y"),
                   help="Override the map origin (bottom-left corner), in metres.")
    args = p.parse_args()

    map_path = Path(args.map_image)
    if not map_path.is_file():
        print(f"ERROR: '{map_path}' is not a file", file=sys.stderr)
        return 2

    import cv2

    image = cv2.imread(str(map_path), cv2.IMREAD_COLOR)
    if image is None:
        print(f"ERROR: cannot decode '{map_path}'", file=sys.stderr)
        return 2
    height, width = image.shape[:2]

    try:
        geo = geomap.MapGeo.from_filename(map_path, width=width, height=height)
    except ValueError as exc:
        if args.resolution is None or args.origin is None:
            print(f"ERROR: {exc}\n       supply --resolution and --origin instead.",
                  file=sys.stderr)
            return 2
        geo = geomap.MapGeo(resolution=args.resolution, origin_x=args.origin[0],
                            origin_y=args.origin[1], width=width, height=height,
                            source="cli")
    if args.resolution is not None:
        geo.resolution, geo.source = args.resolution, geo.source + "+cli-resolution"
    if args.origin is not None:
        geo.origin_x, geo.origin_y = args.origin
        geo.source += "+cli-origin"

    print(f"Map: {geo.describe()}")
    if not geo.resolution_is_consistent():
        print("WARNING: resolution does not match 1 / pixels_per_metre.",
              file=sys.stderr)

    records = pos.read_any(Path(args.ppk))
    if args.anchor == "manual":
        if args.anchor_lat is None or args.anchor_lon is None:
            print("ERROR: --anchor manual needs --anchor-lat and --anchor-lon",
                  file=sys.stderr)
            return 2
        lat0, lon0 = args.anchor_lat, args.anchor_lon
    else:
        lat0 = float(records["latitude_deg"][0])
        lon0 = float(records["longitude_deg"][0])

    east, north, _ = geomap.flat_earth_enu(records["latitude_deg"],
                                           records["longitude_deg"], 0.0,
                                           lat0, lon0, 0.0)
    east, north = rotate_2d(east, north, args.rot)
    x = east + args.offset[0]
    y = north + args.offset[1]

    px, py = geo.xy_to_pixel(x, y)
    pts = np.stack([px, py], axis=1).astype(np.int32).reshape(-1, 1, 2)
    inside = int(np.count_nonzero((px >= 0) & (px < width) & (py >= 0) & (py < height)))

    cv2.polylines(image, [pts], isClosed=False, color=TRACK_COLOR_BGR,
                  thickness=TRACK_THICKNESS_PX)
    cv2.circle(image, (int(px[0]), int(py[0])), MARKER_RADIUS_PX, START_COLOR_BGR, -1)
    cv2.circle(image, (int(px[-1]), int(py[-1])), MARKER_RADIUS_PX, END_COLOR_BGR, -1)

    out_path = Path(args.out) / args.name
    out_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_path), image)

    print(f"Anchor        : {args.anchor} lat {lat0:.9f} lon {lon0:.9f}")
    print(f"Manual fit    : rot {args.rot} deg CCW, offset "
          f"({args.offset[0]}, {args.offset[1]}) m")
    print(f"Track extent  : x [{x.min():.1f}, {x.max():.1f}] m, "
          f"y [{y.min():.1f}, {y.max():.1f}] m (map frame, AFTER the manual fit)")
    print(f"Points inside : {inside}/{records.size}")
    print(f"Written       : {out_path}")
    print("\nREMINDER: the WGS84 -> map-frame tie is NOT solved. This overlay is a "
          "MANUAL fit driven entirely by --rot/--offset; do not read it as a "
          "georeferenced result.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
