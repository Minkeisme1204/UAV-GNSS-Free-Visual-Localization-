#!/usr/bin/env python3
"""kml_to_csv.py — MUN-FRL `ppk_data/*.kml` track -> CSV, with a `.pos` cross-check.

The KML holds one `<LineString>` whose `<coordinates>` blob lists every fix as a
`lon,lat,alt` triple. It carries **no timestamps** and the shipped files have
`alt = 0`, so it is only useful as a geometric sanity check on the `.pos` track.

With `--pos` the script pairs the two tracks row by row (they have the same
length and ordering for the MUN-FRL releases) and reports the maximum horizontal
deviation in metres, using the same flat-earth scaling as
`src/debug_viewer/gps_to_enu.h`.

Examples:
  ./kml_to_csv.py /media/.../ppk_data/Bell412_dataset6.kml
  ./kml_to_csv.py /media/.../ppk_data/Bell412_dataset6.kml \
      --pos /media/.../ppk_data/bell412_dataset6_frl.pos
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np

from munfrl import DEFAULT_OUTPUT_ROOT, csvout, geomap, kmlio, pos

OUTPUT_NAME = "kml.csv"
OUTPUT_COLUMNS = ("point_id", "latitude_deg", "longitude_deg", "altitude_m")

# Above this the two tracks are not the same flight path; below it they are the
# same solution rendered twice (rounding in the KML text is ~1e-7 deg ~ 1 cm).
CROSSCHECK_WARN_M = 1.0


def crosscheck(track: np.ndarray, pos_file: Path) -> int:
    """Compare the KML vertices against the `.pos` fixes. Returns an exit code."""
    records = pos.read_pos(pos_file)
    n = min(track.shape[0], records.size)
    print(f"\nCross-check vs {pos_file.name}")
    print(f"  kml points {track.shape[0]}   pos rows {records.size}")
    if track.shape[0] != records.size:
        print("  NOTE: differing lengths — comparing the first "
              f"{n} rows pairwise only.")

    lat0 = float(records["latitude_deg"][0])
    lon0 = float(records["longitude_deg"][0])
    e_kml, n_kml, _ = geomap.flat_earth_enu(track[:n, 1], track[:n, 0], 0.0,
                                            lat0, lon0, 0.0)
    e_pos, n_pos, _ = geomap.flat_earth_enu(records["latitude_deg"][:n],
                                            records["longitude_deg"][:n], 0.0,
                                            lat0, lon0, 0.0)
    dev = np.hypot(e_kml - e_pos, n_kml - n_pos)
    print(f"  horizontal deviation: max {dev.max():.4f} m, "
          f"mean {dev.mean():.4f} m, rms {np.sqrt((dev ** 2).mean()):.4f} m")
    if dev.max() > CROSSCHECK_WARN_M:
        print(f"  WARNING: max deviation exceeds {CROSSCHECK_WARN_M} m — the KML "
              "and the .pos are probably not the same solution.", file=sys.stderr)
        return 1
    print("  OK: the KML is the same track as the .pos solution.")
    return 0


def main() -> int:
    p = argparse.ArgumentParser(
        description="Convert a MUN-FRL KML track to CSV and cross-check it against .pos.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("kml_file", help="Path to the .kml track.")
    p.add_argument("--out", default=str(DEFAULT_OUTPUT_ROOT), help="Output directory.")
    p.add_argument("--name", default=OUTPUT_NAME, help="Output CSV file name.")
    p.add_argument("--pos", help="Optional .pos file to cross-check against.")
    args = p.parse_args()

    src = Path(args.kml_file)
    if not src.is_file():
        print(f"ERROR: '{src}' is not a file", file=sys.stderr)
        return 2

    track = kmlio.read_kml_track(src)
    out_path = Path(args.out) / args.name
    rows = ([i, f"{lat:.9f}", f"{lon:.9f}", f"{alt:.4f}"]
            for i, (lon, lat, alt) in enumerate(track))
    n = csvout.write_rows(out_path, OUTPUT_COLUMNS, rows)

    print(f"Points   : {n}")
    print(f"Latitude : [{track[:, 1].min():.9f}, {track[:, 1].max():.9f}]")
    print(f"Longitude: [{track[:, 0].min():.9f}, {track[:, 0].max():.9f}]")
    print(f"Altitude : [{track[:, 2].min():.3f}, {track[:, 2].max():.3f}] m "
          "(the MUN-FRL KMLs store 0 here)")
    print(f"Written  : {out_path}")

    if args.pos:
        return crosscheck(track, Path(args.pos))
    return 0


if __name__ == "__main__":
    sys.exit(main())
