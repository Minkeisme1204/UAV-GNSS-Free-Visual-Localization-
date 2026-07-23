#!/usr/bin/env python3
"""make_telemetry_csv.py — fuse frame stamps + PPK (+ AGL) into uavloc telemetry.

Inputs
------
  --frames  frames_nadir.csv  (from `bag_to_video.py`; needs frame_id + timestamp_msec)
  --ppk     ppk.csv or the raw .pos  (from `pos_to_csv.py` / the dataset)
  --agl     agl.csv  (from `agl_from_lidar.py`; only for --altitude-source agl)

For every video frame the PPK solution is interpolated at the frame's **UTC**
capture time — linearly on position/velocity, shortest-arc on roll/pitch/yaw.
Frames outside the PPK span are dropped (never extrapolated) and counted.

Outputs
-------
  bell412_dataset6.csv  (default, the ONLY file written unless asked otherwise)
      The drone-log shape read by `uavloc::sensor::DroneTelemetryCsvReader`
      ("source B") — the same path both existing datasets take. Columns are
      located by header NAME through the mission config's
      `DroneTelemetry.columns` list, so the names here are a contract with that
      list, not with the parser.

Opt-in extras (off by default, kept for `debug_viewer` work):
  --emit-uavloc-10col   telemetry_uavloc.csv        10-column `TelemetryCsvReader` shape
  --emit-viewer-33col   telemetry_yenbai_shape.csv  >=33 columns at the fixed indices
                                                    the `debug_viewer` CSV loader reads

Gimbal columns encode the FIXED camera mounting
-----------------------------------------------
The Bell 412 has **no gimbal** — the nadir camera is bolted to the airframe.
`gimbal_pan_deg` / `gimbal_tilt_deg` are therefore written as constants
(`--gimbal-pan` / `--gimbal-tilt`) that stand in for that fixed mount, because
the fusion back-end has no other channel for it: `uavloc::fusion::factors.h`
builds

    R_enu_cam = C_enu_ned * R_ned_body(heading,pitch,roll)
                * Rz(pan) * Ry(-tilt) * R_CAM_ALIGNMENT

and `R_CAM_ALIGNMENT` is hard-coded for a camera looking FORWARD along body x.
Left at pan = tilt = 0 the whole pipeline would believe this nadir camera is
forward-looking — a 90 deg elevation error the mount-azimuth state cannot absorb.

The defaults (pan = 180, tilt = 90) are derived from the official extrinsic
`body_T_camDown` (.docs/munfrl_calibration.md §8a; body frame is FRD):
x_cam -> -Y_body, y_cam -> +X_body, z_cam -> +Z_body(down). Verified
numerically: `Rz(180)*Ry(-90)*R_CAM_ALIGNMENT` matches that rotation block to
max |element diff| = 0.0153, i.e. the ~0.9 deg by which the real mount is off a
perfect right angle. Note `TelemetryCsvReader` (source A) never populates these
two fields at all, which is precisely why source B is the primary output.

Altitude
--------
`--altitude-source` picks what lands in `altitude_m`, which VO consumes as AGL
to seed its metric scale:
  takeoff-ref  (default) `height(t) - h_ground`, with `h_ground` the median
               `.pos` height while the aircraft is parked at the start of the
               log — a datum-free AGL, exact where the terrain is flat
  ellipsoidal  the raw `.pos` height passthrough — a height above nothing physical
  msl          `height - geoid_separation` (H = h - N)  <- SUSPECT, see below
  agl          the LiDAR estimate from `agl_from_lidar.py`

`msl` is suspect on `bell412_dataset6`: the flight site is ~300 m from the CYOW
airport reference point whose field elevation is ~114 m MSL, and the parked
`.pos` height is 116.7 m — already ~3 m above the field. Subtracting N = -34.2 m
would put a parked helicopter 37 m above the runway, so the `.pos` height most
likely already *is* orthometric and `msl` double-counts the geoid separation.
The datum cannot be settled from the data alone; only `takeoff-ref` sidesteps it
entirely, which is why it is the default. Every other mode prints a warning.

Examples:
  ./make_telemetry_csv.py --frames data/.../frames_nadir.csv --ppk data/.../ppk.csv
  ./make_telemetry_csv.py --frames f.csv --ppk ppk.csv --ground-height 116.70
  ./make_telemetry_csv.py --frames f.csv --ppk ppk.csv --altitude-source agl --agl agl.csv
  ./make_telemetry_csv.py --frames f.csv --ppk ppk.csv --emit-viewer-33col
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np

from munfrl import DEFAULT_OUTPUT_ROOT, csvout, pos

DRONE_OUTPUT_NAME = "bell412_dataset6.csv"
UAVLOC_OUTPUT_NAME = "telemetry_uavloc.csv"
DEBUG_VIEWER_OUTPUT_NAME = "telemetry_yenbai_shape.csv"

# Fixed-mount encoding for the gimbal columns — see the module docstring.
# Derived from `body_T_camDown` (.docs/munfrl_calibration.md §8a), not measured
# gimbal angles: this airframe has no gimbal.
DEFAULT_GIMBAL_PAN_DEG = 180.0
DEFAULT_GIMBAL_TILT_DEG = 90.0

FRAMES_ID_COLUMN = "frame_id"
FRAMES_TIME_COLUMN = "timestamp_msec"

AGL_TIME_COLUMN = "timestamp_msec"
AGL_VALUE_COLUMN = "agl_m"

# Geoid undulation N at the MUN-FRL Ottawa site, straight out of the first
# `$GNGGA` sentence in bell412_dataset6 (field 11). Orthometric height is
# H = h - N with h the WGS84 ellipsoidal height from the .pos.
DEFAULT_GEOID_SEPARATION_M = -34.2

# A LiDAR AGL sample further than this from a frame stamp is not used.
DEFAULT_AGL_MAX_DT_MS = 200.0

ALTITUDE_SOURCES = ("takeoff-ref", "ellipsoidal", "msl", "agl")
DEFAULT_ALTITUDE_SOURCE = "takeoff-ref"

MSL_WARNING = (
    "WARNING: --altitude-source msl very likely DOUBLE-COUNTS the geoid "
    "separation on this dataset. The site is ~300 m from the CYOW airport "
    "reference point (field elevation ~114 m MSL) and the parked .pos height is "
    "116.7 m, i.e. already ~3 m above the field; H = h - N would place a parked "
    "aircraft ~37 m above the runway. The .pos header claims an ellipsoidal "
    "height but the column behaves like an orthometric one. This is an open "
    "question that the data cannot settle — msl is NOT a safe default."
)

NOT_AGL_WARNING = (
    "WARNING: this altitude is NOT height above ground. VO consumes altitude_m "
    "as AGL to seed its metric scale, so the whole trajectory will come out "
    "mis-scaled. Use --altitude-source takeoff-ref instead."
)

TAKEOFF_REF_CAVEAT = (
    "NOTE: takeoff-ref assumes flat terrain relative to the takeoff point. Any "
    "real terrain relief along the track shows up one-for-one as an AGL error "
    "(the site is an airport apron, so this is a small effect here)."
)

LIDAR_AGL_CAVEAT = (
    "WARNING: the Velodyne on this platform is mounted horizontally (steepest "
    "return 75 deg from nadir), so the ground is only visible below ~26 m AGL. "
    "The LiDAR AGL is usable for the takeoff/landing segments only, not for the "
    "cruise phase."
)


def read_frames_csv(path: Path) -> list[tuple[int, float]]:
    """Read `frames_*.csv` -> [(frame_id, timestamp_msec)] sorted by frame_id."""
    rows: list[tuple[int, float]] = []
    with open(path, "r", encoding="utf-8", newline="") as fh:
        reader = csv.DictReader(fh)
        if reader.fieldnames is None:
            raise ValueError(f"'{path}' has no header row")
        for col in (FRAMES_ID_COLUMN, FRAMES_TIME_COLUMN):
            if col not in reader.fieldnames:
                raise ValueError(f"'{path}' is missing the '{col}' column")
        for row in reader:
            try:
                rows.append((int(row[FRAMES_ID_COLUMN]),
                             float(row[FRAMES_TIME_COLUMN])))
            except (TypeError, ValueError):
                continue
    if not rows:
        raise ValueError(f"no usable rows in '{path}'")
    rows.sort(key=lambda r: r[0])
    return rows


def read_agl_csv(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Read `agl.csv` -> (timestamps_msec, agl_m) with blank rows removed."""
    times, values = [], []
    with open(path, "r", encoding="utf-8", newline="") as fh:
        reader = csv.DictReader(fh)
        if reader.fieldnames is None or AGL_VALUE_COLUMN not in reader.fieldnames:
            raise ValueError(f"'{path}' is missing the '{AGL_VALUE_COLUMN}' column")
        for row in reader:
            if not row.get(AGL_VALUE_COLUMN):
                continue
            try:
                times.append(float(row[AGL_TIME_COLUMN]))
                values.append(float(row[AGL_VALUE_COLUMN]))
            except (TypeError, ValueError):
                continue
    if not times:
        raise ValueError(f"no valid AGL rows in '{path}'")
    order = np.argsort(times)
    return np.asarray(times)[order], np.asarray(values)[order]


def nearest_agl(times_ms: np.ndarray, values: np.ndarray, t_ms: float,
                max_dt_ms: float) -> float | None:
    """Nearest AGL sample to `t_ms`, or None when the gap exceeds `max_dt_ms`."""
    idx = int(np.searchsorted(times_ms, t_ms))
    candidates = [i for i in (idx - 1, idx) if 0 <= i < times_ms.size]
    if not candidates:
        return None
    best = min(candidates, key=lambda i: abs(times_ms[i] - t_ms))
    if abs(times_ms[best] - t_ms) > max_dt_ms:
        return None
    return float(values[best])


def main() -> int:
    p = argparse.ArgumentParser(
        description="Build the uavloc drone-log telemetry CSV (DroneTelemetryCsvReader "
                    "shape) from frame stamps + PPK (+ LiDAR AGL).",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--frames", required=True, help="frames_*.csv from bag_to_video.py.")
    p.add_argument("--ppk", required=True, help="ppk.csv from pos_to_csv.py, or a .pos.")
    p.add_argument("--agl", help="agl.csv from agl_from_lidar.py.")
    p.add_argument("--out", default=str(DEFAULT_OUTPUT_ROOT), help="Output directory.")
    p.add_argument("--altitude-source", choices=ALTITUDE_SOURCES,
                   default=DEFAULT_ALTITUDE_SOURCE, help="What goes into altitude_m.")
    p.add_argument("--geoid-separation", type=float, default=DEFAULT_GEOID_SEPARATION_M,
                   help="Geoid undulation N (m) used by --altitude-source msl.")
    p.add_argument("--ground-height", type=float,
                   help="Ground .pos height (m) for --altitude-source takeoff-ref. "
                        "Overrides the stationary-window estimate entirely.")
    p.add_argument("--takeoff-window-s", type=float, default=pos.DEFAULT_TAKEOFF_WINDOW_S,
                   help="Length of the start-of-log window searched for stationary rows.")
    p.add_argument("--stationary-speed-mps", type=float,
                   default=pos.DEFAULT_STATIONARY_SPEED_MPS,
                   help="Speed below which a .pos row counts as parked on the ground.")
    p.add_argument("--agl-max-dt-ms", type=float, default=DEFAULT_AGL_MAX_DT_MS,
                   help="Maximum frame-to-AGL time gap accepted, in milliseconds.")
    p.add_argument("--gimbal-pan", type=float, default=DEFAULT_GIMBAL_PAN_DEG,
                   help="Constant written to gimbal_pan_deg on every row. This "
                        "airframe has NO gimbal: the pair encodes the fixed camera "
                        "mounting from body_T_camDown so the fusion attitude chain "
                        "does not treat the nadir camera as forward-looking.")
    p.add_argument("--gimbal-tilt", type=float, default=DEFAULT_GIMBAL_TILT_DEG,
                   help="Constant written to gimbal_tilt_deg on every row "
                        "(from horizontal; 90 = nadir). See --gimbal-pan.")
    p.add_argument("--out-name", default=DRONE_OUTPUT_NAME,
                   help="File name of the drone-log (DroneTelemetryCsvReader) output.")
    p.add_argument("--emit-uavloc-10col", action="store_true",
                   help="Also write the 10-column TelemetryCsvReader CSV. It cannot "
                        "carry the gimbal columns, so it is not the fusion input.")
    p.add_argument("--emit-viewer-33col", action="store_true",
                   help="Also write the fixed-index >=33-column debug_viewer CSV.")
    p.add_argument("--uavloc-name", default=UAVLOC_OUTPUT_NAME,
                   help="File name of the 10-column output (--emit-uavloc-10col).")
    p.add_argument("--viewer-name", default=DEBUG_VIEWER_OUTPUT_NAME,
                   help="File name of the debug_viewer-shaped output (--emit-viewer-33col).")
    args = p.parse_args()

    frames_path, ppk_path = Path(args.frames), Path(args.ppk)
    for path in (frames_path, ppk_path):
        if not path.is_file():
            print(f"ERROR: '{path}' is not a file", file=sys.stderr)
            return 2

    frames = read_frames_csv(frames_path)
    records = pos.read_any(ppk_path)
    t_ppk0, t_ppk1 = pos.span(records)

    agl_times = agl_values = None
    if args.altitude_source == "agl":
        if not args.agl:
            print("ERROR: --altitude-source agl requires --agl agl.csv", file=sys.stderr)
            return 2
        agl_times, agl_values = read_agl_csv(Path(args.agl))

    ground_height = None
    n_stationary = 0
    if args.ground_height is not None and args.altitude_source != "takeoff-ref":
        print("WARNING: --ground-height only applies to --altitude-source "
              "takeoff-ref; ignoring it.", file=sys.stderr)
    if args.altitude_source == "takeoff-ref":
        if args.ground_height is not None:
            ground_height = float(args.ground_height)
        else:
            try:
                ground_height, n_stationary = pos.estimate_ground_height(
                    records, args.takeoff_window_s, args.stationary_speed_mps)
            except ValueError as exc:
                print(f"ERROR: cannot establish a ground reference: {exc}",
                      file=sys.stderr)
                return 2

    rows: list[dict] = []
    n_outside = 0
    n_no_agl = 0
    for frame_id, t_ms in frames:
        sample = pos.interpolate(records, t_ms / 1000.0)
        if sample is None:
            n_outside += 1
            continue
        if args.altitude_source == "takeoff-ref":
            altitude = sample["height_m"] - ground_height
        elif args.altitude_source == "ellipsoidal":
            altitude = sample["height_m"]
        elif args.altitude_source == "msl":
            altitude = sample["height_m"] - args.geoid_separation
        else:
            altitude = nearest_agl(agl_times, agl_values, t_ms, args.agl_max_dt_ms)
            if altitude is None:
                n_no_agl += 1
                continue
        rows.append({
            "timestamp_msec": t_ms,
            "frame_id": frame_id,
            "heading_deg": sample["yaw_deg"],
            "yaw_deg": sample["yaw_deg"],
            "pitch_deg": sample["pitch_deg"],
            "roll_deg": sample["roll_deg"],
            "gimbal_pan_deg": args.gimbal_pan,
            "gimbal_tilt_deg": args.gimbal_tilt,
            "latitude_deg": sample["latitude_deg"],
            "longitude_deg": sample["longitude_deg"],
            "altitude_m": altitude,
            "speed_mps": sample["speed_mps"],
            "ground_speed_mps": sample["speed_mps"],
            "climb_mps": sample["climb_mps"],
        })

    out_dir = Path(args.out)
    drone_path = out_dir / args.out_name
    n_drone = csvout.write_drone_telemetry(drone_path, rows)

    uavloc_path = viewer_path = None
    n_uavloc = n_viewer = 0
    if args.emit_uavloc_10col:
        uavloc_path = out_dir / args.uavloc_name
        n_uavloc = csvout.write_uavloc_telemetry(uavloc_path, rows)
    if args.emit_viewer_33col:
        viewer_path = out_dir / args.viewer_name
        n_viewer = csvout.write_debug_viewer_telemetry(viewer_path, rows)

    print(f"Frames in      : {len(frames)}  "
          f"({frames[0][1]:.3f} .. {frames[-1][1]:.3f} ms)")
    print(f"PPK span       : {t_ppk0 * 1000.0:.3f} .. {t_ppk1 * 1000.0:.3f} ms UTC "
          f"({records.size} rows)")
    print(f"Rows written   : {n_drone}")
    if n_outside:
        print(f"WARNING: {n_outside} frame(s) fell outside the PPK span and were "
              "dropped (no extrapolation).", file=sys.stderr)
    if n_no_agl:
        print(f"WARNING: {n_no_agl} frame(s) had no AGL sample within "
              f"{args.agl_max_dt_ms} ms and were dropped.", file=sys.stderr)
    print(f"Altitude source: {args.altitude_source}"
          + (f" (geoid separation {args.geoid_separation} m)"
             if args.altitude_source == "msl" else ""))
    if args.altitude_source == "takeoff-ref":
        if args.ground_height is not None:
            print(f"Ground height  : {ground_height:.3f} m  (.pos height, "
                  "manual --ground-height, no detection run)")
        else:
            print(f"Stationary rows: {n_stationary}  (first {args.takeoff_window_s:g} s, "
                  f"speed < {args.stationary_speed_mps:g} m/s)")
            print(f"Ground height  : {ground_height:.3f} m  (.pos height, median "
                  "over those rows)")
    if rows:
        alts = [r["altitude_m"] for r in rows]
        label = "AGL range      " if args.altitude_source in ("takeoff-ref", "agl") \
            else "Altitude range "
        print(f"{label}: [{min(alts):.3f}, {max(alts):.3f}] m")
    if args.altitude_source == "takeoff-ref":
        print(TAKEOFF_REF_CAVEAT, file=sys.stderr)
    elif args.altitude_source == "agl":
        print(LIDAR_AGL_CAVEAT, file=sys.stderr)
    else:
        if args.altitude_source == "msl":
            print(MSL_WARNING, file=sys.stderr)
        print(NOT_AGL_WARNING, file=sys.stderr)
    print(f"Gimbal columns : pan {args.gimbal_pan:g} deg, tilt {args.gimbal_tilt:g} deg "
          "(constant; fixed mount from body_T_camDown, this airframe has no gimbal)")
    print(f"Written        : {drone_path}  ({n_drone} rows, "
          f"{len(csvout.DRONE_COLUMNS)} columns, DroneTelemetryCsvReader shape)")
    if uavloc_path is not None:
        print(f"Written        : {uavloc_path}  ({n_uavloc} rows, "
              "10 columns, no gimbal columns)")
    if viewer_path is not None:
        print(f"Written        : {viewer_path}  ({n_viewer} rows, "
              f"{csvout.DEBUG_VIEWER_MIN_COLUMNS} columns)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
