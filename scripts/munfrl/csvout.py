"""Writers for the telemetry CSV shapes the uavloc C++ code can ingest.

Source B — `uavloc::sensor::DroneTelemetryCsvReader` (**the primary shape**, the
one both existing datasets use). The reader has no fixed schema: the mission
config's `DroneTelemetry.columns` list maps each logical field onto a column
located by header **name**, so the names below only have to match that list.

    frame_id,timestamp_msec,roll_deg,pitch_deg,yaw_deg,
    gimbal_pan_deg,gimbal_tilt_deg,latitude_deg,longitude_deg,altitude_m,
    ground_speed_mps,climb_mps

`gimbal_pan_deg` / `gimbal_tilt_deg` are what makes this shape mandatory for the
fusion back-end: `TelemetryCsvReader` (source A) never populates them, so a
fixed nadir camera would reach `uavloc::fusion::rotation_enu_camera()` with
pan = tilt = 0 — the convention for a *forward*-looking camera. On a platform
with no physical gimbal the two columns are constants that encode the fixed
mounting instead (see `make_telemetry_csv.py --gimbal-pan/--gimbal-tilt`).

Source A — `uavloc::sensor::TelemetryCsvReader` (10 columns, header row
required, only `timestamp_msec` mandatory, column order flexible):

    timestamp_msec,frame_id,heading_deg,pitch_deg,roll_deg,
    latitude_deg,longitude_deg,altitude_m,speed_mps,climb_mps

Debug-viewer shape — `uavloc::debug_viewer::load_telemetry_csv` reads columns by
**fixed 0-based index** and needs at least 33 of them; it also skips the first
row unconditionally. Only these indices are read:

    0 imageId | 21 roll | 22 pitch | 23 yaw | 26 groundSpeed
    30 sensorLatitude | 31 sensorLongitude | 32 sensorAltitude

Every other column is emitted empty. The viewer additionally drops rows whose
latitude is exactly 0.0 (its "GPS not locked" filter).
"""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Iterable, Mapping, Sequence

# --- source B (primary) ----------------------------------------------------- #
DRONE_COLUMNS: tuple[str, ...] = (
    "frame_id",
    "timestamp_msec",
    "roll_deg",
    "pitch_deg",
    "yaw_deg",
    "gimbal_pan_deg",
    "gimbal_tilt_deg",
    "latitude_deg",
    "longitude_deg",
    "altitude_m",
    "ground_speed_mps",
    "climb_mps",
)

# --- source A -------------------------------------------------------------- #
UAVLOC_COLUMNS: tuple[str, ...] = (
    "timestamp_msec",
    "frame_id",
    "heading_deg",
    "pitch_deg",
    "roll_deg",
    "latitude_deg",
    "longitude_deg",
    "altitude_m",
    "speed_mps",
    "climb_mps",
)

# --- debug-viewer shape ---------------------------------------------------- #
DEBUG_VIEWER_MIN_COLUMNS = 33
DEBUG_VIEWER_COLUMN_INDEX: Mapping[str, int] = {
    "frame_id": 0,
    "roll_deg": 21,
    "pitch_deg": 22,
    "yaw_deg": 23,
    "speed_mps": 26,
    "latitude_deg": 30,
    "longitude_deg": 31,
    "altitude_m": 32,
}
# Header names mirroring the YenBai drone log so DroneTelemetryCsvReader can also
# locate the columns by name.
DEBUG_VIEWER_HEADER_NAME: Mapping[int, str] = {
    0: "imageId",
    21: "roll",
    22: "pitch",
    23: "yaw",
    26: "groundSpeed",
    30: "sensorLatitude",
    31: "sensorLongitude",
    32: "sensorAltitude",
}

# Fixed-point formatting so the C++ std::stod parsers never see scientific
# notation or a lat/lon truncated below centimetre resolution.
_FORMATS: Mapping[str, str] = {
    "timestamp_msec": "{:.3f}",
    "frame_id": "{:d}",
    "heading_deg": "{:.4f}",
    "yaw_deg": "{:.4f}",
    "pitch_deg": "{:.4f}",
    "roll_deg": "{:.4f}",
    "latitude_deg": "{:.9f}",
    "longitude_deg": "{:.9f}",
    "altitude_m": "{:.4f}",
    "speed_mps": "{:.4f}",
    "ground_speed_mps": "{:.4f}",
    "climb_mps": "{:.4f}",
    "gimbal_pan_deg": "{:.4f}",
    "gimbal_tilt_deg": "{:.4f}",
}


def _fmt(name: str, value) -> str:
    if value is None:
        return ""
    spec = _FORMATS.get(name, "{}")
    try:
        return spec.format(int(value) if spec.endswith("d}") else float(value))
    except (TypeError, ValueError):
        return str(value)


def write_drone_telemetry(path: str | Path, rows: Iterable[Mapping]) -> int:
    """Write the source-B drone-style telemetry CSV. Returns the row count.

    Missing keys are written as empty strings; `DroneTelemetryCsvReader` parses
    those with `std::stod` inside a try/catch and falls back to 0.0.
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    n = 0
    with open(path, "w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(DRONE_COLUMNS)
        for row in rows:
            writer.writerow([_fmt(c, row.get(c)) for c in DRONE_COLUMNS])
            n += 1
    return n


def write_uavloc_telemetry(path: str | Path, rows: Iterable[Mapping]) -> int:
    """Write the 10-column source-A telemetry CSV. Returns the row count.

    Missing keys are written as empty strings; `TelemetryCsvReader` treats those
    as 0 (only `timestamp_msec` is mandatory).
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    n = 0
    with open(path, "w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(UAVLOC_COLUMNS)
        for row in rows:
            writer.writerow([_fmt(c, row.get(c)) for c in UAVLOC_COLUMNS])
            n += 1
    return n


def write_debug_viewer_telemetry(path: str | Path, rows: Iterable[Mapping],
                                 n_columns: int = DEBUG_VIEWER_MIN_COLUMNS) -> int:
    """Write the fixed-index, >=33-column CSV that `load_telemetry_csv` expects.

    Returns the row count. `n_columns` may be raised to pad the row out further
    (the YenBai logs have 57); it may never go below `DEBUG_VIEWER_MIN_COLUMNS`.
    """
    if n_columns < DEBUG_VIEWER_MIN_COLUMNS:
        raise ValueError(
            f"n_columns must be >= {DEBUG_VIEWER_MIN_COLUMNS}, got {n_columns}"
        )
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    n = 0
    with open(path, "w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        header = [DEBUG_VIEWER_HEADER_NAME.get(i, f"col{i}") for i in range(n_columns)]
        writer.writerow(header)
        for row in rows:
            out = [""] * n_columns
            for name, idx in DEBUG_VIEWER_COLUMN_INDEX.items():
                if name in row and row[name] is not None:
                    out[idx] = _fmt(name, row[name])
            writer.writerow(out)
            n += 1
    return n


def write_rows(path: str | Path, columns: Sequence[str],
               rows: Iterable[Sequence]) -> int:
    """Generic CSV writer used by the extraction scripts. Returns the row count."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    n = 0
    with open(path, "w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(list(columns))
        for row in rows:
            writer.writerow(list(row))
            n += 1
    return n
