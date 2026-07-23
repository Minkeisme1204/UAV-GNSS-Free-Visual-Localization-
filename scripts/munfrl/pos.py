"""RTKLIB `.pos` PPK solution reader for the MUN-FRL `ppk_data/` directory.

File shape (verified on `bell412_dataset6_frl.pos`): two `%` comment lines, then
whitespace/TAB separated rows at 5 Hz with 24 tokens each:

    date(YYYY/MM/DD) time(HH:MM:SS.sss) latitude(deg) longitude(deg) height(m)
    Q ns sdn sde sdu sdne sdeu sdun age ratio roll pitch yaw(deg)
    P_deg_s Q_deg_s R_deg_s Ve_m_s Vn_m_s Vu_m_s

Per the file's own header comment:
  * lat/lon/height are WGS84 with an **ellipsoidal** height (NOT MSL, NOT AGL) —
    but see the datum caveat below; the height column is passed through raw and
    this reader never applies a geoid model,
  * roll/pitch/yaw are in the **NED** frame (yaw 0 = North, clockwise),
  * timestamps are **GPST**, converted here to UTC via `timeutil`,
  * Q is the solution quality: 0 no position, 1 fix, 16 single precision,
    68 PPP-converging (TerraStar), 69 PPP-converged (TerraStar).

Datum caveat (bell412_dataset6): the parked height is 116.69 m while the CYOW
field elevation ~300 m away is ~114 m MSL, so the column behaves like an
*orthometric* height even though the header claims ellipsoidal. It cannot be
settled from the data, so nothing here converts it — `estimate_ground_height`
gives a datum-free reference (heights differenced against the takeoff point) and
is the only altitude the tooling recommends feeding to VO.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Optional

import numpy as np

from .timeutil import gpst_str_to_gpst_epoch, gpst_str_to_utc_epoch

COMMENT_PREFIX = "%"
N_COLUMNS = 24

# Defaults for the "parked on the ground at the start of the log" detector used
# to reference heights to the takeoff point (see `estimate_ground_height`).
DEFAULT_STATIONARY_SPEED_MPS = 0.5
DEFAULT_TAKEOFF_WINDOW_S = 30.0
MIN_STATIONARY_ROWS = 10

# Column order after the date+time pair has been consumed.
VALUE_COLUMNS = (
    "latitude_deg", "longitude_deg", "height_m", "Q", "ns",
    "sdn_m", "sde_m", "sdu_m", "sdne_m", "sdeu_m", "sdun_m",
    "age_s", "ratio", "roll_deg", "pitch_deg", "yaw_deg",
    "P_deg_s", "Q_deg_s", "R_deg_s", "Ve_m_s", "Vn_m_s", "Vu_m_s",
)

# Fields interpolated linearly by `interpolate`.
_LINEAR_FIELDS = (
    "latitude_deg", "longitude_deg", "height_m",
    "Ve_m_s", "Vn_m_s", "Vu_m_s",
    "P_deg_s", "Q_deg_s", "R_deg_s",
)
# Fields interpolated along the shortest arc. ``True`` = report in [0, 360)
# (yaw / heading convention), ``False`` = report in (-180, 180] (signed
# roll/pitch as they appear in the file).
_ANGULAR_FIELDS = {"roll_deg": False, "pitch_deg": False, "yaw_deg": True}
# Fields taken from the nearest sample (discrete / not meaningfully averaged).
_NEAREST_FIELDS = ("Q", "ns")

_DTYPE = np.dtype(
    [("t_utc", "<f8"), ("t_gpst", "<f8")]
    + [(name, "<i4" if name in ("Q", "ns") else "<f8") for name in VALUE_COLUMNS]
)


def read_pos(path: str | Path) -> np.ndarray:
    """Parse an RTKLIB `.pos` file into a structured numpy array.

    Returned dtype carries `t_utc` and `t_gpst` (POSIX seconds) followed by
    every column of the file. Malformed rows are skipped silently; a row count
    mismatch is the caller's cue that something is wrong.
    """
    rows: list[tuple] = []
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith(COMMENT_PREFIX):
                continue
            tok = line.split()
            if len(tok) < N_COLUMNS:
                continue
            try:
                t_utc = gpst_str_to_utc_epoch(tok[0], tok[1])
                t_gpst = gpst_str_to_gpst_epoch(tok[0], tok[1])
                values = [float(x) for x in tok[2:N_COLUMNS]]
            except ValueError:
                continue
            rows.append((t_utc, t_gpst, *values))
    if not rows:
        raise ValueError(f"no data rows parsed from '{path}'")
    return np.array(rows, dtype=_DTYPE)


def read_pos_csv(path: str | Path) -> np.ndarray:
    """Read back the CSV produced by `pos_to_csv.py` into the `read_pos` dtype.

    Accepts either the `t_utc`/`t_gpst` names used internally or the
    `t_utc_epoch`/`t_gpst_epoch` names emitted by the CLI.
    """
    import csv as _csv

    aliases = {"t_utc": ("t_utc", "t_utc_epoch"), "t_gpst": ("t_gpst", "t_gpst_epoch")}
    rows: list[tuple] = []
    with open(path, "r", encoding="utf-8", newline="") as fh:
        reader = _csv.DictReader(fh)
        if reader.fieldnames is None:
            raise ValueError(f"'{path}' has no header row")
        for row in reader:
            values = []
            for name in _DTYPE.names:
                key = next((a for a in aliases.get(name, (name,)) if a in row), None)
                if key is None:
                    raise ValueError(f"'{path}' is missing the '{name}' column")
                values.append(float(row[key]))
            rows.append(tuple(values))
    if not rows:
        raise ValueError(f"no data rows in '{path}'")
    return np.array(rows, dtype=_DTYPE)


def read_any(path: str | Path) -> np.ndarray:
    """Read a PPK solution from either an RTKLIB `.pos` or a `pos_to_csv.py` CSV."""
    return read_pos_csv(path) if Path(path).suffix.lower() == ".csv" else read_pos(path)


def read_pos_comments(path: str | Path) -> list[str]:
    """Return the leading `%` comment lines of a `.pos` file."""
    out: list[str] = []
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        for line in fh:
            if not line.startswith(COMMENT_PREFIX):
                break
            out.append(line.rstrip("\n"))
    return out


def q_histogram(records: np.ndarray) -> dict[int, int]:
    """Count solution-quality flags: ``{Q: n_rows}``."""
    values, counts = np.unique(records["Q"], return_counts=True)
    return {int(v): int(c) for v, c in zip(values, counts)}


def span(records: np.ndarray) -> tuple[float, float]:
    """``(t_first_utc, t_last_utc)`` of the solution, in POSIX seconds."""
    return float(records["t_utc"][0]), float(records["t_utc"][-1])


def stationary_mask(records: np.ndarray,
                    window_s: float = DEFAULT_TAKEOFF_WINDOW_S,
                    speed_threshold_mps: float = DEFAULT_STATIONARY_SPEED_MPS
                    ) -> np.ndarray:
    """Boolean mask of rows that look parked on the ground at the start of the log.

    A row qualifies when it falls inside the first `window_s` seconds of the
    solution **and** both the horizontal speed ``hypot(Ve, Vn)`` and ``|Vu|``
    stay below `speed_threshold_mps`.
    """
    if window_s <= 0.0 or speed_threshold_mps <= 0.0:
        raise ValueError("window_s and speed_threshold_mps must both be > 0")
    elapsed = records["t_utc"] - records["t_utc"][0]
    speed = np.hypot(records["Ve_m_s"], records["Vn_m_s"])
    return ((elapsed <= window_s)
            & (speed < speed_threshold_mps)
            & (np.abs(records["Vu_m_s"]) < speed_threshold_mps))


def estimate_ground_height(records: np.ndarray,
                           window_s: float = DEFAULT_TAKEOFF_WINDOW_S,
                           speed_threshold_mps: float = DEFAULT_STATIONARY_SPEED_MPS,
                           min_rows: int = MIN_STATIONARY_ROWS
                           ) -> tuple[float, int]:
    """Height of the ground at the takeoff point, from the parked start of the log.

    Returns ``(h_ground_m, n_stationary_rows)`` where `h_ground_m` is the
    **median** `.pos` height over `stationary_mask` — a median rather than a mean
    so a single bad epoch cannot move it. Raises `ValueError` when fewer than
    `min_rows` rows qualify, because a ground reference guessed from a couple of
    samples is worse than no reference at all.
    """
    mask = stationary_mask(records, window_s, speed_threshold_mps)
    n = int(np.count_nonzero(mask))
    if n < min_rows:
        raise ValueError(
            f"only {n} stationary row(s) found in the first {window_s:g} s "
            f"(speed < {speed_threshold_mps:g} m/s), need >= {min_rows}; widen "
            "--takeoff-window-s / --stationary-speed-mps, or pass the ground "
            "height explicitly with --ground-height"
        )
    return float(np.median(records["height_m"][mask])), n


def _lerp_angle_deg(a0: float, a1: float, w: float, wrap360: bool) -> float:
    """Interpolate degrees along the shortest arc (unwrapped through +/-180).

    `wrap360` selects the output branch: ``True`` -> [0, 360) for yaw/heading,
    ``False`` -> (-180, 180] for signed roll/pitch.
    """
    delta = (a1 - a0 + 180.0) % 360.0 - 180.0
    value = a0 + delta * w
    if wrap360:
        return value % 360.0
    return (value + 180.0) % 360.0 - 180.0


def interpolate(records: np.ndarray, t_utc: float) -> Optional[dict]:
    """Interpolate the PPK solution at UTC epoch `t_utc`.

    Linear on position/velocity/rates, shortest-arc on roll/pitch/yaw, nearest
    sample on the discrete `Q`/`ns` flags. **Never extrapolates**: returns
    ``None`` when `t_utc` lies outside the solution span, leaving the drop /
    raise decision to the caller.
    """
    times = records["t_utc"]
    if not math.isfinite(t_utc) or t_utc < times[0] or t_utc > times[-1]:
        return None

    idx = int(np.searchsorted(times, t_utc, side="right")) - 1
    idx = min(max(idx, 0), len(times) - 1)
    if idx >= len(times) - 1:
        lo = hi = len(times) - 1
        w = 0.0
    else:
        lo, hi = idx, idx + 1
        dt = float(times[hi] - times[lo])
        w = 0.0 if dt <= 0.0 else (t_utc - float(times[lo])) / dt

    r0, r1 = records[lo], records[hi]
    out: dict = {"t_utc": float(t_utc)}
    out["t_gpst"] = float(r0["t_gpst"]) + (float(r1["t_gpst"]) - float(r0["t_gpst"])) * w
    for name in _LINEAR_FIELDS:
        out[name] = float(r0[name]) + (float(r1[name]) - float(r0[name])) * w
    for name, wrap360 in _ANGULAR_FIELDS.items():
        out[name] = _lerp_angle_deg(float(r0[name]), float(r1[name]), w, wrap360)
    for name in _NEAREST_FIELDS:
        out[name] = int(r0[name] if w < 0.5 else r1[name])
    out["speed_mps"] = math.hypot(out["Ve_m_s"], out["Vn_m_s"])
    out["climb_mps"] = out["Vu_m_s"]
    return out


def speed_climb(records: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Derived ``(speed_mps, climb_mps)`` columns for a whole record array."""
    speed = np.hypot(records["Ve_m_s"], records["Vn_m_s"])
    return speed, records["Vu_m_s"].copy()
