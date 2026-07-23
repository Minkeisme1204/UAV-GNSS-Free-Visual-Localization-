#!/usr/bin/env python3
"""pos_to_csv.py — RTKLIB `.pos` PPK solution -> a plain CSV.

Emits every column of the `.pos` file plus:
  * `t_gpst_epoch` — the raw GPS-time stamp as POSIX seconds,
  * `t_utc_epoch` / `timestamp_msec` — the UTC stamp the ROS bag uses
    (`t_utc = t_gpst - 18 s`, see `munfrl.timeutil.GPS_UTC_LEAP_SECONDS`),
  * `utc_string` — a human-readable `YYYY/MM/DD HH:MM:SS.sss`,
  * `speed_mps = hypot(Ve, Vn)` and `climb_mps = Vu`.

Reminders carried over from the file's own header comment:
  * `height_m` is WGS84 **ellipsoidal** — it is neither MSL nor AGL,
  * `roll/pitch/yaw` are **NED** frame degrees (yaw 0 = North, clockwise),
  * `Q` is the solution quality (69 = PPP-converged TerraStar for these files).

Example:
  ./pos_to_csv.py /media/.../ppk_data/bell412_dataset6_frl.pos
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np

from munfrl import DEFAULT_OUTPUT_ROOT, csvout, pos, timeutil

OUTPUT_NAME = "ppk.csv"

OUTPUT_COLUMNS = (
    "timestamp_msec", "t_utc_epoch", "t_gpst_epoch", "utc_string",
    "latitude_deg", "longitude_deg", "height_m",
    "Q", "ns",
    "sdn_m", "sde_m", "sdu_m", "sdne_m", "sdeu_m", "sdun_m",
    "age_s", "ratio",
    "roll_deg", "pitch_deg", "yaw_deg",
    "P_deg_s", "Q_deg_s", "R_deg_s",
    "Ve_m_s", "Vn_m_s", "Vu_m_s",
    "speed_mps", "climb_mps",
)


def rows_from_records(records: np.ndarray):
    speed, climb = pos.speed_climb(records)
    for i, rec in enumerate(records):
        yield [
            timeutil.format_msec(float(rec["t_utc"])),
            f"{float(rec['t_utc']):.6f}",
            f"{float(rec['t_gpst']):.6f}",
            timeutil.epoch_to_utc_string(float(rec["t_utc"])),
            f"{float(rec['latitude_deg']):.9f}",
            f"{float(rec['longitude_deg']):.9f}",
            f"{float(rec['height_m']):.4f}",
            int(rec["Q"]), int(rec["ns"]),
            *(f"{float(rec[c]):.4f}" for c in
              ("sdn_m", "sde_m", "sdu_m", "sdne_m", "sdeu_m", "sdun_m",
               "age_s", "ratio", "roll_deg", "pitch_deg", "yaw_deg",
               "P_deg_s", "Q_deg_s", "R_deg_s", "Ve_m_s", "Vn_m_s", "Vu_m_s")),
            f"{float(speed[i]):.4f}",
            f"{float(climb[i]):.4f}",
        ]


def main() -> int:
    p = argparse.ArgumentParser(
        description="Convert an RTKLIB .pos PPK solution to CSV (GPST + UTC columns).",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("pos_file", help="Path to the RTKLIB .pos file.")
    p.add_argument("--out", default=str(DEFAULT_OUTPUT_ROOT), help="Output directory.")
    p.add_argument("--name", default=OUTPUT_NAME, help="Output CSV file name.")
    args = p.parse_args()

    src = Path(args.pos_file)
    if not src.is_file():
        print(f"ERROR: '{src}' is not a file", file=sys.stderr)
        return 2

    records = pos.read_pos(src)
    out_path = Path(args.out) / args.name
    n = csvout.write_rows(out_path, OUTPUT_COLUMNS, rows_from_records(records))

    t0, t1 = pos.span(records)
    print(f"Rows           : {n}")
    print(f"GPST span      : {timeutil.epoch_to_utc_string(float(records['t_gpst'][0]))}"
          f" .. {timeutil.epoch_to_utc_string(float(records['t_gpst'][-1]))}")
    print(f"UTC span       : {timeutil.epoch_to_utc_string(t0)}"
          f" .. {timeutil.epoch_to_utc_string(t1)}"
          f"   (leap seconds applied: {timeutil.GPS_UTC_LEAP_SECONDS})")
    print(f"Rate           : {(n - 1) / (t1 - t0):.3f} Hz" if t1 > t0 else "")
    print(f"Q histogram    : {pos.q_histogram(records)}")
    print(f"Height (ellips): [{records['height_m'].min():.3f}, "
          f"{records['height_m'].max():.3f}] m  (NOT MSL, NOT AGL)")
    speed, _ = pos.speed_climb(records)
    print(f"Ground speed   : [{speed.min():.3f}, {speed.max():.3f}] m/s")
    print(f"Written        : {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
