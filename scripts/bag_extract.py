#!/usr/bin/env python3
"""bag_extract.py — export the non-image MUN-FRL bag topics to CSV / binary.

Everything selected is written in ONE merged traversal of the bag (the reader
iterates the union of the selected connections in log-time order), so asking for
ten topics costs one pass, not ten.

Topics handled
--------------
  /imu/data            -> imu_data.csv
  /imu/data_stamped    -> imu_data_stamped.csv  (custom xsens msg; adds time_ref)
  /imu/mag             -> imu_mag.csv
  /imu/time_ref        -> time_ref_imu.csv
  /imu/time_ref_cam    -> time_ref_cam.csv
  /imu/time_ref_pps    -> time_ref_pps.csv
  /time_ref_scan       -> time_ref_scan.csv
  /fix                 -> fix.csv
  /nmea_sentence       -> nmea_sentence.csv (raw) + nmea_gga.csv (parsed GGA;
                          rows failing the `*` XOR checksum are skipped)
  /scan                -> scan_meta.csv (+ scan_ranges.npy with --scan-ranges)
  /velodyne_points     -> lidar/NNNNNN.bin + lidar/index.csv  (with --lidar)

Each `.bin` is a flat float32 `x,y,z,intensity` array — the KITTI velodyne
layout — readable with `np.fromfile(p, np.float32).reshape(-1, 4)`.

Examples:
  ./bag_extract.py /media/.../bell412_dataset6                 # everything but LiDAR
  ./bag_extract.py /media/.../bell412_dataset6 --fix --nmea
  ./bag_extract.py /media/.../bell412_dataset6 --lidar --max-messages 100
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np

from munfrl import DEFAULT_OUTPUT_ROOT, bagio, timeutil

TOPIC_IMU = "/imu/data"
TOPIC_IMU_STAMPED = "/imu/data_stamped"
TOPIC_MAG = "/imu/mag"
TOPIC_FIX = "/fix"
TOPIC_NMEA = "/nmea_sentence"
TOPIC_SCAN = "/scan"
TOPIC_LIDAR = "/velodyne_points"
TIME_REF_TOPICS = {
    "/imu/time_ref": "time_ref_imu.csv",
    "/imu/time_ref_cam": "time_ref_cam.csv",
    "/imu/time_ref_pps": "time_ref_pps.csv",
    "/time_ref_scan": "time_ref_scan.csv",
}

LIDAR_DIRNAME = "lidar"
LIDAR_INDEX_NAME = "index.csv"
LIDAR_FILE_TEMPLATE = "{:06d}.bin"
# Rough count for the warning below (bell412_dataset6 has 5160 Velodyne scans).
LIDAR_FILE_WARN_THRESHOLD = 500

PROGRESS_EVERY = 20000

# Common leading columns on every per-message CSV.
BASE_COLUMNS = ("timestamp_msec", "bag_msec", "seq", "frame_id")

NMEA_GGA_COLUMNS = (
    "timestamp_msec", "bag_msec", "talker", "utc_hhmmss", "utc_epoch",
    "latitude_deg", "longitude_deg", "fix_quality", "num_satellites",
    "hdop", "altitude_msl_m", "geoid_separation_m", "sentence",
)


# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #
def _hdr(msg, t_bag_ns: int) -> list:
    """The four BASE_COLUMNS values for a stamped message."""
    t_hdr = timeutil.ros_stamp_to_epoch(msg.header.stamp)
    if t_hdr <= 0:
        t_hdr = timeutil.ns_to_epoch(t_bag_ns)
    return [
        timeutil.format_msec(t_hdr),
        timeutil.format_msec(timeutil.ns_to_epoch(t_bag_ns)),
        getattr(msg.header, "seq", ""),
        msg.header.frame_id,
    ]


def _diag3(cov) -> list:
    """The three diagonal entries of a row-major 3x3 covariance."""
    arr = np.asarray(cov, dtype=np.float64).reshape(-1)
    if arr.size < 9:
        return ["", "", ""]
    return [float(arr[0]), float(arr[4]), float(arr[8])]


def nmea_checksum_ok(sentence: str) -> bool:
    """Validate the NMEA `*hh` XOR checksum over the payload between `$` and `*`."""
    s = sentence.strip()
    if not s.startswith("$") or "*" not in s:
        return False
    body, _, tail = s[1:].partition("*")
    given = tail[:2].strip().upper()
    if len(given) != 2:
        return False
    calc = 0
    for ch in body:
        calc ^= ord(ch)
    return f"{calc:02X}" == given


def _ddmm_to_deg(token: str, hemisphere: str) -> float | None:
    """NMEA `ddmm.mmmm` + hemisphere letter -> signed decimal degrees."""
    if not token:
        return None
    dot = token.find(".")
    if dot < 3:
        return None
    deg = float(token[: dot - 2])
    minutes = float(token[dot - 2:])
    value = deg + minutes / 60.0
    if hemisphere.upper() in ("S", "W"):
        value = -value
    return value


def parse_gga(sentence: str, bag_epoch: float) -> dict | None:
    """Parse a `$--GGA` sentence. Returns None when it is not a valid GGA."""
    s = sentence.strip()
    if not nmea_checksum_ok(s):
        return None
    body = s[1:].split("*")[0]
    f = body.split(",")
    if len(f) < 12 or not f[0].endswith("GGA"):
        return None
    try:
        utc_epoch = timeutil.nmea_utc_to_epoch(f[1], bag_epoch) if f[1] else None
    except ValueError:
        utc_epoch = None
    def num(tok, cast=float):
        try:
            return cast(tok)
        except (TypeError, ValueError):
            return None
    return {
        "talker": f[0][:2],
        "utc_hhmmss": f[1],
        "utc_epoch": utc_epoch,
        "latitude_deg": _ddmm_to_deg(f[2], f[3]),
        "longitude_deg": _ddmm_to_deg(f[4], f[5]),
        "fix_quality": num(f[6], int),
        "num_satellites": num(f[7], int),
        "hdop": num(f[8]),
        "altitude_msl_m": num(f[9]),
        "geoid_separation_m": num(f[11]),
    }


class CsvSink:
    """A CSV file plus its header, opened lazily and closed by the caller."""

    def __init__(self, path: Path, columns):
        path.parent.mkdir(parents=True, exist_ok=True)
        self.path = path
        self._fh = open(path, "w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._fh)
        self._writer.writerow(list(columns))
        self.rows = 0

    def write(self, row) -> None:
        self._writer.writerow(row)
        self.rows += 1

    def close(self) -> None:
        self._fh.close()


# --------------------------------------------------------------------------- #
# per-topic handler construction
# --------------------------------------------------------------------------- #
def build_handlers(selected: set[str], out_dir: Path, keep_scan_ranges: bool):
    """Return ``(handlers, sinks, extras)``.

    `handlers` maps topic -> callable(t_bag_ns, msg); `sinks` is the list of open
    CsvSinks; `extras` holds side artefacts that need finalising.
    """
    sinks: list[CsvSink] = []
    handlers: dict[str, callable] = {}
    extras: dict = {"scan_ranges": [], "lidar_count": 0}

    def add(path_name: str, columns) -> CsvSink:
        sink = CsvSink(out_dir / path_name, columns)
        sinks.append(sink)
        return sink

    if TOPIC_IMU in selected:
        sink = add("imu_data.csv", BASE_COLUMNS + (
            "qx", "qy", "qz", "qw",
            "orientation_cov_xx", "orientation_cov_yy", "orientation_cov_zz",
            "wx_rad_s", "wy_rad_s", "wz_rad_s",
            "angular_velocity_cov_xx", "angular_velocity_cov_yy",
            "angular_velocity_cov_zz",
            "ax_m_s2", "ay_m_s2", "az_m_s2",
            "linear_acceleration_cov_xx", "linear_acceleration_cov_yy",
            "linear_acceleration_cov_zz",
        ))

        def on_imu(t_ns, msg, sink=sink):
            o, w, a = msg.orientation, msg.angular_velocity, msg.linear_acceleration
            sink.write(_hdr(msg, t_ns)
                       + [o.x, o.y, o.z, o.w] + _diag3(msg.orientation_covariance)
                       + [w.x, w.y, w.z] + _diag3(msg.angular_velocity_covariance)
                       + [a.x, a.y, a.z] + _diag3(msg.linear_acceleration_covariance))

        handlers[TOPIC_IMU] = on_imu

    if TOPIC_IMU_STAMPED in selected:
        sink = add("imu_data_stamped.csv", BASE_COLUMNS + (
            "time_ref_msec", "qx", "qy", "qz", "qw",
            "wx_rad_s", "wy_rad_s", "wz_rad_s",
            "ax_m_s2", "ay_m_s2", "az_m_s2",
        ))

        def on_imu_stamped(t_ns, msg, sink=sink):
            o, w, a = msg.orientation, msg.angular_velocity, msg.linear_acceleration
            t_ref = timeutil.ros_stamp_to_epoch(msg.time_ref)
            sink.write(_hdr(msg, t_ns) + [timeutil.format_msec(t_ref)]
                       + [o.x, o.y, o.z, o.w]
                       + [w.x, w.y, w.z]
                       + [a.x, a.y, a.z])

        handlers[TOPIC_IMU_STAMPED] = on_imu_stamped

    if TOPIC_MAG in selected:
        sink = add("imu_mag.csv", BASE_COLUMNS + ("mx", "my", "mz"))

        def on_mag(t_ns, msg, sink=sink):
            v = msg.vector
            sink.write(_hdr(msg, t_ns) + [v.x, v.y, v.z])

        handlers[TOPIC_MAG] = on_mag

    for topic, filename in TIME_REF_TOPICS.items():
        if topic not in selected:
            continue
        sink = add(filename, BASE_COLUMNS + ("time_ref_msec", "source"))

        def on_time_ref(t_ns, msg, sink=sink):
            sink.write(_hdr(msg, t_ns)
                       + [timeutil.format_msec(timeutil.ros_stamp_to_epoch(msg.time_ref)),
                          msg.source])

        handlers[topic] = on_time_ref

    if TOPIC_FIX in selected:
        sink = add("fix.csv", BASE_COLUMNS + (
            "status", "service", "latitude_deg", "longitude_deg",
            "altitude_m", "cov_ee", "cov_nn", "cov_uu", "covariance_type",
        ))

        def on_fix(t_ns, msg, sink=sink):
            sink.write(_hdr(msg, t_ns)
                       + [msg.status.status, msg.status.service,
                          msg.latitude, msg.longitude, msg.altitude]
                       + _diag3(msg.position_covariance)
                       + [msg.position_covariance_type])

        handlers[TOPIC_FIX] = on_fix

    if TOPIC_NMEA in selected:
        raw_sink = add("nmea_sentence.csv",
                       ("timestamp_msec", "bag_msec", "frame_id", "sentence"))
        gga_sink = add("nmea_gga.csv", NMEA_GGA_COLUMNS)

        def on_nmea(t_ns, msg, raw_sink=raw_sink, gga_sink=gga_sink):
            t_hdr = timeutil.ros_stamp_to_epoch(msg.header.stamp)
            t_bag = timeutil.ns_to_epoch(t_ns)
            if t_hdr <= 0:
                t_hdr = t_bag
            sentence = msg.sentence.strip()
            raw_sink.write([timeutil.format_msec(t_hdr), timeutil.format_msec(t_bag),
                            msg.header.frame_id, sentence])
            parsed = parse_gga(sentence, t_bag)
            if parsed is None:
                return
            gga_sink.write([
                timeutil.format_msec(t_hdr), timeutil.format_msec(t_bag),
                parsed["talker"], parsed["utc_hhmmss"],
                "" if parsed["utc_epoch"] is None else f"{parsed['utc_epoch']:.3f}",
                parsed["latitude_deg"], parsed["longitude_deg"],
                parsed["fix_quality"], parsed["num_satellites"], parsed["hdop"],
                parsed["altitude_msl_m"], parsed["geoid_separation_m"], sentence,
            ])

        handlers[TOPIC_NMEA] = on_nmea

    if TOPIC_SCAN in selected:
        sink = add("scan_meta.csv", BASE_COLUMNS + (
            "angle_min_rad", "angle_max_rad", "angle_increment_rad",
            "time_increment_s", "scan_time_s", "range_min_m", "range_max_m",
            "num_ranges", "num_intensities",
        ))

        def on_scan(t_ns, msg, sink=sink):
            sink.write(_hdr(msg, t_ns) + [
                msg.angle_min, msg.angle_max, msg.angle_increment,
                msg.time_increment, msg.scan_time, msg.range_min, msg.range_max,
                len(msg.ranges), len(msg.intensities),
            ])
            if keep_scan_ranges:
                extras["scan_ranges"].append(np.asarray(msg.ranges, dtype=np.float32))

        handlers[TOPIC_SCAN] = on_scan

    if TOPIC_LIDAR in selected:
        lidar_dir = out_dir / LIDAR_DIRNAME
        lidar_dir.mkdir(parents=True, exist_ok=True)
        sink = add(f"{LIDAR_DIRNAME}/{LIDAR_INDEX_NAME}",
                   ("scan_id", "timestamp_msec", "bag_msec", "num_points", "file"))

        def on_lidar(t_ns, msg, sink=sink, lidar_dir=lidar_dir):
            scan_id = extras["lidar_count"]
            xyzi = bagio.pointcloud2_xyzi(msg)
            filename = LIDAR_FILE_TEMPLATE.format(scan_id)
            xyzi.tofile(lidar_dir / filename)
            t_hdr = timeutil.ros_stamp_to_epoch(msg.header.stamp)
            t_bag = timeutil.ns_to_epoch(t_ns)
            sink.write([scan_id, timeutil.format_msec(t_hdr or t_bag),
                        timeutil.format_msec(t_bag), int(xyzi.shape[0]), filename])
            extras["lidar_count"] = scan_id + 1

        handlers[TOPIC_LIDAR] = on_lidar

    return handlers, sinks, extras


# --------------------------------------------------------------------------- #
def selected_topics(args) -> set[str]:
    """Resolve the CLI flags into the set of topics to extract."""
    chosen: set[str] = set()
    if args.imu:
        chosen.add(TOPIC_IMU)
    if args.imu_stamped:
        chosen.add(TOPIC_IMU_STAMPED)
    if args.mag:
        chosen.add(TOPIC_MAG)
    if args.time_ref:
        chosen.update(TIME_REF_TOPICS)
    if args.fix:
        chosen.add(TOPIC_FIX)
    if args.nmea:
        chosen.add(TOPIC_NMEA)
    if args.scan or args.scan_ranges:
        chosen.add(TOPIC_SCAN)
    if args.lidar:
        chosen.add(TOPIC_LIDAR)
    if args.all:
        chosen.update({TOPIC_IMU, TOPIC_IMU_STAMPED, TOPIC_MAG, TOPIC_FIX,
                       TOPIC_NMEA, TOPIC_SCAN, TOPIC_LIDAR})
        chosen.update(TIME_REF_TOPICS)
    if not chosen:
        # Default: everything cheap — LiDAR is opt-in because it writes ~5160 files.
        chosen = {TOPIC_IMU, TOPIC_IMU_STAMPED, TOPIC_MAG, TOPIC_FIX, TOPIC_NMEA,
                  TOPIC_SCAN}
        chosen.update(TIME_REF_TOPICS)
        print("No topic flags given — extracting every non-image topic except "
              "/velodyne_points (use --lidar for that).")
    return chosen


def main() -> int:
    p = argparse.ArgumentParser(
        description="Export non-image MUN-FRL bag topics to CSV in a single pass.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("path", help="A .bag file, or a dataset/split-bag directory.")
    p.add_argument("--out", default=str(DEFAULT_OUTPUT_ROOT), help="Output directory.")
    p.add_argument("--imu", action="store_true", help="Extract /imu/data.")
    p.add_argument("--imu-stamped", action="store_true", help="Extract /imu/data_stamped.")
    p.add_argument("--mag", action="store_true", help="Extract /imu/mag.")
    p.add_argument("--time-ref", action="store_true",
                   help="Extract all four TimeReference topics.")
    p.add_argument("--fix", action="store_true", help="Extract /fix.")
    p.add_argument("--nmea", action="store_true",
                   help="Extract /nmea_sentence (raw + parsed GGA).")
    p.add_argument("--scan", action="store_true", help="Extract /scan metadata.")
    p.add_argument("--scan-ranges", action="store_true",
                   help="Also store the full /scan ranges as scan_ranges.npy.")
    p.add_argument("--lidar", action="store_true",
                   help="Extract /velodyne_points to lidar/NNNNNN.bin "
                        "(WARNING: ~5160 files, several GB).")
    p.add_argument("--all", action="store_true", help="Extract every topic above.")
    p.add_argument("--max-messages", type=int, default=0,
                   help="Stop after N messages total (0 = all). For smoke tests.")
    args = p.parse_args()

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    chosen = selected_topics(args)

    if TOPIC_LIDAR in chosen:
        print(f"WARNING: --lidar writes one .bin per scan into {out_dir / LIDAR_DIRNAME}/ "
              f"(this dataset has ~5160 scans, > {LIDAR_FILE_WARN_THRESHOLD} files).")

    handlers, sinks, extras = build_handlers(chosen, out_dir, args.scan_ranges)
    if not handlers:
        print("ERROR: nothing selected.", file=sys.stderr)
        return 2

    n_total = 0
    try:
        with bagio.open_bags(args.path) as reader:
            available = {c.topic for c in reader.connections}
            missing = sorted(set(handlers) - available)
            for topic in missing:
                print(f"WARNING: topic '{topic}' not in bag — skipped.")
                handlers.pop(topic)
            conns = [c for c in reader.connections if c.topic in handlers]
            if not conns:
                print("ERROR: none of the selected topics exist in this bag.",
                      file=sys.stderr)
                return 2
            print(f"Reading {len(conns)} connection(s): {', '.join(sorted(handlers))}")
            for conn, t_ns, raw in reader.messages(connections=conns):
                handlers[conn.topic](t_ns, reader.deserialize(raw, conn.msgtype))
                n_total += 1
                if n_total % PROGRESS_EVERY == 0:
                    print(f"  {n_total} messages", end="\r", flush=True)
                if args.max_messages and n_total >= args.max_messages:
                    print(f"\nStopping at --max-messages {args.max_messages}.")
                    break
    finally:
        for sink in sinks:
            sink.close()

    if args.scan_ranges and extras["scan_ranges"]:
        lengths = {a.size for a in extras["scan_ranges"]}
        ranges_path = out_dir / "scan_ranges.npy"
        if len(lengths) == 1:
            np.save(ranges_path, np.stack(extras["scan_ranges"]))
        else:
            print(f"WARNING: /scan ranges have varying lengths {sorted(lengths)}; "
                  "saving as an object array.")
            np.save(ranges_path, np.array(extras["scan_ranges"], dtype=object),
                    allow_pickle=True)
        print(f"  {ranges_path}")

    print(f"\nRead {n_total} messages.")
    for sink in sinks:
        print(f"  {sink.rows:>8d} rows  {sink.path}")
    if extras["lidar_count"]:
        print(f"  {extras['lidar_count']:>8d} files  {out_dir / LIDAR_DIRNAME}/")
    return 0


if __name__ == "__main__":
    sys.exit(main())
