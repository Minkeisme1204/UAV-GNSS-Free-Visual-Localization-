#!/usr/bin/env python3
"""inspect_dataset.py — fast survey of a MUN-FRL `bell412_datasetN` directory.

Lists every artefact by kind and summarises each one WITHOUT a full scan of the
51 GB bag:

  * bag   — topic table (type / count / rate / span) from the bag's own index,
            plus the first message of each image topic for encoding+resolution
  * .pos  — row count, span in GPST and UTC, solution-quality (Q) histogram
  * .kml  — vertex count and bounding box
  * .pcd  — header, point count, per-axis bounds
  * map   — geo-parameters parsed from the file name + metric extent

It also runs the **leap-second sanity check**: the `.pos` clock is GPST while
the bag and NMEA clocks are UTC, and the two must line up after subtracting
`munfrl.timeutil.GPS_UTC_LEAP_SECONDS`.

A machine-readable JSON summary is written to `--out`.

Examples:
  ./inspect_dataset.py /media/.../bell412_dataset6
  ./inspect_dataset.py /media/.../bell412_dataset6 --out /tmp/summary --no-pcd
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from munfrl import DEFAULT_OUTPUT_ROOT, bagio, geomap, kmlio, pcdio, pos, timeutil

SUMMARY_FILENAME = "dataset_summary.json"

BAG_SUFFIXES = (".bag",)
POS_SUFFIXES = (".pos",)
KML_SUFFIXES = (".kml",)
PCD_SUFFIXES = (".pcd",)
MAP_SUFFIXES = (".jpg", ".jpeg", ".png", ".tif", ".tiff")

IMAGE_TOPIC_HINT = "image"

# The bag and the PPK solution are separate recordings, so their start times
# never match exactly; anything beyond this after the leap-second correction
# means the time bases are wrong, not merely offset.
LEAP_CHECK_TOLERANCE_S = 5.0


# --------------------------------------------------------------------------- #
def find_files(root: Path) -> dict[str, list[Path]]:
    """Classify every file under `root` (recursively) by artefact kind."""
    kinds = {"bag": [], "pos": [], "kml": [], "pcd": [], "map": [], "other": []}
    for path in sorted(root.rglob("*")):
        if not path.is_file():
            continue
        suffix = path.suffix.lower()
        if suffix in BAG_SUFFIXES:
            kinds["bag"].append(path)
        elif suffix in POS_SUFFIXES:
            kinds["pos"].append(path)
        elif suffix in KML_SUFFIXES:
            kinds["kml"].append(path)
        elif suffix in PCD_SUFFIXES:
            kinds["pcd"].append(path)
        elif suffix in MAP_SUFFIXES:
            kinds["map"].append(path)
        else:
            kinds["other"].append(path)
    return kinds


def image_size(path: Path) -> tuple[int, int]:
    """(width, height) of a raster; returns (0, 0) when it cannot be read."""
    import cv2

    img = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if img is None:
        return 0, 0
    return int(img.shape[1]), int(img.shape[0])


# --------------------------------------------------------------------------- #
def inspect_bags(bag_paths: list[Path]) -> dict:
    """Topic table + image metadata + first NMEA sentence, index-driven."""
    out: dict = {"files": [str(p) for p in bag_paths]}
    with bagio.open_bags(bag_paths) as reader:
        t0 = timeutil.ns_to_epoch(reader.start_time)
        t1 = timeutil.ns_to_epoch(reader.end_time)
        duration = reader.duration * 1e-9
        out["start_epoch"] = t0
        out["end_epoch"] = t1
        out["start_utc"] = timeutil.epoch_to_utc_string(t0)
        out["end_utc"] = timeutil.epoch_to_utc_string(t1)
        out["duration_s"] = duration
        out["message_count"] = reader.message_count

        topics = []
        for topic, info in bagio.topic_index(reader).items():
            times = bagio.topic_bag_times(reader, topic)
            if times.size >= 2:
                span_s = float(times[-1] - times[0]) * 1e-9
                rate = (times.size - 1) / span_s if span_s > 0 else 0.0
            else:
                span_s, rate = 0.0, 0.0
            entry = {
                "topic": topic,
                "msgtype": info["msgtype"],
                "count": info["count"],
                "rate_hz": round(rate, 3),
                "span_s": round(span_s, 3),
            }
            if IMAGE_TOPIC_HINT in topic:
                first = bagio.first_message(reader, topic)
                if first is not None:
                    _, msg = first
                    entry.update(
                        encoding=msg.encoding,
                        width=int(msg.width),
                        height=int(msg.height),
                        step=int(msg.step),
                        first_header_stamp=timeutil.ros_stamp_to_epoch(msg.header.stamp),
                    )
            topics.append(entry)
        out["topics"] = topics

        # First NMEA GGA — the independent UTC witness for the leap-second check.
        try:
            first = bagio.first_message(reader, "/nmea_sentence")
        except KeyError:
            first = None
        if first is not None:
            t_ns, msg = first
            out["first_nmea"] = {
                "bag_epoch": timeutil.ns_to_epoch(t_ns),
                "sentence": msg.sentence.strip(),
            }
    return out


def inspect_pos(path: Path) -> dict:
    records = pos.read_pos(path)
    t0, t1 = pos.span(records)
    return {
        "file": str(path),
        "rows": int(records.size),
        "comments": pos.read_pos_comments(path),
        "utc_start": timeutil.epoch_to_utc_string(t0),
        "utc_end": timeutil.epoch_to_utc_string(t1),
        "utc_start_epoch": t0,
        "utc_end_epoch": t1,
        "gpst_start": timeutil.epoch_to_utc_string(float(records["t_gpst"][0])),
        "gpst_end": timeutil.epoch_to_utc_string(float(records["t_gpst"][-1])),
        "duration_s": round(t1 - t0, 3),
        "rate_hz": round((records.size - 1) / (t1 - t0), 3) if t1 > t0 else 0.0,
        "q_histogram": pos.q_histogram(records),
        "lat_range": [float(records["latitude_deg"].min()),
                      float(records["latitude_deg"].max())],
        "lon_range": [float(records["longitude_deg"].min()),
                      float(records["longitude_deg"].max())],
        "height_range_m": [float(records["height_m"].min()),
                           float(records["height_m"].max())],
    }


def inspect_kml(path: Path) -> dict:
    track = kmlio.read_kml_track(path)
    return {
        "file": str(path),
        "points": int(track.shape[0]),
        "lon_range": [float(track[:, 0].min()), float(track[:, 0].max())],
        "lat_range": [float(track[:, 1].min()), float(track[:, 1].max())],
        "alt_range_m": [float(track[:, 2].min()), float(track[:, 2].max())],
    }


def inspect_pcd(path: Path) -> dict:
    header, arr = pcdio.read_pcd(path)
    xyzi = pcdio.to_xyzi(arr)
    return {
        "file": str(path),
        "header": {k: v for k, v in header.items()},
        "points": int(arr.size),
        "itemsize_bytes": int(arr.dtype.itemsize),
        "numpy_dtype": str(arr.dtype),
        "bounds": {k: list(v) for k, v in pcdio.bounds(xyzi).items()},
    }


def inspect_map(path: Path) -> dict:
    width, height = image_size(path)
    entry: dict = {"file": str(path), "width": width, "height": height}
    try:
        geo = geomap.MapGeo.from_filename(path, width=width, height=height)
    except ValueError as exc:
        entry["geo_error"] = str(exc)
        return entry
    x0, x1, y0, y1 = geo.extent()
    entry.update(
        resolution_m_per_px=geo.resolution,
        pixels_per_metre=geo.pixels_per_metre,
        origin_x_m=geo.origin_x,
        origin_y_m=geo.origin_y,
        resolution_consistent=geo.resolution_is_consistent(),
        extent_x_m=[x0, x1],
        extent_y_m=[y0, y1],
        description=geo.describe(),
    )
    return entry


def leap_second_check(bag_info: dict, pos_info: dict) -> dict:
    """Compare the bag's UTC span with the `.pos` GPST span."""
    bag_start = bag_info.get("start_epoch")
    if bag_start is None or not pos_info:
        return {"status": "skipped", "reason": "need both a bag and a .pos file"}

    pos_start_utc = pos_info["utc_start_epoch"]
    pos_start_gpst = pos_start_utc + timeutil.GPS_UTC_LEAP_SECONDS
    raw_offset = pos_start_gpst - bag_start
    corrected = pos_start_utc - bag_start
    overlap = (pos_info["utc_end_epoch"] >= bag_start and
               pos_start_utc <= bag_info["end_epoch"])
    ok = abs(corrected) <= LEAP_CHECK_TOLERANCE_S and overlap
    return {
        "status": "ok" if ok else "warning",
        "leap_seconds_applied": timeutil.GPS_UTC_LEAP_SECONDS,
        "pos_start_minus_bag_start_raw_gpst_s": round(raw_offset, 3),
        "pos_start_minus_bag_start_after_leap_correction_s": round(corrected, 3),
        "spans_overlap": bool(overlap),
        "tolerance_s": LEAP_CHECK_TOLERANCE_S,
    }


def nmea_utc_check(bag_info: dict) -> dict:
    """Compare the first NMEA GGA's UTC time-of-day with its bag log time."""
    nmea = bag_info.get("first_nmea")
    if not nmea:
        return {"status": "skipped", "reason": "no /nmea_sentence in bag"}
    fields = nmea["sentence"].split(",")
    if len(fields) < 2 or not fields[0].endswith("GGA"):
        return {"status": "skipped", "reason": "first sentence is not a GGA"}
    try:
        t_nmea = timeutil.nmea_utc_to_epoch(fields[1], nmea["bag_epoch"])
    except ValueError as exc:
        return {"status": "skipped", "reason": str(exc)}
    delta = t_nmea - nmea["bag_epoch"]
    return {
        "status": "ok" if abs(delta) <= LEAP_CHECK_TOLERANCE_S else "warning",
        "nmea_utc": timeutil.epoch_to_utc_string(t_nmea),
        "bag_utc": timeutil.epoch_to_utc_string(nmea["bag_epoch"]),
        "nmea_minus_bag_s": round(delta, 3),
        "note": "NMEA is UTC like the bag; a ~18 s gap here would mean GPST leakage",
    }


# --------------------------------------------------------------------------- #
def print_report(summary: dict) -> None:
    print(f"Dataset: {summary['dataset']}")
    print("\n--- files ---")
    for kind, files in summary["files"].items():
        for f in files:
            print(f"  {kind:6s} {f}")

    bag = summary.get("bag")
    if bag:
        print(f"\n--- bag ({len(bag['files'])} file(s)) ---")
        print(f"  span   {bag['start_utc']} .. {bag['end_utc']} UTC "
              f"({bag['duration_s']:.3f} s)")
        print(f"  msgs   {bag['message_count']}")
        print(f"  {'topic':28s} {'msgtype':40s} {'count':>8s} {'Hz':>8s}")
        for t in bag["topics"]:
            line = f"  {t['topic']:28s} {t['msgtype']:40s} {t['count']:>8d} {t['rate_hz']:>8.2f}"
            if "encoding" in t:
                line += f"   {t['encoding']} {t['width']}x{t['height']} step {t['step']}"
            print(line)
        if "first_nmea" in bag:
            print(f"  first NMEA: {bag['first_nmea']['sentence']}")

    for info in summary.get("pos", []):
        print(f"\n--- pos: {Path(info['file']).name} ---")
        print(f"  rows {info['rows']}  @ {info['rate_hz']} Hz  ({info['duration_s']} s)")
        print(f"  GPST {info['gpst_start']} .. {info['gpst_end']}")
        print(f"  UTC  {info['utc_start']} .. {info['utc_end']}")
        print(f"  Q histogram {info['q_histogram']}")
        print(f"  lat {info['lat_range']}  lon {info['lon_range']}  "
              f"h(ellipsoidal) {info['height_range_m']} m")

    for info in summary.get("kml", []):
        print(f"\n--- kml: {Path(info['file']).name} ---")
        print(f"  {info['points']} points  lat {info['lat_range']}  lon {info['lon_range']}")

    for info in summary.get("pcd", []):
        print(f"\n--- pcd: {Path(info['file']).name} ---")
        for k in ("VERSION", "FIELDS", "SIZE", "TYPE", "COUNT", "WIDTH", "HEIGHT",
                  "VIEWPOINT", "POINTS", "DATA"):
            if k in info["header"]:
                print(f"  {k:10s} {info['header'][k]}")
        print(f"  points {info['points']}  itemsize {info['itemsize_bytes']} B")
        for axis, rng in info["bounds"].items():
            print(f"  {axis:9s} [{rng[0]:.3f}, {rng[1]:.3f}]")

    for info in summary.get("map", []):
        print(f"\n--- map: {Path(info['file']).name} ---")
        if "geo_error" in info:
            print(f"  WARNING: {info['geo_error']}")
        else:
            print(f"  {info['description']}")
            print(f"  resolution == 1/px_per_m ? {info['resolution_consistent']}")

    print("\n--- time-base cross-checks ---")
    for name in ("leap_second_check", "nmea_utc_check"):
        check = summary.get(name, {})
        status = check.get("status", "skipped")
        marker = "OK  " if status == "ok" else ("WARN" if status == "warning" else "SKIP")
        print(f"  [{marker}] {name}")
        for k, v in check.items():
            if k != "status":
                print(f"         {k}: {v}")


# --------------------------------------------------------------------------- #
def main() -> int:
    p = argparse.ArgumentParser(
        description="Survey a MUN-FRL bell412 dataset directory (fast, index-driven).",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("dataset", help="Dataset directory (or a single .bag file).")
    p.add_argument("--out", default=str(DEFAULT_OUTPUT_ROOT),
                   help="Output directory for the JSON summary.")
    p.add_argument("--summary-name", default=SUMMARY_FILENAME,
                   help="File name of the JSON summary.")
    p.add_argument("--no-bag", action="store_true", help="Skip the bag inspection.")
    p.add_argument("--no-pcd", action="store_true",
                   help="Skip the PCD inspection (it reads the whole cloud).")
    args = p.parse_args()

    root = Path(args.dataset)
    if not root.exists():
        print(f"ERROR: '{root}' does not exist", file=sys.stderr)
        return 2
    if root.is_file():
        root = root.parent

    files = find_files(root)
    summary: dict = {
        "dataset": str(root),
        "files": {k: [str(p) for p in v] for k, v in files.items() if v},
    }

    if files["bag"] and not args.no_bag:
        summary["bag"] = inspect_bags(files["bag"])
    summary["pos"] = [inspect_pos(p) for p in files["pos"]]
    summary["kml"] = [inspect_kml(p) for p in files["kml"]]
    if not args.no_pcd:
        summary["pcd"] = [inspect_pcd(p) for p in files["pcd"]]
    summary["map"] = [inspect_map(p) for p in files["map"]]

    summary["leap_second_check"] = leap_second_check(
        summary.get("bag", {}), summary["pos"][0] if summary["pos"] else {}
    )
    summary["nmea_utc_check"] = nmea_utc_check(summary.get("bag", {}))

    print_report(summary)

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / args.summary_name
    with open(out_path, "w", encoding="utf-8") as fh:
        json.dump(summary, fh, indent=2, default=float)
    print(f"\nSummary written to {out_path}")

    warnings = [n for n in ("leap_second_check", "nmea_utc_check")
                if summary[n].get("status") == "warning"]
    if warnings:
        print(f"WARNING: cross-check(s) failed: {', '.join(warnings)}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
