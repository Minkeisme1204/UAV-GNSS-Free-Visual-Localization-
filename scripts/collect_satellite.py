#!/usr/bin/env python3
"""
collect_satellite.py - Build a georeferenced satellite/ortho basemap for a UAV
                       flight trajectory (MUN-FRL or any lat/lon track).

The extent/resolution can come either from CLI flags or from a dataset mission
YAML (``--config``), which also supplies the trajectory through its
``VideoReader.DroneTelemetry`` column mapping. Any CLI flag given alongside
``--config`` overrides the corresponding YAML value.

Supports three access patterns:
  1. xyz      - any XYZ tile provider via contextily (Esri World Imagery, etc.)
  2. mapserver- ArcGIS REST MapServer /export  (e.g. City of Ottawa 20cm ortho)
  3. imageserver - ArcGIS REST ImageServer /exportImage (e.g. Geospatial Ontario
                   GEO-IDS, returns raw 4-band RGB+NIR data)

Outputs a GeoTIFF in EPSG:3857 (or reprojected to local UTM), and optionally
an MBTiles file for a SQLite-backed tile store.

Dependencies:
    pip install contextily rasterio pyproj requests numpy
    pip install rio-mbtiles        # only if you want --mbtiles

Examples
--------
# YenBai 800 m: everything (track, extent, zoom, output) from the mission YAML.
# Prints the resolved track/bbox/zoom table and exits without downloading.
python3 scripts/collect_satellite.py \
    --config config/uavloc_yenbai800m.yaml --list-zooms

# Same config, but download it (SatelliteMap.zoom_level pins the pyramid level)
python3 scripts/collect_satellite.py --config config/uavloc_yenbai800m.yaml

# Ottawa (bell412) from the City of Ottawa 20 cm open ortho, target 0.4 m/px
python fetch_basemap.py \
    --ppk bell412_dataset1_ppk.pos \
    --source mapserver \
    --url https://maps.ottawa.ca/arcgis/rest/services/Basemap_Imagery_2022/MapServer \
    --target-gsd 0.4 --buffer 800 --utm --mbtiles \
    --out maps/ottawa_bell412_1

# Ottawa raw imagery incl. NIR from Geospatial Ontario
python fetch_basemap.py \
    --ppk bell412_dataset1_ppk.pos \
    --source imageserver \
    --url https://ws.geoservices.lrc.gov.on.ca/arcgis5/rest/services/AerialImagery/GEO_Imagery_Data_Service_2018to2022/ImageServer \
    --target-gsd 0.4 --buffer 800 --out maps/ottawa_geoids

# St. John's (quarry/lighthouse) fallback to Esri World Imagery, fixed zoom
python fetch_basemap.py \
    --ppk quarry2_ppk.pos \
    --source xyz --provider Esri.WorldImagery \
    --zoom 18 --buffer 500 --utm --out maps/stjohns_quarry2

# Inspect zoom/GSD options and the service's real cache levels, download nothing
python fetch_basemap.py \
    --ppk bell412_dataset1_ppk.pos --buffer 800 \
    --source mapserver \
    --url https://maps.ottawa.ca/arcgis/rest/services/Basemap_Imagery_2022/MapServer \
    --list-zooms --out /tmp/ignored

NOTE: the ArcGIS endpoints above were located from public service directories
but have NOT been exercised from this machine. Open the URL in a browser first
and confirm the extent covers your track and that `export`/`exportImage` is in
the Supported Operations list.
"""

from __future__ import annotations

import argparse
import io
import math
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import requests

# NOTE: rasterio / pyproj / contextily / cv2 / yaml are imported lazily inside
# the functions that need them, so the config, track and zoom logic stays
# usable (and testable) on a machine that only has the download stack missing.

WEBMERC_CIRC = 2 * math.pi * 6378137.0  # 40075016.686 m
TILE_PX = 256
# ArcGIS servers usually cap a single export request; stay well under.
MAX_REQ_PX = 2048
# Fallbacks used only when neither the CLI nor the config supplies a value.
DEFAULT_GSD = 0.4        # m/px
DEFAULT_BUFFER_M = 500.0
DEFAULT_PROVIDER = "Esri.WorldImagery"
DEFAULT_FORMAT = "png"


# --------------------------------------------------------------------------
# Trajectory reading
# --------------------------------------------------------------------------
def read_track(path: str) -> np.ndarray:
    """Read lat/lon from an RTKLIB .pos / PPK file or a generic CSV.

    RTKLIB .pos: whitespace-separated, comment lines start with '%',
                 columns are  date time lat lon height Q ns sdn sde sdu ...
    CSV:         must contain columns named lat/latitude and lon/longitude.

    Returns an (N, 2) array of [lat, lon].
    """
    p = Path(path)
    text = p.read_text(errors="replace").splitlines()

    # --- try RTKLIB .pos style first -------------------------------------
    rows = []
    for line in text:
        s = line.strip()
        if not s or s.startswith("%") or s.startswith("#"):
            continue
        parts = s.replace(",", " ").split()
        if len(parts) < 4:
            continue
        # look for two consecutive floats that look like lat/lon
        for i in range(len(parts) - 1):
            try:
                a, b = float(parts[i]), float(parts[i + 1])
            except ValueError:
                continue
            if -90.0 <= a <= 90.0 and -180.0 <= b <= 180.0 and abs(b) > 1e-6:
                rows.append((a, b))
                break
    if not rows:
        raise SystemExit(
            f"Could not parse lat/lon from {path}. "
            "Inspect the file header and adapt read_track()."
        )
    arr = np.asarray(rows, dtype=float)

    # Guard against a file where the parser latched onto a constant pair of
    # columns (e.g. the YenBai drone log's imgScale/digitalStab, both 1.0):
    # every row then decodes to the same point and the spread check below
    # cannot fire because the spread is exactly zero.
    if arr[:, 0].ptp() < 1e-6 and arr[:, 1].ptp() < 1e-6:
        raise SystemExit(
            f"parsed track collapses to a single point "
            f"(lat={arr[0, 0]:.6f}, lon={arr[0, 1]:.6f}): the column guess is "
            "wrong. Pass --lat-col/--lon-col, or use --config with a "
            "DroneTelemetry column mapping."
        )

    # Guard against a file where the parser latched onto the wrong pair:
    # a real flight track spans a small area, so reject absurd spreads.
    if arr[:, 0].ptp() > 2.0 or arr[:, 1].ptp() > 2.0:
        print(
            "WARNING: parsed track spans >2 degrees. The column guess is "
            "probably wrong - check read_track() against your file.",
            file=sys.stderr,
        )
    return arr


def _column_index(spec, header, what: str) -> int:
    """Resolve a column spec (header-name string or 0-based int) to an index."""
    if isinstance(spec, bool):
        raise SystemExit(f"{what} column spec {spec!r} is not a name or index")
    if isinstance(spec, int):
        if spec < 0:
            raise SystemExit(f"{what} column index {spec} is negative")
        return spec
    name = str(spec).strip()
    if name.lstrip("+-").isdigit():
        return int(name)
    if header is None:
        raise SystemExit(
            f"{what} column {name!r} is a header name but the file was "
            "declared as having no header row"
        )
    if name not in header:
        raise SystemExit(
            f"{what} column {name!r} not found in the CSV header "
            f"({len(header)} columns: {', '.join(header[:12])}...)"
        )
    return header.index(name)


def read_track_csv(path: str, lat_col, lon_col, id_col=None,
                   id_range=None, has_header: bool = True) -> np.ndarray:
    """Read lat/lon from a CSV using explicit column specs - no guessing.

    lat_col/lon_col/id_col accept a header-name string or a 0-based int index.
    Rows that fail to parse are skipped, as are rows whose fix is exactly zero
    (GPS not locked). When id_range=(lo, hi) is given, only rows with
    lo <= id <= hi are kept; either bound may be None for open-ended.

    Returns an (N, 2) array of [lat, lon].
    """
    import csv

    p = Path(path)
    if not p.is_file():
        raise SystemExit(f"track CSV not found: {path}")

    lo, hi = id_range if id_range is not None else (None, None)
    if (lo is not None or hi is not None) and id_col is None:
        raise SystemExit("an id range was requested but no id column was given")

    rows = []
    n_bad = 0
    n_nofix = 0
    with p.open(newline="", errors="replace") as fh:
        reader = csv.reader(fh)
        header = None
        if has_header:
            try:
                header = [h.strip() for h in next(reader)]
            except StopIteration:
                raise SystemExit(f"track CSV is empty: {path}")
        i_lat = _column_index(lat_col, header, "latitude")
        i_lon = _column_index(lon_col, header, "longitude")
        i_id = (_column_index(id_col, header, "frame id")
                if id_col is not None else None)

        for parts in reader:
            try:
                if i_id is not None:
                    fid = int(float(parts[i_id]))
                    if (lo is not None and fid < lo) or \
                       (hi is not None and fid > hi):
                        continue
                lat = float(parts[i_lat])
                lon = float(parts[i_lon])
            except (IndexError, ValueError):
                n_bad += 1
                continue
            if abs(lat) < 1e-9 or abs(lon) < 1e-9:
                n_nofix += 1
                continue
            rows.append((lat, lon))

    if n_bad or n_nofix:
        print(f"[track] skipped {n_bad} malformed row(s), "
              f"{n_nofix} row(s) without a GPS fix")
    if len(rows) < 2:
        raise SystemExit(
            f"only {len(rows)} usable fix(es) in {path} "
            f"(lat_col={lat_col!r}, lon_col={lon_col!r}, id_range={id_range}) "
            "- check the column mapping and the id range"
        )
    return np.asarray(rows, dtype=float)


def _video_frame_count(video_path) -> tuple:
    """Count the frames of a video, returning (n_frames, source_label).

    Probe order:
      1. ``ffprobe -count_packets`` - walks the container index (~140 ms) and
         reports the real number of coded video frames. NOT ``-count_frames``,
         which decodes the whole stream and takes minutes.
      2. ``cv2.CAP_PROP_FRAME_COUNT`` - reads the container header, which
         over-reports (17952 vs 17862 on cut_2025-07-30_800m-1x.mkv), so it is
         only a fallback for when ffprobe is missing or fails.

    Returns (0, "") when neither probe yields a positive count.
    """
    cmd = [
        "ffprobe", "-v", "error", "-select_streams", "v:0", "-count_packets",
        "-show_entries", "stream=nb_read_packets",
        "-of", "default=nw=1:nk=1", str(video_path),
    ]
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
        if proc.returncode != 0:
            why = (f"ffprobe exited {proc.returncode}: "
                   f"{proc.stderr.strip()[:160]}")
        else:
            text = proc.stdout.strip().splitlines()
            try:
                n = int(text[0]) if text else 0
            except ValueError:
                n = 0
            if n > 0:
                return n, "ffprobe"
            why = f"ffprobe returned {proc.stdout.strip()[:80]!r}"
    except FileNotFoundError:
        why = "ffprobe not found on PATH"
    except subprocess.TimeoutExpired:
        why = "ffprobe timed out after 60 s"
    except OSError as exc:  # noqa: BLE001
        why = f"ffprobe failed: {exc}"

    print(f"WARNING: falling back to cv2 CAP_PROP_FRAME_COUNT for "
          f"{str(video_path)!r} ({why})", file=sys.stderr)

    try:
        import cv2

        cap = cv2.VideoCapture(str(video_path))
        n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) if cap.isOpened() else 0
        cap.release()
    except Exception as exc:  # noqa: BLE001
        print(f"WARNING: could not probe {video_path}: {exc}", file=sys.stderr)
        return 0, ""
    if n > 0:
        return n, f"cv2 CAP_PROP_FRAME_COUNT, {why}"
    return 0, ""


def resolve_track_id_range(offset: int, start_frame: int, end_frame: int,
                           video_path) -> tuple:
    """Resolve `track_id_range: auto` to (lo, hi) absolute frame ids.

    lo = offset + start_frame,
    hi = offset + (end_frame if end_frame >= 0 else video_frame_count - 1).
    If the video cannot be probed, hi is None (open-ended: the whole CSV
    from lo on).
    """
    lo = int(offset) + int(start_frame)
    if end_frame is not None and int(end_frame) >= 0:
        return lo, int(offset) + int(end_frame)

    n_frames, src = (_video_frame_count(video_path) if video_path else (0, ""))
    if n_frames <= 0:
        print(f"WARNING: video frame count unavailable ({video_path!r}); "
              f"using an open-ended id range from {lo}", file=sys.stderr)
        return lo, None
    print(f"[track] frame count {n_frames} ({src})")
    return lo, int(offset) + n_frames - 1


# --------------------------------------------------------------------------
# Web Mercator / zoom helpers
# --------------------------------------------------------------------------
def gsd_at_zoom(lat: float, z: int) -> float:
    """Ground sample distance (m/px) of a Web Mercator tile pyramid."""
    return WEBMERC_CIRC * math.cos(math.radians(lat)) / (TILE_PX * 2 ** z)


def zoom_for_gsd(lat: float, target_gsd: float) -> int:
    """Smallest zoom whose GSD is <= target_gsd (i.e. at least as sharp)."""
    for z in range(1, 24):
        if gsd_at_zoom(lat, z) <= target_gsd:
            return z
    return 23


def print_zoom_table(lat: float, highlight: int | None = None,
                     zlo: int = 13, zhi: int = 22) -> None:
    print(f"\nWeb Mercator GSD at latitude {lat:.4f} deg:")
    print(f"   {'zoom':>5}  {'m/px':>9}   {'1 tile (256px)':>16}")
    print("   " + "-" * 36)
    for z in range(zlo, zhi + 1):
        g = gsd_at_zoom(lat, z)
        mark = " <--" if highlight == z else ""
        print(f"   {z:>5}  {g:>9.4f}   {g * TILE_PX:>13.1f} m{mark}")
    print()


def probe_arcgis_lods(url: str) -> None:
    """Print the tile-cache levels of detail an ArcGIS service actually has.

    A cached MapServer only serves the zoom levels baked into its cache, so
    asking for z21 on a cache that stops at z19 gives you nothing useful.
    Dynamic /export and /exportImage are unaffected by this.
    """
    try:
        r = requests.get(url.rstrip("/"), params={"f": "json"}, timeout=30)
        r.raise_for_status()
        meta = r.json()
    except Exception as exc:  # noqa: BLE001
        print(f"[probe] could not read service metadata: {exc}")
        return

    desc = meta.get("serviceDescription") or meta.get("description") or ""
    if desc:
        print(f"[probe] {desc.strip()[:200]}")
    if "pixelSizeX" in meta:
        print(f"[probe] native pixel size: {meta['pixelSizeX']} m")
    caps = meta.get("capabilities", "")
    if caps:
        print(f"[probe] capabilities: {caps}")

    tile_info = meta.get("tileInfo")
    if not tile_info:
        print("[probe] no tile cache (dynamic service) - any --zoom is fine, "
              "it only sets the requested pixel size")
        return

    lods = tile_info.get("lods", [])
    print(f"[probe] tile cache with {len(lods)} level(s) of detail:")
    for lod in lods:
        # ArcGIS LOD levels are numbered from the cache origin, which for a
        # Web Mercator cache normally coincides with the XYZ zoom level.
        print(f"    level {lod['level']:>3}  {lod['resolution']:>12.6f} m/px")
    if lods:
        print(f"[probe] usable zoom range: {lods[0]['level']}..{lods[-1]['level']}")


def utm_epsg(lat: float, lon: float) -> int:
    zone = int((lon + 180.0) // 6) + 1
    return (32600 if lat >= 0 else 32700) + zone


def bbox_from_track(track: np.ndarray, buffer_m: float) -> tuple:
    """Return (west, south, east, north) in lon/lat, padded by buffer_m."""
    lat0 = float(track[:, 0].mean())
    dlat = buffer_m / 111_320.0
    dlon = buffer_m / (111_320.0 * math.cos(math.radians(lat0)))
    return (
        float(track[:, 1].min()) - dlon,
        float(track[:, 0].min()) - dlat,
        float(track[:, 1].max()) + dlon,
        float(track[:, 0].max()) + dlat,
    )


def bbox_from_center(lat0: float, lon0: float,
                     width_m: float, height_m: float) -> tuple:
    """Return (west, south, east, north) in lon/lat for a ground-size extent."""
    dlat = (height_m / 2.0) / 111_320.0
    dlon = (width_m / 2.0) / (111_320.0 * math.cos(math.radians(lat0)))
    return (lon0 - dlon, lat0 - dlat, lon0 + dlon, lat0 + dlat)


def ll_to_3857(w, s, e, n) -> tuple:
    from pyproj import Transformer

    tf = Transformer.from_crs("EPSG:4326", "EPSG:3857", always_xy=True)
    xmin, ymin = tf.transform(w, s)
    xmax, ymax = tf.transform(e, n)
    return xmin, ymin, xmax, ymax


# --------------------------------------------------------------------------
# Mission YAML (--config)
# --------------------------------------------------------------------------
def _cfg_value(node: dict, key: str, default=None):
    """Read a YAML key, mapping null / empty string / empty list to default."""
    val = node.get(key, default)
    if val is None:
        return default
    if isinstance(val, str) and not val.strip():
        return default
    if isinstance(val, (list, tuple)) and not val:
        return default
    return val


def _resolve_path(value: str, root: Path) -> str:
    """Resolve a YAML path against the repo root when it is relative."""
    p = Path(str(value))
    return str(p if p.is_absolute() else (root / p))


def _telemetry_columns(node) -> dict:
    """Flatten the ordered list of single-key maps into {field: column}."""
    cols = {}
    if not node:
        return cols
    if isinstance(node, dict):
        return dict(node)
    for entry in node:
        if isinstance(entry, dict):
            cols.update(entry)
    return cols


def _parse_center(value):
    """`auto` -> None (track centroid); [lat, lon] -> (lat, lon)."""
    if value is None:
        return None
    if isinstance(value, str):
        if value.strip().lower() == "auto":
            return None
        raise SystemExit(f"SatelliteMap.center: expected 'auto' or [lat, lon], "
                         f"got {value!r}")
    seq = list(value)
    if len(seq) != 2:
        raise SystemExit(f"SatelliteMap.center: expected [lat, lon], got {value!r}")
    return float(seq[0]), float(seq[1])


def _parse_id_range(value):
    """`auto` -> the string 'auto'; [lo, hi] -> (lo, hi); missing -> 'auto'."""
    if value is None:
        return "auto"
    if isinstance(value, str):
        if value.strip().lower() == "auto":
            return "auto"
        raise SystemExit(f"SatelliteMap.track_id_range: expected 'auto' or "
                         f"[lo, hi], got {value!r}")
    seq = list(value)
    if len(seq) != 2:
        raise SystemExit(f"SatelliteMap.track_id_range: expected [lo, hi], "
                         f"got {value!r}")
    return int(seq[0]), int(seq[1])


def load_mission_config(path: str) -> dict:
    """Read a dataset mission YAML into the settings this script consumes.

    Relative paths in the YAML are resolved against the repository root, which
    is taken to be the config file's parent's parent (<root>/config/x.yaml).
    """
    import yaml

    cfg_path = Path(path).expanduser().resolve()
    if not cfg_path.is_file():
        raise SystemExit(f"config file not found: {path}")
    root = cfg_path.parent.parent

    doc = yaml.safe_load(cfg_path.read_text()) or {}
    sat = doc.get("SatelliteMap")
    if not isinstance(sat, dict):
        raise SystemExit(f"{path}: no 'SatelliteMap:' node - nothing to "
                         "configure the basemap collection with")

    vr = doc.get("VideoReader") or {}
    dt = vr.get("DroneTelemetry") or {}
    csv_path = str(_cfg_value(dt, "csv_path", "")).strip()
    if not csv_path:
        raise SystemExit(f"{path}: VideoReader.DroneTelemetry.csv_path is "
                         "empty - no trajectory to build the extent from")
    columns = _telemetry_columns(dt.get("columns"))
    for key in ("latitude_deg", "longitude_deg"):
        if key not in columns:
            raise SystemExit(f"{path}: VideoReader.DroneTelemetry.columns has "
                             f"no '{key}' entry - cannot read the track")

    video_path = str(_cfg_value(vr, "video_path", "")).strip()
    size_m = _cfg_value(sat, "size_m")
    if size_m is not None:
        seq = list(size_m)
        if len(seq) != 2:
            raise SystemExit(f"{path}: SatelliteMap.size_m expects "
                             f"[width_m, height_m], got {size_m!r}")
        size_m = (float(seq[0]), float(seq[1]))

    return {
        "config_path": str(cfg_path),
        "root": root,
        # ---- SatelliteMap
        "source": _cfg_value(sat, "source"),
        "provider": _cfg_value(sat, "provider"),
        "url": _cfg_value(sat, "url"),
        "zoom_level": _cfg_value(sat, "zoom_level"),
        "target_gsd": _cfg_value(sat, "target_gsd"),
        "size_m": size_m,
        "buffer_m": _cfg_value(sat, "buffer_m"),
        "center": _parse_center(_cfg_value(sat, "center")),
        "track_id_range": _parse_id_range(_cfg_value(sat, "track_id_range")),
        "out_prefix": (_resolve_path(_cfg_value(sat, "out_prefix"), root)
                       if _cfg_value(sat, "out_prefix") else None),
        "write_utm": bool(sat.get("write_utm", False)),
        "format": _cfg_value(sat, "format"),
        # ---- VideoReader / DroneTelemetry
        "csv_path": _resolve_path(csv_path, root),
        "has_header": bool(dt.get("has_header", True)),
        "columns": columns,
        "telemetry_frame_id_offset": int(
            _cfg_value(vr, "telemetry_frame_id_offset", 0)),
        "start_frame": int(_cfg_value(vr, "start_frame", 0)),
        "end_frame": int(_cfg_value(vr, "end_frame", -1)),
        "video_path": _resolve_path(video_path, root) if video_path else "",
    }


# --------------------------------------------------------------------------
# Source 1: XYZ tiles via contextily
# --------------------------------------------------------------------------
def fetch_xyz(bbox_ll, zoom, provider_name, out_tif):
    import contextily as cx

    prov = cx.providers
    for part in provider_name.split("."):
        prov = prov[part]

    w, s, e, n = bbox_ll
    print(f"[xyz] provider={provider_name} zoom={zoom}")
    cx.bounds2raster(w, s, e, n, out_tif, zoom=zoom, source=prov, ll=True)
    return out_tif


# --------------------------------------------------------------------------
# Sources 2 & 3: ArcGIS REST, chunked export + mosaic
# --------------------------------------------------------------------------
def _arcgis_chunk(url, op, xmin, ymin, xmax, ymax, w_px, h_px, fmt, session):
    params = {
        "bbox": f"{xmin},{ymin},{xmax},{ymax}",
        "bboxSR": "3857",
        "imageSR": "3857",
        "size": f"{w_px},{h_px}",
        "format": fmt,
        "f": "image",
    }
    if op == "export":
        params["transparent"] = "false"
    else:  # exportImage
        params["pixelType"] = "U8"
        params["noData"] = "0"
        params["interpolation"] = "RSP_BilinearInterpolation"

    r = session.get(f"{url.rstrip('/')}/{op}", params=params, timeout=120)
    r.raise_for_status()
    if b"error" in r.content[:200].lower() and len(r.content) < 2000:
        raise RuntimeError(f"server returned an error payload: {r.content[:300]!r}")
    return r.content


def fetch_arcgis(bbox_ll, gsd, url, op, out_tif, fmt="png"):
    """Download an ArcGIS REST MapServer/ImageServer extent as a GeoTIFF."""
    import rasterio
    from rasterio.merge import merge as rio_merge
    from rasterio.transform import from_bounds

    xmin, ymin, xmax, ymax = ll_to_3857(*bbox_ll)
    total_w = int(round((xmax - xmin) / gsd))
    total_h = int(round((ymax - ymin) / gsd))
    print(f"[arcgis:{op}] target {total_w} x {total_h} px @ {gsd:.3f} m/px")

    nx = math.ceil(total_w / MAX_REQ_PX)
    ny = math.ceil(total_h / MAX_REQ_PX)
    print(f"[arcgis:{op}] {nx * ny} request(s)")

    session = requests.Session()
    session.headers["User-Agent"] = "uavloc-basemap-fetch/1.0"

    tmp_dir = Path(out_tif).with_suffix("")
    tmp_dir.mkdir(parents=True, exist_ok=True)
    parts = []

    for iy in range(ny):
        for ix in range(nx):
            cw = min(MAX_REQ_PX, total_w - ix * MAX_REQ_PX)
            ch = min(MAX_REQ_PX, total_h - iy * MAX_REQ_PX)
            cxmin = xmin + ix * MAX_REQ_PX * gsd
            cxmax = cxmin + cw * gsd
            # image rows run north -> south
            cymax = ymax - iy * MAX_REQ_PX * gsd
            cymin = cymax - ch * gsd

            blob = _arcgis_chunk(
                url, op, cxmin, cymin, cxmax, cymax, cw, ch, fmt, session
            )
            with rasterio.open(io.BytesIO(blob)) as src:
                data = src.read()

            # The response carries no georeferencing, so attach it ourselves.
            part = tmp_dir / f"c_{iy:03d}_{ix:03d}.tif"
            with rasterio.open(
                part,
                "w",
                driver="GTiff",
                height=ch,
                width=cw,
                count=data.shape[0],
                dtype=data.dtype,
                crs="EPSG:3857",
                transform=from_bounds(cxmin, cymin, cxmax, cymax, cw, ch),
                compress="deflate",
            ) as dst:
                dst.write(data)
            parts.append(part)
            print(f"    chunk {len(parts)}/{nx * ny}", end="\r", flush=True)

    print()
    srcs = [rasterio.open(p) for p in parts]
    mosaic, transform = rio_merge(srcs)
    profile = srcs[0].profile
    profile.update(
        height=mosaic.shape[1],
        width=mosaic.shape[2],
        count=mosaic.shape[0],
        transform=transform,
        driver="GTiff",
        compress="deflate",
        tiled=True,
        blockxsize=512,
        blockysize=512,
    )
    with rasterio.open(out_tif, "w", **profile) as dst:
        dst.write(mosaic)
    for s in srcs:
        s.close()
    for p in parts:
        p.unlink()
    tmp_dir.rmdir()
    return out_tif


# --------------------------------------------------------------------------
# Post-processing
# --------------------------------------------------------------------------
def reproject_to_utm(in_tif, out_tif, lat, lon, gsd=None):
    """Reproject to local UTM.

    gsd is None (zoom-locked): keep whatever resolution the tile pyramid gave
    us - `calculate_default_transform` picks it. gsd set (gsd-locked): force
    that pixel size and snap the origin onto the gsd grid, so several maps
    fetched at the same target GSD share one pixel lattice.
    """
    import rasterio
    from rasterio.transform import Affine
    from rasterio.warp import calculate_default_transform, reproject, Resampling

    epsg = utm_epsg(lat, lon)
    dst_crs = f"EPSG:{epsg}"
    print(f"[utm] reprojecting to {dst_crs}")
    with rasterio.open(in_tif) as src:
        if gsd is None:
            transform, width, height = calculate_default_transform(
                src.crs, dst_crs, src.width, src.height, *src.bounds
            )
            print("[utm] zoom-locked -> native pyramid resolution, "
                  "no grid snap")
        else:
            transform, width, height = calculate_default_transform(
                src.crs, dst_crs, src.width, src.height, *src.bounds,
                resolution=(gsd, gsd),
            )
            # Snap the top-left origin outward onto the gsd grid; grow the
            # raster by one pixel on each snapped axis so nothing is cropped.
            c_snap = math.floor(transform.c / gsd) * gsd
            f_snap = math.ceil(transform.f / gsd) * gsd
            if abs(c_snap - transform.c) > 1e-9:
                width += 1
            if abs(f_snap - transform.f) > 1e-9:
                height += 1
            transform = Affine(transform.a, transform.b, c_snap,
                               transform.d, transform.e, f_snap)
            print(f"[utm] gsd-locked -> resolution=({gsd}, {gsd}), "
                  "origin snapped to grid")
        profile = src.profile.copy()
        profile.update(
            crs=dst_crs,
            transform=transform,
            width=width,
            height=height,
            driver="GTiff",
            compress="deflate",
            tiled=True,
            blockxsize=512,
            blockysize=512,
        )
        with rasterio.open(out_tif, "w", **profile) as dst:
            for b in range(1, src.count + 1):
                reproject(
                    source=rasterio.band(src, b),
                    destination=rasterio.band(dst, b),
                    src_transform=src.transform,
                    src_crs=src.crs,
                    dst_transform=transform,
                    dst_crs=dst_crs,
                    resampling=Resampling.bilinear,
                )
    return out_tif


def to_mbtiles(in_tif, out_mbt, zmin, zmax):
    """Export to MBTiles via rio-mbtiles (falls back to gdal_translate)."""
    print(f"[mbtiles] {out_mbt}  z{zmin}..{zmax}")
    cmd = [
        "rio", "mbtiles", in_tif, "-o", out_mbt,
        "--zoom-levels", f"{zmin}..{zmax}",
        "--format", "PNG", "-j", str(os.cpu_count() or 4),
    ]
    try:
        subprocess.run(cmd, check=True)
        return out_mbt
    except (FileNotFoundError, subprocess.CalledProcessError) as exc:
        print(f"[mbtiles] rio mbtiles unavailable/failed ({exc}); trying gdal")
    subprocess.run(
        ["gdal_translate", "-of", "MBTILES", in_tif, out_mbt], check=True
    )
    subprocess.run(
        ["gdaladdo", "-r", "average", out_mbt] +
        [str(2 ** k) for k in range(1, zmax - zmin + 1)],
        check=True,
    )
    return out_mbt


# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(
        description="Fetch a georeferenced basemap for a UAV trajectory."
    )
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--ppk", help="PPK/.pos/CSV file containing lat/lon")
    src.add_argument(
        "--bbox", nargs=4, type=float,
        metavar=("W", "S", "E", "N"), help="explicit lon/lat bbox",
    )
    src.add_argument(
        "--config",
        help="dataset mission YAML (SatelliteMap: + VideoReader.DroneTelemetry). "
             "Any CLI flag given as well overrides the YAML value.",
    )

    ap.add_argument(
        "--source", default=None,
        choices=["xyz", "mapserver", "imageserver"],
        help="required unless it comes from --config (SatelliteMap.source)",
    )
    ap.add_argument("--url", help="ArcGIS REST service root (non-xyz sources)")
    ap.add_argument(
        "--provider", default=None,
        help=f"xyzservices provider path (default {DEFAULT_PROVIDER})",
    )
    res = ap.add_mutually_exclusive_group()
    res.add_argument(
        "--zoom", type=int,
        help="Web Mercator zoom level to fetch at (e.g. 18). "
             "Takes precedence over --target-gsd and over the config.",
    )
    res.add_argument(
        "--target-gsd", type=float,
        help="desired map resolution in m/px; the nearest zoom that is at "
             f"least this sharp is chosen (default {DEFAULT_GSD} if neither "
             "flag nor config gives one)",
    )
    ap.add_argument(
        "--list-zooms", action="store_true",
        help="print the zoom/GSD table (and the service's cache levels if "
             "--url is given), then exit without downloading",
    )
    ap.add_argument(
        "--buffer", type=float, default=None,
        help=f"padding around the track in metres (default {DEFAULT_BUFFER_M})",
    )
    ap.add_argument("--out", default=None, help="output path prefix")
    ap.add_argument("--utm", action="store_true", help="also write a UTM copy")
    ap.add_argument("--mbtiles", action="store_true", help="also write MBTiles")
    ap.add_argument(
        "--format", default=None, choices=["png", "png32", "jpg", "tiff"],
        help=f"ArcGIS response format (default {DEFAULT_FORMAT})",
    )
    # ---- explicit CSV columns (bare CSV, no YAML) ----------------------
    ap.add_argument("--lat-col", help="latitude column: header name or index")
    ap.add_argument("--lon-col", help="longitude column: header name or index")
    ap.add_argument("--id-col", help="frame-id column: header name or index")
    ap.add_argument(
        "--id-range", nargs=2, type=int, metavar=("LO", "HI"),
        help="keep only rows whose frame id is in [LO, HI] (needs --id-col)",
    )
    args = ap.parse_args()

    cfg = load_mission_config(args.config) if args.config else None
    if cfg is not None:
        print(f"[config] {cfg['config_path']}")

    def pick(cli_value, key, fallback=None):
        """CLI flag > config value > built-in fallback."""
        if cli_value is not None:
            return cli_value
        if cfg is not None and cfg.get(key) is not None:
            return cfg[key]
        return fallback

    source = pick(args.source, "source")
    if source is None:
        ap.error("--source is required (or set SatelliteMap.source in --config)")
    if source not in ("xyz", "mapserver", "imageserver"):
        ap.error(f"unknown source {source!r} (xyz | mapserver | imageserver)")
    url = pick(args.url, "url")
    provider = pick(args.provider, "provider", DEFAULT_PROVIDER)
    fmt = pick(args.format, "format", DEFAULT_FORMAT)
    out = pick(args.out, "out_prefix")
    write_utm = args.utm or bool(cfg is not None and cfg.get("write_utm"))

    if source != "xyz" and not url:
        ap.error("--url is required for mapserver/imageserver sources")
    if out is None and not args.list_zooms:
        ap.error("--out is required (or set SatelliteMap.out_prefix in --config)")

    # ---- trajectory ----------------------------------------------------
    track = None
    if args.bbox:
        pass
    elif cfg is not None:
        cols = cfg["columns"]
        lat_col = args.lat_col if args.lat_col is not None else cols["latitude_deg"]
        lon_col = args.lon_col if args.lon_col is not None else cols["longitude_deg"]
        id_col = args.id_col if args.id_col is not None else cols.get("frame_id")

        if args.id_range is not None:
            id_range = tuple(args.id_range)
        elif cfg["track_id_range"] == "auto":
            id_range = resolve_track_id_range(
                cfg["telemetry_frame_id_offset"], cfg["start_frame"],
                cfg["end_frame"], cfg["video_path"],
            )
        else:
            id_range = cfg["track_id_range"]
        if id_col is None and id_range is not None:
            print("WARNING: no frame_id column in DroneTelemetry.columns; "
                  "using the whole CSV instead of the id range",
                  file=sys.stderr)
            id_range = None

        track = read_track_csv(cfg["csv_path"], lat_col, lon_col, id_col,
                               id_range, cfg["has_header"])
        if id_range is not None:
            lo, hi = id_range
            print(f"[track] imageId {lo}..{hi if hi is not None else 'end'} "
                  f"-> {len(track)} fixes")
    elif args.lat_col is not None and args.lon_col is not None:
        track = read_track_csv(
            args.ppk, args.lat_col, args.lon_col, args.id_col,
            tuple(args.id_range) if args.id_range else None,
        )
    else:
        if args.id_range is not None or args.id_col is not None:
            ap.error("--id-col/--id-range require --lat-col and --lon-col")
        track = read_track(args.ppk)

    # ---- extent --------------------------------------------------------
    if track is not None:
        centre = cfg["center"] if cfg is not None else None
        if centre is None:
            centre = (float(track[:, 0].mean()), float(track[:, 1].mean()))
        lat0, lon0 = centre
        print(f"[track] {len(track)} fixes, centre {lat0:.5f}, {lon0:.5f}")

        size_m = cfg["size_m"] if cfg is not None else None
        if size_m is not None:
            bbox_ll = bbox_from_center(lat0, lon0, size_m[0], size_m[1])
            print(f"[bbox] size_m {size_m[0]:.0f} x {size_m[1]:.0f} m about "
                  f"({lat0:.6f}, {lon0:.6f})")
        else:
            buffer_m = float(pick(args.buffer, "buffer_m", DEFAULT_BUFFER_M))
            bbox_ll = bbox_from_track(track, buffer_m)
            print(f"[bbox] track bbox + buffer {buffer_m:.0f} m")
    else:
        bbox_ll = tuple(args.bbox)
        lat0 = (bbox_ll[1] + bbox_ll[3]) / 2.0
        lon0 = (bbox_ll[0] + bbox_ll[2]) / 2.0

    w, s, e, n = bbox_ll
    span_x = (e - w) * 111_320.0 * math.cos(math.radians(lat0))
    span_y = (n - s) * 111_320.0
    print(f"[bbox] {w:.6f} {s:.6f} {e:.6f} {n:.6f}")
    print(f"[bbox] ~{span_x:.0f} x {span_y:.0f} m")

    # ---- resolve zoom --------------------------------------------------
    # Precedence: --zoom > SatelliteMap.zoom_level > --target-gsd >
    #             SatelliteMap.target_gsd > DEFAULT_GSD.
    # The two zoom cases lock the pyramid level: target_gsd is ignored and the
    # UTM reprojection later keeps the native resolution (no grid snap).
    cfg_zoom = cfg["zoom_level"] if cfg is not None else None
    if args.zoom is not None or cfg_zoom is not None:
        zoom_locked = True
        if args.zoom is not None:
            zoom, origin = int(args.zoom), "--zoom"
        else:
            zoom, origin = int(cfg_zoom), "config"
        actual = gsd_at_zoom(lat0, zoom)
        print(f"[gsd] zoom_level={zoom} ({origin}) -> {actual:.4f} m/px at "
              f"lat {lat0:.4f} - zoom-locked, target_gsd ignored")
    else:
        zoom_locked = False
        if args.target_gsd is not None:
            target, origin = float(args.target_gsd), "--target-gsd"
        elif cfg is not None and cfg["target_gsd"] is not None:
            target, origin = float(cfg["target_gsd"]), "config"
        else:
            target, origin = DEFAULT_GSD, "default"
        zoom = zoom_for_gsd(lat0, target)
        actual = gsd_at_zoom(lat0, zoom)
        print(f"[gsd] target_gsd={target:.3f} m/px ({origin}) -> z={zoom} "
              f"({actual:.4f} m/px) - gsd-locked")

    print_zoom_table(lat0, highlight=zoom)

    px_w = int(round(span_x / actual))
    px_h = int(round(span_y / actual))
    est_mb = px_w * px_h * 3 / 1e6
    print(f"[size] full map ~{px_w} x {px_h} px "
          f"(~{est_mb:.0f} MB uncompressed RGB)")
    if source == "xyz":
        n_tiles = math.ceil(px_w / TILE_PX) * math.ceil(px_h / TILE_PX)
        print(f"[size] ~{n_tiles} tile request(s) at z={zoom}")

    if args.list_zooms:
        if url:
            print()
            probe_arcgis_lods(url)
        return

    out_prefix = Path(out)
    out_prefix.parent.mkdir(parents=True, exist_ok=True)
    merc_tif = str(out_prefix) + "_3857.tif"

    # ---- fetch ---------------------------------------------------------
    if source == "xyz":
        fetch_xyz(bbox_ll, zoom, provider, merc_tif)
    else:
        op = "export" if source == "mapserver" else "exportImage"
        fetch_arcgis(bbox_ll, actual, url, op, merc_tif, fmt=fmt)

    import rasterio

    with rasterio.open(merc_tif) as ds:
        print(f"[done] {merc_tif}  {ds.width} x {ds.height} x {ds.count} "
              f"{ds.dtypes[0]}  crs={ds.crs}")

    if write_utm:
        reproject_to_utm(merc_tif, str(out_prefix) + "_utm.tif", lat0, lon0,
                         gsd=None if zoom_locked else actual)

    if args.mbtiles:
        to_mbtiles(merc_tif, str(out_prefix) + ".mbtiles",
                   max(zoom - 4, 1), zoom)


if __name__ == "__main__":
    main()