#!/usr/bin/env python3
"""
stitch_tiles.py - Mosaic an already-downloaded XYZ tile pyramid on disk into
                  georeferenced GeoTIFFs, one pair of files per zoom level.

Input is the standard slippy-map layout ``<tiles>/<z>/<x>/<y>.png`` (256x256
Web Mercator tiles). Nothing is downloaded: use ``collect_satellite.py`` for
that. This script only reads the tiles, so it is safe to point ``--tiles`` at a
read-only archive.

For every requested zoom it writes

    <out>/<name>_z<zz>_3857.tif   EPSG:3857, the pyramid's native resolution
    <out>/<name>_z<zz>_utm.tif    reprojected to UTM (bilinear), metres

Both are tiled (512x512 blocks), DEFLATE+predictor=2 compressed, 3-band RGB
(the alpha channel of an RGBA tile is dropped), and carry overviews
[2, 4, 8, 16] built with `average`. ``--no-overviews`` writes the full-resolution
image only - a single image per file, roughly a third smaller on disk, at the
cost of slow zoomed-out redraws in a GIS. Overviews can always be added later
with ``gdaladdo -r average <file> 2 4 8 16``.

Georeferencing is computed from the tile indices, not estimated:

    merc_x(x, z) = x / 2**z * 2*ORIGIN - ORIGIN
    merc_y(y, z) = ORIGIN - y / 2**z * 2*ORIGIN
    pixel size   = 2*ORIGIN / (2**z * 256)          [Web Mercator metres]

with ORIGIN = 20037508.342789244. The mosaic's top-left corner is the top-left
corner of tile (x_min, y_min). Note that a Web Mercator metre is not a ground
metre: the ground sample distance is ``pixel size * cos(latitude)``, which is
what the UTM copy carries.

The mosaic is written one tile-row (256 px tall) at a time, so a 23808x15616
z21 map never exists in RAM as a whole.

Two guards keep the files from bloating to several times their pixel content:

* The UTM reprojection warps **all bands in a single pass**. Warping one band
  at a time rewrites each destination block once per band, and a recompressed
  block of a pixel-interleaved DEFLATE GeoTIFF does not fit its old slot, so
  GDAL appends it and orphans the old copy. Measured: bell412_dataset3 z21
  4.32 GB / 58.4 % dead (2026-08-13), YenBai500m z19 7.71 GB / 56.6 % dead
  even with an 8 GB cache (2026-08-14).
* ``--gdal-cachemax-mb`` (default 4096) raises GDAL's block cache for the whole
  writing phase. It is a second line of defence only: it works while source and
  destination both fit in RAM, which stops being true past a few gigapixels.

Every file written is then audited - the sum of its ``TileByteCounts`` against
its size on disk - and anything over ``DEAD_SPACE_WARN_FRACTION`` is reported
loudly instead of being left for the next person to discover.

Provenance is written into the GeoTIFF tags (source directory, zoom, tile
count, GSD, creation date). The imagery *provider* is NOT knowable from a bare
tile tree, so the ``SOURCE_PROVENANCE`` tag is filled with a placeholder unless
``--provenance`` is given - fill it in before citing the map anywhere.

Dependencies:
    pip install rasterio pillow numpy pyproj

Examples
--------
# What would be produced - grid, pixel size, bbox, disk estimate. Writes nothing.
python3 scripts/stitch_tiles.py \
    --tiles "/media/ssd/Thesis Satellite Maps/Mun DS6" \
    --zooms 19 20 21 --out data/munfrl_bell412_dataset6/map \
    --name bell412_dataset6 --dry-run

# The real run (UTM zone pinned; without --utm-epsg it is derived from the
# mosaic's centre longitude)
python3 scripts/stitch_tiles.py \
    --tiles "/media/ssd/Thesis Satellite Maps/Mun DS6" \
    --zooms 19 20 21 --out data/munfrl_bell412_dataset6/map \
    --name bell412_dataset6 --utm-epsg 32618
"""

from __future__ import annotations

import argparse
import datetime
import math
import struct
import sys
from pathlib import Path

import numpy as np

# rasterio / PIL / pyproj are imported lazily inside the functions that need
# them so that --help and the grid scan stay usable without the write stack.

ORIGIN = 20037508.342789244   # half the Web Mercator extent [m]
TILE_PX = 256
BAND_COUNT = 3               # RGB; tile alpha is dropped
OVERVIEW_FACTORS = [2, 4, 8, 16]
BLOCK_PX = 512
TILE_EXTS = (".png", ".jpg", ".jpeg", ".webp")

# GDAL block cache used while writing. A whole block-row of the largest map we
# stitch must fit, or blocks get recompressed and re-appended between band
# passes (see the module docstring: 58.4 % dead bytes at the default cache).
# 4096 MB holds ~5400 512x512x3 blocks, i.e. a block-row of a 2.7 Mpx-wide
# mosaic, with room for the reprojection's source window on top.
DEFAULT_GDAL_CACHEMAX_MB = 4096
# Fraction of a written file that may be orphaned before it is worth a warning.
DEAD_SPACE_WARN_FRACTION = 0.10

PROVENANCE_PLACEHOLDER = (
    "UNKNOWN - not recoverable from the tile tree; fill in imagery provider, "
    "capture date and licence before citing this map"
)


# --------------------------------------------------------------------------
# Web Mercator helpers
# --------------------------------------------------------------------------
def merc_pixel_size(z: int) -> float:
    """Web Mercator metres per pixel of a 256 px tile pyramid at zoom z."""
    return 2.0 * ORIGIN / (2 ** z * TILE_PX)


def merc_x(x: float, z: int) -> float:
    """Left edge of tile column x, in Web Mercator metres."""
    return x / 2 ** z * 2.0 * ORIGIN - ORIGIN


def merc_y(y: float, z: int) -> float:
    """Top edge of tile row y, in Web Mercator metres."""
    return ORIGIN - y / 2 ** z * 2.0 * ORIGIN


def merc_to_lonlat(mx: float, my: float) -> tuple:
    """Inverse Web Mercator, without pulling in pyproj."""
    lon = mx / ORIGIN * 180.0
    lat = math.degrees(
        2.0 * math.atan(math.exp(my / ORIGIN * math.pi)) - math.pi / 2.0
    )
    return lon, lat


def utm_epsg(lat: float, lon: float) -> int:
    zone = int((lon + 180.0) // 6) + 1
    return (32600 if lat >= 0 else 32700) + zone


# --------------------------------------------------------------------------
# Tile-tree scanning
# --------------------------------------------------------------------------
class Grid:
    """The rectangular x/y span of one zoom level plus what is actually there."""

    def __init__(self, z: int, tiles: dict):
        self.z = z
        self.tiles = tiles                      # {(x, y): Path}
        xs = sorted({x for x, _ in tiles})
        ys = sorted({y for _, y in tiles})
        self.x_min, self.x_max = xs[0], xs[-1]
        self.y_min, self.y_max = ys[0], ys[-1]
        self.nx = self.x_max - self.x_min + 1
        self.ny = self.y_max - self.y_min + 1

    @property
    def width(self) -> int:
        return self.nx * TILE_PX

    @property
    def height(self) -> int:
        return self.ny * TILE_PX

    @property
    def n_present(self) -> int:
        return len(self.tiles)

    @property
    def n_expected(self) -> int:
        return self.nx * self.ny

    def missing(self) -> list:
        """Grid cells inside the bounding rectangle that have no tile file."""
        out = []
        for y in range(self.y_min, self.y_max + 1):
            for x in range(self.x_min, self.x_max + 1):
                if (x, y) not in self.tiles:
                    out.append((x, y))
        return out

    def bounds_3857(self) -> tuple:
        """(left, bottom, right, top) of the mosaic in EPSG:3857."""
        px = merc_pixel_size(self.z)
        left = merc_x(self.x_min, self.z)
        top = merc_y(self.y_min, self.z)
        return left, top - self.height * px, left + self.width * px, top

    def centre_lonlat(self) -> tuple:
        left, bottom, right, top = self.bounds_3857()
        return merc_to_lonlat((left + right) / 2.0, (bottom + top) / 2.0)

    def gsd(self) -> float:
        """Ground sample distance [m/px] at the mosaic's centre latitude."""
        _, lat = self.centre_lonlat()
        return merc_pixel_size(self.z) * math.cos(math.radians(lat))


def scan_zoom(tiles_root: Path, z: int) -> Grid:
    """Find every ``<tiles_root>/<z>/<x>/<y>.<ext>`` tile of one zoom level."""
    zdir = tiles_root / str(z)
    if not zdir.is_dir():
        raise FileNotFoundError(f"no zoom directory {zdir}")

    tiles = {}
    n_odd = 0
    for xdir in zdir.iterdir():
        if not xdir.is_dir() or not xdir.name.lstrip("-").isdigit():
            n_odd += 1
            continue
        x = int(xdir.name)
        for f in xdir.iterdir():
            if f.suffix.lower() not in TILE_EXTS:
                n_odd += 1
                continue
            stem = f.stem
            if not stem.lstrip("-").isdigit():
                n_odd += 1
                continue
            tiles[(x, int(stem))] = f
    if not tiles:
        raise FileNotFoundError(f"{zdir} contains no <x>/<y> tile files")
    if n_odd:
        print(f"[scan] z{z}: ignored {n_odd} entr(y|ies) that are not "
              "<x>/<y>.<ext> tiles")
    return Grid(z, tiles)


# --------------------------------------------------------------------------
# Reporting
# --------------------------------------------------------------------------
def report_grid(g: Grid) -> list:
    """Print one zoom's geometry; return its missing-cell list."""
    left, bottom, right, top = g.bounds_3857()
    px = merc_pixel_size(g.z)
    lon_w, lat_s = merc_to_lonlat(left, bottom)
    lon_e, lat_n = merc_to_lonlat(right, top)
    gsd = g.gsd()
    raw_mb = g.width * g.height * BAND_COUNT / 1e6

    print(f"\n[z{g.z}] grid x {g.x_min}..{g.x_max} ({g.nx}), "
          f"y {g.y_min}..{g.y_max} ({g.ny})")
    print(f"[z{g.z}] tiles present {g.n_present} / {g.n_expected} expected")
    print(f"[z{g.z}] mosaic {g.width} x {g.height} px  "
          f"({raw_mb:.0f} MB uncompressed RGB)")
    print(f"[z{g.z}] pixel {px:.6f} merc-m  ->  GSD {gsd:.4f} m/px  "
          f"(ground {g.width * gsd:.0f} x {g.height * gsd:.0f} m)")
    print(f"[z{g.z}] bbox 3857  {left:.2f} {bottom:.2f} {right:.2f} {top:.2f}")
    print(f"[z{g.z}] bbox lonlat  W {lon_w:.6f}  S {lat_s:.6f}  "
          f"E {lon_e:.6f}  N {lat_n:.6f}")

    missing = g.missing()
    if missing:
        head = ", ".join(f"({x},{y})" for x, y in missing[:10])
        more = "" if len(missing) <= 10 else f", ... (+{len(missing) - 10} more)"
        print(f"[z{g.z}] WARNING: {len(missing)} tile(s) missing inside the "
              f"bounding rectangle -> those pixels will be black: {head}{more}",
              file=sys.stderr)
    return missing


# --------------------------------------------------------------------------
# Dead-space audit (TIFF / BigTIFF, no extra dependency)
# --------------------------------------------------------------------------
TIFF_TAG_STRIP_BYTE_COUNTS = 279
TIFF_TAG_TILE_BYTE_COUNTS = 325
TIFF_TAG_SUB_IFDS = 330
# TIFF field type -> struct format, for the types byte-count tags can use.
TIFF_TYPE_FMT = {1: "B", 3: "H", 4: "I", 16: "Q"}


def _tiff_read_values(fh, endian: str, fmt: str, count: int,
                      raw: bytes, entry_value_size: int, offset: int) -> list:
    """Values of one IFD entry, inline if they fit, else read from `offset`."""
    size = struct.calcsize(fmt) * count
    if size <= entry_value_size:
        buf = raw[:size]
    else:
        fh.seek(offset)
        buf = fh.read(size)
    if len(buf) < size:
        raise ValueError("truncated IFD value block")
    return list(struct.unpack(f"{endian}{count}{fmt}", buf))


def tiff_live_bytes(path: Path) -> int:
    """Sum of every Strip/TileByteCounts entry in every IFD of a TIFF.

    That is the number of bytes in the file that a reader will ever touch for
    pixel data. Anything beyond it (minus the few kB of header/IFD/tag
    overhead) is orphaned. Handles classic TIFF and BigTIFF, chained IFDs
    (GDAL's overviews) and SubIFDs. Raises on anything it does not recognise -
    the caller degrades to "not measured" rather than to a wrong number.
    """
    with open(path, "rb") as fh:
        head = fh.read(16)
        if head[:2] == b"II":
            endian = "<"
        elif head[:2] == b"MM":
            endian = ">"
        else:
            raise ValueError(f"{path} is not a TIFF (bad byte-order mark)")
        version = struct.unpack(f"{endian}H", head[2:4])[0]
        # entry layout: tag u2 | type u2 | count | value-or-offset
        #   classic TIFF: count u4, value 4 B, entry 12 B, IFD entry count u2
        #   BigTIFF:      count u8, value 8 B, entry 20 B, IFD entry count u8
        if version == 42:
            off_fmt, entry_size, entry_value_size = "I", 12, 4
            n_entries_fmt, n_entries_size = "H", 2
            tag_count_fmt, tag_count_size = "I", 4
            next_off = struct.unpack(f"{endian}I", head[4:8])[0]
        elif version == 43:
            off_size, pad = struct.unpack(f"{endian}HH", head[4:8])
            if off_size != 8 or pad != 0:
                raise ValueError(f"{path}: unsupported BigTIFF header")
            off_fmt, entry_size, entry_value_size = "Q", 20, 8
            n_entries_fmt, n_entries_size = "Q", 8
            tag_count_fmt, tag_count_size = "Q", 8
            next_off = struct.unpack(f"{endian}Q", head[8:16])[0]
        else:
            raise ValueError(f"{path}: unknown TIFF version {version}")

        total = 0
        pending = [next_off]
        seen = set()
        while pending:
            ifd = pending.pop()
            if ifd == 0 or ifd in seen:
                continue
            seen.add(ifd)
            fh.seek(ifd)
            n_entries = struct.unpack(f"{endian}{n_entries_fmt}",
                                      fh.read(n_entries_size))[0]
            entries = fh.read(entry_size * n_entries)
            if len(entries) < entry_size * n_entries:
                raise ValueError(f"{path}: truncated IFD at {ifd}")
            pending.append(struct.unpack(f"{endian}{off_fmt}",
                                         fh.read(entry_value_size))[0])
            for i in range(n_entries):
                e = entries[i * entry_size:(i + 1) * entry_size]
                tag, ftype = struct.unpack(f"{endian}HH", e[:4])
                if tag not in (TIFF_TAG_STRIP_BYTE_COUNTS,
                               TIFF_TAG_TILE_BYTE_COUNTS, TIFF_TAG_SUB_IFDS):
                    continue
                count = struct.unpack(f"{endian}{tag_count_fmt}",
                                      e[4:4 + tag_count_size])[0]
                raw = e[4 + tag_count_size:]
                fmt = TIFF_TYPE_FMT.get(ftype)
                if fmt is None:
                    raise ValueError(f"{path}: tag {tag} has field type {ftype}")
                offset = struct.unpack(f"{endian}{off_fmt}",
                                       raw[:entry_value_size])[0]
                pos = fh.tell()
                vals = _tiff_read_values(fh, endian, fmt, count, raw,
                                         entry_value_size, offset)
                fh.seek(pos)
                if tag == TIFF_TAG_SUB_IFDS:
                    pending.extend(vals)
                else:
                    total += sum(vals)
        return total


def report_dead_space(path: Path, label: str) -> None:
    """Compare pixel bytes against file size; shout if too much is orphaned."""
    file_bytes = path.stat().st_size
    try:
        live = tiff_live_bytes(path)
    except Exception as exc:                              # noqa: BLE001
        print(f"{label} dead-space check skipped ({exc})", file=sys.stderr)
        return
    dead = file_bytes - live
    frac = dead / file_bytes if file_bytes else 0.0
    msg = (f"{label} {file_bytes} B on disk, {live} B live pixel data, "
           f"{dead} B dead ({frac * 100:.1f} %)")
    if frac > DEAD_SPACE_WARN_FRACTION:
        print(f"{msg}\n{label} WARNING: more than "
              f"{DEAD_SPACE_WARN_FRACTION * 100:.0f} % of this file is orphaned "
              "- raise --gdal-cachemax-mb and rewrite it, or repack with "
              "'gdal_translate -co TILED=YES -co BLOCKXSIZE=512 "
              "-co BLOCKYSIZE=512 -co COMPRESS=DEFLATE -co PREDICTOR=2 "
              "-co BIGTIFF=YES <in> <out>'", file=sys.stderr)
    else:
        print(msg)


# --------------------------------------------------------------------------
# Stitching
# --------------------------------------------------------------------------
def _read_tile(path: Path) -> np.ndarray:
    """Load one tile as (3, TILE_PX, TILE_PX) uint8, dropping alpha."""
    from PIL import Image

    with Image.open(path) as im:
        arr = np.asarray(im.convert("RGB"), dtype=np.uint8)
    if arr.shape[0] != TILE_PX or arr.shape[1] != TILE_PX:
        raise ValueError(
            f"{path} is {arr.shape[1]}x{arr.shape[0]} px, expected "
            f"{TILE_PX}x{TILE_PX}"
        )
    return np.transpose(arr, (2, 0, 1))


def stitch_zoom(g: Grid, out_tif: Path, tiles_root: Path,
                provenance: str, missing: list,
                overviews: bool = True) -> Path:
    """Write the EPSG:3857 mosaic of one zoom level, one tile-row at a time."""
    import rasterio
    from rasterio.enums import Resampling
    from rasterio.transform import Affine
    from rasterio.windows import Window

    px = merc_pixel_size(g.z)
    left, _, _, top = g.bounds_3857()
    transform = Affine(px, 0.0, left, 0.0, -px, top)

    profile = dict(
        driver="GTiff",
        width=g.width,
        height=g.height,
        count=BAND_COUNT,
        dtype="uint8",
        crs="EPSG:3857",
        transform=transform,
        tiled=True,
        blockxsize=BLOCK_PX,
        blockysize=BLOCK_PX,
        compress="DEFLATE",
        predictor=2,
        BIGTIFF="IF_SAFER",
    )

    print(f"[z{g.z}] writing {out_tif}")
    n_bad = 0
    with rasterio.open(out_tif, "w", **profile) as dst:
        strip = np.zeros((BAND_COUNT, TILE_PX, g.width), dtype=np.uint8)
        for row, y in enumerate(range(g.y_min, g.y_max + 1)):
            strip[:] = 0
            for col, x in enumerate(range(g.x_min, g.x_max + 1)):
                path = g.tiles.get((x, y))
                if path is None:
                    continue          # already reported as missing
                try:
                    tile = _read_tile(path)
                except Exception as exc:              # noqa: BLE001
                    n_bad += 1
                    print(f"\n[z{g.z}] WARNING: unreadable tile {path}: {exc} "
                          "-> black", file=sys.stderr)
                    continue
                x0 = col * TILE_PX
                strip[:, :, x0:x0 + TILE_PX] = tile
            dst.write(strip, window=Window(0, row * TILE_PX,
                                           g.width, TILE_PX))
            print(f"    tile row {row + 1}/{g.ny}", end="\r", flush=True)
        print()

        _, lat = g.centre_lonlat()
        dst.update_tags(
            SOURCE_PROVENANCE=provenance,
            TILE_SOURCE_DIR=str(tiles_root),
            TILE_ZOOM=str(g.z),
            TILE_GRID=f"x {g.x_min}..{g.x_max}, y {g.y_min}..{g.y_max}",
            TILE_COUNT_PRESENT=str(g.n_present),
            TILE_COUNT_EXPECTED=str(g.n_expected),
            TILE_COUNT_MISSING=str(len(missing)),
            TILE_COUNT_UNREADABLE=str(n_bad),
            MERC_PIXEL_SIZE_M=f"{px:.9f}",
            GSD_M_PER_PX=f"{g.gsd():.6f}",
            CENTRE_LATITUDE_DEG=f"{lat:.6f}",
            CREATED_UTC=datetime.datetime.now(
                datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
            GENERATOR="scripts/stitch_tiles.py",
        )

        if overviews:
            print(f"[z{g.z}] building overviews {OVERVIEW_FACTORS} (average)")
            dst.build_overviews(OVERVIEW_FACTORS, Resampling.average)
            dst.update_tags(ns="rio_overview", resampling="average")
        else:
            print(f"[z{g.z}] no overviews (--no-overviews)")

    if n_bad:
        print(f"[z{g.z}] WARNING: {n_bad} tile(s) could not be decoded",
              file=sys.stderr)
    return out_tif


def reproject_to_utm(in_tif: Path, out_tif: Path, epsg: int,
                     overviews: bool = True) -> Path:
    """Reproject a mosaic to UTM at its native resolution, bilinear.

    All bands are warped in ONE call. Warping them one at a time writes every
    destination block three times, and in a pixel-interleaved compressed
    GeoTIFF a re-compressed block no longer fits its old slot, so GDAL appends
    it and orphans the previous copy - which is how a 2.19 Gpx map came out at
    7.71 GB with 56.6 % dead bytes (measured, YenBai500m z19, 2026-08-14).
    Raising the block cache only helps while source *and* destination fit in
    it, which stops being possible at a few gigapixels; one pass per block does
    not depend on RAM at all. Pixels are unchanged: the one-call and the
    band-by-band output of z16 differ in 0 of 108 730 377 samples.
    """
    import rasterio
    from rasterio.enums import Resampling
    from rasterio.warp import calculate_default_transform, reproject

    dst_crs = f"EPSG:{epsg}"
    print(f"[utm] {out_tif}  ({dst_crs}, bilinear)")
    with rasterio.open(in_tif) as src:
        transform, width, height = calculate_default_transform(
            src.crs, dst_crs, src.width, src.height, *src.bounds
        )
        profile = src.profile.copy()
        profile.update(
            crs=dst_crs,
            transform=transform,
            width=width,
            height=height,
            driver="GTiff",
            tiled=True,
            blockxsize=BLOCK_PX,
            blockysize=BLOCK_PX,
            compress="DEFLATE",
            predictor=2,
            BIGTIFF="IF_SAFER",
        )
        tags = src.tags()
        with rasterio.open(out_tif, "w", **profile) as dst:
            print(f"    warping {src.count} band(s) in one pass ...",
                  flush=True)
            reproject(
                source=rasterio.band(src, src.indexes),
                destination=rasterio.band(dst, dst.indexes),
                src_transform=src.transform,
                src_crs=src.crs,
                dst_transform=transform,
                dst_crs=dst_crs,
                resampling=Resampling.bilinear,
            )
            tags["REPROJECTED_FROM"] = str(in_tif.name)
            tags["REPROJECT_RESAMPLING"] = "bilinear"
            dst.update_tags(**tags)
            if overviews:
                print(f"[utm] building overviews {OVERVIEW_FACTORS} (average)")
                dst.build_overviews(OVERVIEW_FACTORS, Resampling.average)
                dst.update_tags(ns="rio_overview", resampling="average")
            else:
                print("[utm] no overviews (--no-overviews)")
    return out_tif


# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(
        description="Mosaic a downloaded XYZ tile pyramid into georeferenced "
                    "GeoTIFFs, one _3857 + one _utm file per zoom level.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "example:\n"
            "  python3 scripts/stitch_tiles.py \\\n"
            "      --tiles '/media/ssd/Thesis Satellite Maps/Mun DS6' \\\n"
            "      --zooms 19 20 21 --out data/munfrl_bell412_dataset6/map \\\n"
            "      --name bell412_dataset6 --utm-epsg 32618\n"
        ),
    )
    ap.add_argument(
        "--tiles", required=True,
        help="root of the tile pyramid, holding <z>/<x>/<y>.png",
    )
    ap.add_argument(
        "--zooms", nargs="+", type=int, required=True,
        help="zoom levels to stitch, e.g. --zooms 19 20 21",
    )
    ap.add_argument("--out", required=True, help="output directory")
    ap.add_argument(
        "--name", required=True,
        help="output basename; files are <name>_z<zz>_3857.tif and "
             "<name>_z<zz>_utm.tif",
    )
    ap.add_argument(
        "--utm-epsg", type=int, default=None,
        help="EPSG code of the UTM copy (default: derived from the mosaic's "
             "centre lon/lat)",
    )
    ap.add_argument(
        "--no-utm", action="store_true",
        help="write only the EPSG:3857 mosaic",
    )
    ap.add_argument(
        "--no-overviews", action="store_true",
        help=f"do not build the {OVERVIEW_FACTORS} overview pyramid; each .tif "
             "then holds exactly one image (~1/3 smaller, but slower to pan "
             "and zoom in a GIS). Add them later with "
             "'gdaladdo -r average <file> 2 4 8 16'",
    )
    ap.add_argument(
        "--gdal-cachemax-mb", type=int, default=DEFAULT_GDAL_CACHEMAX_MB,
        help="GDAL block cache [MB] held during the whole writing phase "
             f"(default {DEFAULT_GDAL_CACHEMAX_MB}). A bigger cache means "
             "fewer block rewrites, hence a smaller file and less I/O; it "
             "cannot help once source plus destination stop fitting in RAM, "
             "which is why the reprojection warps all bands in one pass. "
             "Every written file is audited for orphaned blocks either way.",
    )
    ap.add_argument(
        "--provenance", default=PROVENANCE_PLACEHOLDER,
        help="text stored in the SOURCE_PROVENANCE GeoTIFF tag (imagery "
             "provider / capture date / licence)",
    )
    ap.add_argument(
        "--dry-run", action="store_true",
        help="print the grid/size/bbox table for every zoom and exit "
             "without writing anything",
    )
    ap.add_argument(
        "--overwrite", action="store_true",
        help="replace output files that already exist (default: skip the "
             "zoom and say so)",
    )
    args = ap.parse_args()

    tiles_root = Path(args.tiles).expanduser()
    if not tiles_root.is_dir():
        raise SystemExit(f"tile root not found: {tiles_root}")
    out_dir = Path(args.out).expanduser()

    print(f"[tiles] {tiles_root}")
    print(f"[out]   {out_dir}/{args.name}_z<zz>_{{3857,utm}}.tif")

    # ---- scan every zoom first: a broken tree should fail before writing --
    grids, missing_by_z = {}, {}
    for z in args.zooms:
        try:
            g = scan_zoom(tiles_root, z)
        except FileNotFoundError as exc:
            print(f"[z{z}] SKIPPED: {exc}", file=sys.stderr)
            continue
        grids[z] = g
        missing_by_z[z] = report_grid(g)
    if not grids:
        raise SystemExit("no usable zoom level found under " + str(tiles_root))

    print("\n" + "-" * 74)
    print(f"{'zoom':>5} {'tiles':>7} {'grid':>12} {'pixels':>14} "
          f"{'GSD m/px':>9} {'raw MB':>9}")
    print("-" * 74)
    for z, g in sorted(grids.items()):
        print(f"{z:>5} {g.n_present:>7} {f'{g.nx} x {g.ny}':>12} "
              f"{f'{g.width} x {g.height}':>14} {g.gsd():>9.4f} "
              f"{g.width * g.height * BAND_COUNT / 1e6:>9.0f}")
    print("-" * 74)
    print("raw MB is the uncompressed RGB size; DEFLATE typically lands well "
          "below it,\nand each UTM copy plus overviews adds roughly as much "
          "again.")

    if args.dry_run:
        print("\n[dry-run] nothing written")
        return

    if args.gdal_cachemax_mb <= 0:
        raise SystemExit("--gdal-cachemax-mb must be positive")

    import rasterio

    # rasterio routes GDAL_CACHEMAX to GDALSetCacheMax64, which takes an int
    # number of BYTES (a string raises TypeError, and the plain-config-option
    # form would read a small number as megabytes instead).
    cachemax_bytes = args.gdal_cachemax_mb * 1024 * 1024
    print(f"\n[gdal] block cache {args.gdal_cachemax_mb} MB "
          f"({cachemax_bytes} B)")

    biggest = max(g.width * g.height * BAND_COUNT for g in grids.values())
    if cachemax_bytes < biggest:
        print(f"[gdal] note: largest mosaic is {biggest / 2**20:.0f} MB "
              f"uncompressed, above the {args.gdal_cachemax_mb} MB cache - "
              "expect more I/O; correctness does not depend on it.")

    out_dir.mkdir(parents=True, exist_ok=True)
    failures = []
    with rasterio.Env(GDAL_CACHEMAX=cachemax_bytes):
        for z, g in sorted(grids.items()):
            merc_tif = out_dir / f"{args.name}_z{z:02d}_3857.tif"
            utm_tif = out_dir / f"{args.name}_z{z:02d}_utm.tif"
            try:
                if merc_tif.exists() and not args.overwrite:
                    print(f"\n[z{z}] {merc_tif.name} exists - skipped "
                          "(pass --overwrite to replace)")
                else:
                    stitch_zoom(g, merc_tif, tiles_root, args.provenance,
                                missing_by_z[z],
                                overviews=not args.no_overviews)
                    print(f"[z{z}] {merc_tif.name}  "
                          f"{merc_tif.stat().st_size / 1e6:.1f} MB")
                    report_dead_space(merc_tif, f"[z{z}]")

                if args.no_utm:
                    continue
                if utm_tif.exists() and not args.overwrite:
                    print(f"[z{z}] {utm_tif.name} exists - skipped "
                          "(pass --overwrite to replace)")
                    continue
                if args.utm_epsg is not None:
                    epsg = args.utm_epsg
                else:
                    lon, lat = g.centre_lonlat()
                    epsg = utm_epsg(lat, lon)
                    print(f"[z{z}] UTM zone derived from centre "
                          f"({lat:.5f}, {lon:.5f}) -> EPSG:{epsg}")
                reproject_to_utm(merc_tif, utm_tif, epsg,
                                 overviews=not args.no_overviews)
                print(f"[z{z}] {utm_tif.name}  "
                      f"{utm_tif.stat().st_size / 1e6:.1f} MB")
                report_dead_space(utm_tif, f"[z{z}]")
            except Exception as exc:                      # noqa: BLE001
                failures.append((z, exc))
                print(f"[z{z}] FAILED: {exc}", file=sys.stderr)

    if failures:
        print("\n" + "=" * 74, file=sys.stderr)
        for z, exc in failures:
            print(f"zoom {z} failed: {exc}", file=sys.stderr)
        raise SystemExit(f"{len(failures)} of {len(grids)} zoom level(s) failed")
    print("\n[done] all requested zoom levels written")


if __name__ == "__main__":
    main()
