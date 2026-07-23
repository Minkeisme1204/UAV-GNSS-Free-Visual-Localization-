"""Geo-parameters of the MUN-FRL map JPEG, recovered from its file name.

MUN-FRL ships the 2-D reference map as a plain JPEG whose ROS `map_server`
parameters are baked into the file name, e.g.

    dataset6_3_602_pxpm_map_yaml_parameters_0_277624_-502_-86.jpg
             ^^^^^                          ^^^^^^^^ ^^^^ ^^^
             3.602 px/m                     0.277624 m/px, origin (-502, -86) m

`1 / 3.602 = 0.277624`, so the two numbers are consistent. Combined with the
image size (4800 x 2991) the map covers x in [-502, 830.2] m and
y in [-86, 744.4] m, which brackets the map point cloud's bounds — the JPEG and
the `.pcd` therefore share one local metric frame.

Pixel convention follows `map_server`: the origin sits at the **bottom-left** of
the image and y grows upward, so the row index has to be flipped.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np

# `<int>_<frac>_pxpm_ ... _<int>_<frac>_<origin_x>_<origin_y>` at the stem's end.
FILENAME_RE = re.compile(
    r"(?P<ppm_i>\d+)_(?P<ppm_f>\d+)_pxpm_"
    r".*?_(?P<res_i>\d+)_(?P<res_f>\d+)_(?P<ox>-?\d+)_(?P<oy>-?\d+)$"
)

# Tolerance when checking that resolution == 1 / pixels_per_metre.
RESOLUTION_CONSISTENCY_TOL = 1e-4


@dataclass
class MapGeo:
    """Geo-referencing of a `map_server`-style raster in a local metric frame."""

    resolution: float          # metres per pixel
    origin_x: float            # metric x of the image's bottom-left corner
    origin_y: float            # metric y of the image's bottom-left corner
    width: int = 0             # image width in pixels
    height: int = 0            # image height in pixels
    pixels_per_metre: Optional[float] = None
    source: str = ""           # where these numbers came from (file name, CLI, ...)

    # -- construction ------------------------------------------------------ #
    @classmethod
    def from_filename(cls, path: str | Path,
                      width: int = 0, height: int = 0) -> "MapGeo":
        """Parse the geo-parameters out of a MUN-FRL map file name.

        Raises ValueError when the name does not carry the `_pxpm_` pattern.
        """
        stem = Path(path).stem
        m = FILENAME_RE.search(stem)
        if not m:
            raise ValueError(
                f"file name '{stem}' does not match the MUN-FRL map pattern "
                "'<a>_<b>_pxpm_..._<c>_<d>_<origin_x>_<origin_y>'"
            )
        ppm = float(f"{m.group('ppm_i')}.{m.group('ppm_f')}")
        res = float(f"{m.group('res_i')}.{m.group('res_f')}")
        return cls(
            resolution=res,
            origin_x=float(m.group("ox")),
            origin_y=float(m.group("oy")),
            width=int(width),
            height=int(height),
            pixels_per_metre=ppm,
            source=f"filename:{Path(path).name}",
        )

    # -- queries ----------------------------------------------------------- #
    def resolution_is_consistent(self) -> bool:
        """True when `resolution` matches `1 / pixels_per_metre` (or ppm is unknown)."""
        if not self.pixels_per_metre:
            return True
        return abs(self.resolution - 1.0 / self.pixels_per_metre) < RESOLUTION_CONSISTENCY_TOL

    def extent(self) -> tuple[float, float, float, float]:
        """Metric coverage ``(x_min, x_max, y_min, y_max)`` of the raster."""
        return (
            self.origin_x,
            self.origin_x + self.width * self.resolution,
            self.origin_y,
            self.origin_y + self.height * self.resolution,
        )

    # -- conversions ------------------------------------------------------- #
    def xy_to_pixel(self, x, y):
        """Local metric (x, y) -> (px, py) float pixel coordinates.

        `map_server` puts the origin at the bottom-left, hence the row flip:
        ``py = height - 1 - (y - origin_y) / resolution``.
        """
        x = np.asarray(x, dtype=np.float64)
        y = np.asarray(y, dtype=np.float64)
        px = (x - self.origin_x) / self.resolution
        py = (self.height - 1) - (y - self.origin_y) / self.resolution
        return px, py

    def pixel_to_xy(self, px, py):
        """(px, py) pixel coordinates -> local metric (x, y). Inverse of `xy_to_pixel`."""
        px = np.asarray(px, dtype=np.float64)
        py = np.asarray(py, dtype=np.float64)
        x = px * self.resolution + self.origin_x
        y = ((self.height - 1) - py) * self.resolution + self.origin_y
        return x, y

    def describe(self) -> str:
        """One-paragraph human summary, for CLI output."""
        x0, x1, y0, y1 = self.extent()
        ppm = self.pixels_per_metre if self.pixels_per_metre else float("nan")
        return (
            f"resolution {self.resolution:.6f} m/px ({ppm:.3f} px/m), "
            f"origin ({self.origin_x:.1f}, {self.origin_y:.1f}) m, "
            f"size {self.width}x{self.height} px, "
            f"extent x [{x0:.1f}, {x1:.1f}] m, y [{y0:.1f}, {y1:.1f}] m "
            f"[{self.source}]"
        )


# --------------------------------------------------------------------------- #
# Flat-earth ENU, mirroring src/debug_viewer/gps_to_enu.h
# --------------------------------------------------------------------------- #
# Metres per degree of latitude. Same constant the C++ viewer uses; the
# per-degree longitude scale is derived from it with cos(lat0). Good enough for
# tracks up to a few km, which covers every MUN-FRL flight.
METRES_PER_DEG_LAT = 111319.5


def flat_earth_enu(lat, lon, alt, lat0: float, lon0: float, alt0: float):
    """WGS84 lat/lon/alt -> local ENU metres anchored at (lat0, lon0, alt0).

    Flat-earth approximation, identical in form to
    `src/debug_viewer/gps_to_enu.h`. Returns ``(east, north, up)`` arrays.
    """
    lat = np.asarray(lat, dtype=np.float64)
    lon = np.asarray(lon, dtype=np.float64)
    alt = np.asarray(alt, dtype=np.float64)
    m_lon = METRES_PER_DEG_LAT * np.cos(np.deg2rad(lat0))
    east = (lon - lon0) * m_lon
    north = (lat - lat0) * METRES_PER_DEG_LAT
    up = alt - alt0
    return east, north, up
