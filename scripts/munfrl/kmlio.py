"""Minimal KML reader for the MUN-FRL `ppk_data/*.kml` track files.

The MUN-FRL KML is a single `<LineString>` whose `<coordinates>` element holds
every fix as a `lon,lat,alt` triple (the shipped files use `alt = 0`). Parsed
with the stdlib `xml.etree` — no extra dependency.

The KML carries **no timestamps**, so it is only useful as a geometric
cross-check of the `.pos` track.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

COORDS_TAG = "coordinates"


def _localname(tag: str) -> str:
    """Strip the `{namespace}` prefix ElementTree prepends to every tag."""
    return tag.rsplit("}", 1)[-1]


def parse_coordinate_blob(text: str) -> np.ndarray:
    """Parse a KML `<coordinates>` payload into an ``(n, 3)`` lon/lat/alt array.

    Tuples are whitespace separated and their components comma separated; the
    altitude component is optional and defaults to 0.
    """
    pts: list[tuple[float, float, float]] = []
    for token in text.split():
        parts = token.split(",")
        if len(parts) < 2:
            continue
        try:
            lon = float(parts[0])
            lat = float(parts[1])
            alt = float(parts[2]) if len(parts) > 2 and parts[2] != "" else 0.0
        except ValueError:
            continue
        pts.append((lon, lat, alt))
    if not pts:
        return np.empty((0, 3), dtype=np.float64)
    return np.asarray(pts, dtype=np.float64)


def read_kml_track(path: str | Path) -> np.ndarray:
    """Read every `<coordinates>` element of a KML into one ``(n, 3)`` array.

    Columns are ``longitude_deg, latitude_deg, altitude_m`` — note the KML
    lon-before-lat ordering, which is the opposite of the `.pos` file.
    """
    root = ET.parse(str(path)).getroot()
    blocks = [
        parse_coordinate_blob(el.text or "")
        for el in root.iter()
        if _localname(el.tag) == COORDS_TAG
    ]
    blocks = [b for b in blocks if b.size]
    if not blocks:
        raise ValueError(f"no <{COORDS_TAG}> data found in '{path}'")
    return np.concatenate(blocks, axis=0)
