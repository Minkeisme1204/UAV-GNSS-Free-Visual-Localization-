"""Generic PCD (Point Cloud Data) v0.7 reader/writer — ASCII and binary.

Written generically against the PCD header; nothing about the field order is
assumed. The MUN-FRL map cloud is nonetheless a good stress case because its
field order is unusual and includes a padding field:

    FIELDS intensity x y z _
    SIZE   4 4 4 4 1
    TYPE   F F F F U
    COUNT  1 1 1 1 4        -> itemsize 20, the trailing `_` is 4 pad bytes

`binary_compressed` is not implemented (the MUN-FRL maps do not use it).
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

HEADER_KEYS = (
    "VERSION", "FIELDS", "SIZE", "TYPE", "COUNT",
    "WIDTH", "HEIGHT", "VIEWPOINT", "POINTS", "DATA",
)
PAD_FIELD_NAME = "_"          # PCL's conventional name for padding fields
PAD_DTYPE_PREFIX = "V"        # numpy "void" -> raw bytes, never interpreted

# PCD TYPE letter -> numpy kind character.
_TYPE_KIND = {"I": "i", "U": "u", "F": "f"}


def _parse_header(fh) -> tuple[dict, int]:
    """Read the PCD header from a binary file handle.

    Returns ``(header_dict, data_start_offset)``. The handle is left positioned
    at the first data byte.
    """
    header: dict = {}
    while True:
        raw = fh.readline()
        if not raw:
            raise ValueError("unexpected EOF while reading PCD header")
        line = raw.decode("ascii", errors="replace").strip()
        if not line or line.startswith("#"):
            continue
        key, _, rest = line.partition(" ")
        key = key.upper()
        header[key] = rest.strip()
        if key == "DATA":
            return header, fh.tell()


def _build_dtype(header: dict) -> np.dtype:
    """Turn FIELDS/SIZE/TYPE/COUNT into a packed numpy structured dtype."""
    fields = header["FIELDS"].split()
    sizes = [int(x) for x in header["SIZE"].split()]
    types = header["TYPE"].split()
    counts = (
        [int(x) for x in header["COUNT"].split()]
        if "COUNT" in header
        else [1] * len(fields)
    )
    if not (len(fields) == len(sizes) == len(types) == len(counts)):
        raise ValueError("PCD header FIELDS/SIZE/TYPE/COUNT lengths disagree")

    names, formats, seen = [], [], {}
    for i, (name, size, typ, count) in enumerate(zip(fields, sizes, types, counts)):
        # Padding fields carry no information; keep the bytes, never decode them.
        if name == PAD_FIELD_NAME:
            names.append(f"pad{i}")
            formats.append(f"{PAD_DTYPE_PREFIX}{size * count}")
            continue
        kind = _TYPE_KIND.get(typ.upper())
        if kind is None:
            raise ValueError(f"unknown PCD TYPE '{typ}' for field '{name}'")
        # Guard against duplicate field names in a malformed header.
        uniq = name if name not in seen else f"{name}_{seen[name]}"
        seen[name] = seen.get(name, 0) + 1
        base = f"<{kind}{size}"
        names.append(uniq)
        formats.append(base if count == 1 else (base, count))
    return np.dtype({"names": names, "formats": formats})


def read_pcd(path: str | Path) -> tuple[dict, np.ndarray]:
    """Read a PCD file -> ``(header_dict, structured_array)``.

    ASCII and `binary` DATA are supported; `binary_compressed` raises
    NotImplementedError.
    """
    path = Path(path)
    with open(path, "rb") as fh:
        header, _ = _parse_header(fh)
        dtype = _build_dtype(header)
        n_points = int(header.get("POINTS") or
                       int(header["WIDTH"]) * int(header["HEIGHT"]))
        data_mode = header["DATA"].lower()

        if data_mode == "binary":
            arr = np.fromfile(fh, dtype=dtype, count=n_points)
        elif data_mode == "ascii":
            text = fh.read().decode("ascii", errors="replace")
            rows = [ln.split() for ln in text.splitlines() if ln.strip()]
            arr = np.zeros(len(rows), dtype=dtype)
            for col, name in enumerate(dtype.names):
                if name.startswith("pad"):
                    continue
                arr[name] = np.asarray([r[col] for r in rows], dtype=dtype[name])
        elif data_mode == "binary_compressed":
            raise NotImplementedError(
                "binary_compressed PCD is not supported (LZF decoding not implemented)"
            )
        else:
            raise ValueError(f"unknown PCD DATA mode '{header['DATA']}'")

    if arr.size != n_points:
        raise ValueError(
            f"PCD claims {n_points} points but only {arr.size} were read from '{path}'"
        )
    return header, arr


def to_xyzi(arr: np.ndarray, intensity_field: str = "intensity") -> np.ndarray:
    """Project a structured PCD array onto a plain ``(n, 4)`` float32 x/y/z/i array."""
    names = arr.dtype.names or ()
    for axis in ("x", "y", "z"):
        if axis not in names:
            raise ValueError(f"PCD has no '{axis}' field (fields: {names})")
    xyz = np.stack([arr["x"], arr["y"], arr["z"]], axis=1).astype(np.float32)
    if intensity_field in names:
        inten = arr[intensity_field].astype(np.float32)
    else:
        inten = np.zeros(xyz.shape[0], dtype=np.float32)
    return np.ascontiguousarray(np.concatenate([xyz, inten[:, None]], axis=1))


def write_pcd_xyzi(path: str | Path, xyzi: np.ndarray) -> None:
    """Write an ``(n, 4)`` float32 array as a standard `x y z intensity` binary PCD."""
    xyzi = np.ascontiguousarray(np.asarray(xyzi, dtype=np.float32))
    if xyzi.ndim != 2 or xyzi.shape[1] != 4:
        raise ValueError(f"expected an (n, 4) array, got shape {xyzi.shape}")
    n = xyzi.shape[0]
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA binary\n"
    )
    with open(path, "wb") as fh:
        fh.write(header.encode("ascii"))
        fh.write(xyzi.tobytes())


def bounds(xyzi: np.ndarray) -> dict:
    """Per-column min/max of an ``(n, 4)`` x/y/z/intensity array."""
    if xyzi.size == 0:
        return {}
    lo = xyzi.min(axis=0)
    hi = xyzi.max(axis=0)
    return {
        name: (float(lo[i]), float(hi[i]))
        for i, name in enumerate(("x", "y", "z", "intensity"))
    }
