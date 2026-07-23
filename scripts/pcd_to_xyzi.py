#!/usr/bin/env python3
"""pcd_to_xyzi.py — MUN-FRL map point cloud -> `.npy` and/or a standard PCD.

The shipped map cloud has an unusual header that trips naive readers:

    FIELDS intensity x y z _        <- intensity FIRST, and a 4-byte pad field
    SIZE   4 4 4 4 1
    TYPE   F F F F U
    COUNT  1 1 1 1 4                <- itemsize 20, not 16

`munfrl.pcdio` parses the header generically, so the field order is discovered
rather than assumed. The output is always the conventional `x, y, z, intensity`
column order.

The coordinates are in the dataset's **local metric frame** — the same frame the
map JPEG is georeferenced in (see `map_overlay.py`), NOT WGS84.

Examples:
  ./pcd_to_xyzi.py /media/.../map_pcd/bell412_dataset6.pcd
  ./pcd_to_xyzi.py map.pcd --subsample 10 --write-pcd
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np

from munfrl import DEFAULT_OUTPUT_ROOT, pcdio

NPY_NAME = "map_cloud.npy"
PCD_NAME = "map_cloud.pcd"


def main() -> int:
    p = argparse.ArgumentParser(
        description="Convert a PCD map cloud to an (n, 4) x/y/z/intensity array.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("pcd_file", help="Path to the source .pcd.")
    p.add_argument("--out", default=str(DEFAULT_OUTPUT_ROOT), help="Output directory.")
    p.add_argument("--npy-name", default=NPY_NAME, help="Output .npy file name.")
    p.add_argument("--pcd-name", default=PCD_NAME, help="Output .pcd file name.")
    p.add_argument("--subsample", type=int, default=1, metavar="N",
                   help="Keep every N-th point (1 = keep all).")
    p.add_argument("--write-pcd", action="store_true",
                   help="Also write a standard 'FIELDS x y z intensity' binary PCD.")
    p.add_argument("--no-npy", action="store_true", help="Skip the .npy output.")
    args = p.parse_args()

    src = Path(args.pcd_file)
    if not src.is_file():
        print(f"ERROR: '{src}' is not a file", file=sys.stderr)
        return 2
    if args.subsample < 1:
        print("ERROR: --subsample must be >= 1", file=sys.stderr)
        return 2

    header, arr = pcdio.read_pcd(src)
    print("PCD header:")
    for key, value in header.items():
        print(f"  {key:10s} {value}")
    print(f"  numpy dtype {arr.dtype} (itemsize {arr.dtype.itemsize} B)")

    xyzi = pcdio.to_xyzi(arr)
    if args.subsample > 1:
        xyzi = np.ascontiguousarray(xyzi[:: args.subsample])
    print(f"\nPoints: {xyzi.shape[0]}"
          + (f"  (subsampled 1/{args.subsample} of {arr.size})"
             if args.subsample > 1 else ""))
    for axis, (lo, hi) in pcdio.bounds(xyzi).items():
        print(f"  {axis:9s} [{lo:.3f}, {hi:.3f}]")

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    if not args.no_npy:
        npy_path = out_dir / args.npy_name
        np.save(npy_path, xyzi)
        print(f"\nWritten: {npy_path}")
    if args.write_pcd:
        pcd_path = out_dir / args.pcd_name
        pcdio.write_pcd_xyzi(pcd_path, xyzi)
        print(f"Written: {pcd_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
