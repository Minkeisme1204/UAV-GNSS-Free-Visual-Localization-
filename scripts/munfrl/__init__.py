"""munfrl — decoding helpers for the MUN-FRL Bell 412 aerial dataset.

Pure-Python (no ROS install required) readers for the artefacts shipped with a
MUN-FRL `bell412_datasetN` directory:

  * ROS 1 bag            -> `bagio`   (via the `rosbags` library)
  * fisheye calibration   -> `calib`  (camodocal / Kalibr -> pinhole rectify)
  * RTKLIB `.pos` PPK     -> `pos`
  * Google Earth `.kml`   -> `kmlio`
  * map point cloud `.pcd`-> `pcdio`
  * map_server-style JPEG -> `geomap`
  * GPS/UTC time algebra  -> `timeutil`
  * uavloc CSV emitters   -> `csvout`

See `.docs/designs/munfrl_dataset_decoding.md` for the dataset inventory, the
verified decoding traps, and the field mapping onto the uavloc telemetry CSV.
"""

from pathlib import Path

# Repository-relative default output root. `data/` is gitignored — decoded
# artefacts must never be committed.
REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OUTPUT_ROOT = REPO_ROOT / "data" / "munfrl_bell412_dataset6"

__all__ = [
    "DEFAULT_OUTPUT_ROOT",
    "REPO_ROOT",
    "bagio",
    "calib",
    "csvout",
    "geomap",
    "kmlio",
    "pcdio",
    "pos",
    "timeutil",
]
