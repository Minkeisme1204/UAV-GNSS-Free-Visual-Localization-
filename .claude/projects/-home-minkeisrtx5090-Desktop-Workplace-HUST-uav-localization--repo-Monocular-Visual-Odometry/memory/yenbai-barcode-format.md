---
name: yenbai-barcode-format
description: What the per-frame barcode in the YenBai800m video actually encodes (imageId, not telemetry)
metadata:
  type: project
---

The per-frame barcode in `data/YenBai800m/cut_2025-07-30_800m-1x.mkv` does NOT carry telemetry directly. It is a **CODE-128** symbol near the top edge (~x=174, y=0) whose payload is a **zero-padded 7-digit integer equal to the CSV `imageId`** (increments by 1 per frame). This `.mkv` is a cut starting at imageId **55166**.

To get telemetry (lat/lon/yaw/alt) you must decode the barcode → imageId → look it up in `data/YenBai800m/cut_2025-07-30_800m-1x.csv` (col0 imageId, 21 roll, 22 pitch, 23 yaw, 26 groundSpeed, 30 sensorLatitude, 31 sensorLongitude, 32 sensorAltitude; ~91741 records). Cross-check: imageId 55166 → lat 21.702024, lon 104.850000, alt 693.46, yaw -39.7662.

This is implemented in `src/debug_viewer/barcode_decoder.cpp` (zbar, all symbologies, returns the integer) and `src/debug_viewer/debug_viewer.cpp` (CSV index + lookup). The viewer is headless-safe (skips GL rendering when no DISPLAY). Communication-design rationale lives in `.docs/designs/debug_viewer_communication.md`. Related: [[debug-viewer-module]].
