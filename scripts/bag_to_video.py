#!/usr/bin/env python3
"""bag_to_video.py — MUN-FRL camera imagery from ROS 1 bags, in ONE pass.

Reads `sensor_msgs/Image` with the pure-Python `rosbags` library (no ROS, no
conda, no Docker) and either pipes raw frames into ffmpeg to produce an `.mp4`
(default) or dumps individual PNG frames (`--frames`).

Single-pass design
------------------
The frame rate is taken from the bag's own **per-connection index**, which
`rosbags` parses when the file is opened — no message payload is touched. The
image data is therefore read exactly once. (The previous version made three
passes over the bag, i.e. ~150 GB of reads for a 51 GB bag.)

While encoding, a companion `frames_<group>.csv` is written:

    frame_id,header_stamp_sec,bag_stamp_sec,timestamp_msec

`frame_id` is the 0-based index of the frame **in the produced video**, which is
exactly what `uavloc::sensor::VideoReader` will report, and `timestamp_msec` is
the header (capture) stamp in milliseconds — the join key for
`make_telemetry_csv.py`.

Camera topics in this dataset:
  /camera/image_color        bgr8  1440x1080  nadir (down)
  /camera/image_mono         mono8 1440x1080  nadir (down), same exposures
  /front_camera/image_color  bgr8   720x540   front
  /front_camera/image_mono   mono8  720x540   front

Fisheye rectification (`--rectify`)
-----------------------------------
The nadir camera is a Kannala-Brandt **fisheye**, while uavloc's `Camera:` block
models only pinhole + radtan. `--rectify --calib cam_intrinsic_bell412.yaml`
therefore undistorts every frame to a pinhole model *before* it is encoded, and
writes the resulting intrinsics next to the output as
`rectified_camera_<camera>.yaml` — that file, not the raw camodocal numbers, is
what belongs in the mission config.

Requirements: rosbags, numpy, ffmpeg on PATH; opencv for --frames/--resize/
--rectify/Bayer.

Examples:
  # nadir colour -> data/munfrl_bell412_dataset6/nadir_color.mp4 + frames_nadir.csv
  ./bag_to_video.py /media/.../bell412_dataset6 -t /camera/image_color

  # same, rectified to pinhole with no black borders
  ./bag_to_video.py /media/.../bell412_dataset6 -t /camera/image_color \
      --rectify --calib data/munfrl_bell412_dataset6/cam_intrinsic_bell412.yaml

  # a folder of split bags, front mono, forced 20 fps, explicit output
  ./bag_to_video.py ./bell412_dataset1/ -t /front_camera/image_mono -o front.mp4 --fps 20

  # cheap sanity run: first 30 frames only
  ./bag_to_video.py /media/.../bell412_dataset6 -t /camera/image_color --max-frames 30

  # PNG frames instead of a video
  ./bag_to_video.py bag.bag -t /camera/image_color --frames out_frames/
"""

from __future__ import annotations

import argparse
import csv
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np

from munfrl import DEFAULT_OUTPUT_ROOT, bagio, calib as calib_mod, timeutil

# Friendly output basenames for the known MUN-FRL camera topics.
TOPIC_VIDEO_NAMES = {
    "/camera/image_color": "nadir_color",
    "/camera/image_mono": "nadir_mono",
    "/front_camera/image_color": "front_color",
    "/front_camera/image_mono": "front_mono",
}
# Colour and mono share identical header stamps per exposure, so one frame CSV
# per physical camera is enough.
TOPIC_CAMERA_GROUPS = {
    "/camera/image_color": "nadir",
    "/camera/image_mono": "nadir",
    "/front_camera/image_color": "front",
    "/front_camera/image_mono": "front",
}

FRAME_CSV_COLUMNS = ("frame_id", "header_stamp_sec", "bag_stamp_sec", "timestamp_msec")

FALLBACK_FPS = 20.0        # used only when no timestamps are available at all
X264_CRF = "18"
X264_PRESET = "medium"
PROGRESS_EVERY = 200       # frames between progress lines

# `--rectify` writes the intrinsics of the rectified stream here, beside the
# video/PNGs. NEVER into config/ — that directory is the user's own area.
RECTIFIED_CALIB_PREFIX = "rectified_camera_"
RECTIFY_BALANCE = 0.0      # 0 = crop to the all-valid region (no black border)
RECTIFY_FOV_SCALE = 1.0


# --------------------------------------------------------------------------- #
def sanitize_topic(topic: str) -> str:
    """`/front_camera/image_mono` -> `front_camera_image_mono`."""
    return topic.strip("/").replace("/", "_")


def video_basename(topic: str) -> str:
    return TOPIC_VIDEO_NAMES.get(topic, sanitize_topic(topic))


def camera_group(topic: str) -> str:
    return TOPIC_CAMERA_GROUPS.get(topic, sanitize_topic(topic))


def estimate_fps(times_ns: np.ndarray, override: float | None) -> tuple[float, str]:
    """Frame rate from the median inter-frame gap. Returns ``(fps, provenance)``."""
    if override:
        return float(override), "cli override"
    times_ns = np.asarray(times_ns, dtype=np.int64)
    if times_ns.size < 2:
        return FALLBACK_FPS, "fallback (no timestamps)"
    dt = np.diff(times_ns) * 1e-9
    dt = dt[dt > 0]
    if dt.size == 0:
        return FALLBACK_FPS, "fallback (degenerate timestamps)"
    return float(round(1.0 / float(np.median(dt)), 3)), "bag index median dt"


def stamp_seconds(msg, t_bag_ns: int) -> float:
    """Header (capture) stamp in seconds; falls back to the bag log time."""
    t = timeutil.ros_stamp_to_epoch(msg.header.stamp)
    return t if t > 0 else timeutil.ns_to_epoch(t_bag_ns)


def spawn_ffmpeg(out_path: Path, width: int, height: int, fps: float,
                 crf: str, preset: str) -> subprocess.Popen:
    """Start an ffmpeg process consuming raw rgb24 frames on stdin."""
    out_path.parent.mkdir(parents=True, exist_ok=True)
    cmd = [
        "ffmpeg", "-y",
        "-f", "rawvideo", "-pix_fmt", "rgb24",
        "-s", f"{width}x{height}", "-r", str(fps),
        "-i", "-",
        "-an", "-c:v", "libx264", "-pix_fmt", "yuv420p",
        "-crf", crf, "-preset", preset,
        str(out_path),
    ]
    try:
        return subprocess.Popen(cmd, stdin=subprocess.PIPE)
    except FileNotFoundError:
        sys.exit("ERROR: ffmpeg not found on PATH (video mode needs it).")


def check_rectify_args(args) -> None:
    """`--rectify` and `--calib` only make sense together."""
    if args.rectify and not args.calib:
        raise ValueError("--rectify needs --calib PATH (the fisheye calibration).")
    if args.calib and not args.rectify:
        print("WARNING: --calib was given without --rectify; the calibration is "
              "only loaded for a sanity check and the frames are NOT rectified.",
              file=sys.stderr)


def write_rectified_calib(path: Path, k_new, size, camera_id: str,
                          fisheye, balance: float, fov_scale: float,
                          invalid_fraction: float | None = None) -> str:
    """Emit the rectified pinhole intrinsics as a uavloc `Camera:` block."""
    block = calib_mod.camera_block_yaml(k_new, size, camera_id, fisheye,
                                        balance, fov_scale, invalid_fraction)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(block, encoding="utf-8")
    return block


# --------------------------------------------------------------------------- #
def run(args) -> int:
    topic = args.topic
    out_dir = Path(args.out)
    check_rectify_args(args)

    frames_csv = Path(args.frames_csv) if args.frames_csv else (
        out_dir / f"frames_{camera_group(topic)}.csv"
    )
    frames_csv.parent.mkdir(parents=True, exist_ok=True)

    if args.frames:
        mode = "frames"
        frames_dir = Path(args.frames)
        frames_dir.mkdir(parents=True, exist_ok=True)
        target_desc = str(frames_dir)
        calib_out_dir = frames_dir
    else:
        mode = "video"
        video_path = Path(args.output) if args.output else (
            out_dir / f"{video_basename(topic)}.mp4"
        )
        target_desc = str(video_path)
        calib_out_dir = video_path.parent

    cv2 = None
    if mode == "frames" or args.resize or args.rectify:
        try:
            import cv2 as _cv2
        except ImportError:
            sys.exit("ERROR: --frames/--resize/--rectify need opencv-python-headless.")
        cv2 = _cv2

    fisheye = None
    if args.calib:
        fisheye = calib_mod.FisheyeCalib.from_yaml(args.calib)
        print(f"Calibration: {fisheye.describe()}")

    # Built once, on the first frame — never per frame.
    rect_map1 = rect_map2 = None
    ff = None
    width = height = 0
    n = 0

    with bagio.open_bags(args.path) as reader, \
            open(frames_csv, "w", newline="", encoding="utf-8") as csv_fh:
        writer = csv.writer(csv_fh)
        writer.writerow(FRAME_CSV_COLUMNS)

        bag_times = bagio.topic_bag_times(reader, topic)
        fps, provenance = estimate_fps(bag_times, args.fps)
        total = int(bag_times.size)
        print(f"Topic {topic}: {total} messages, fps {fps} ({provenance})")
        print(f"Mode: {mode} -> {target_desc}")
        print(f"Frame CSV: {frames_csv}")

        for t_bag_ns, msg in bagio.iter_topic(reader, topic):
            if args.max_frames and n >= args.max_frames:
                break
            frame = bagio.image_msg_to_rgb(msg)

            if args.rectify:
                if rect_map1 is None:
                    src_h, src_w = frame.shape[:2]
                    # A calibration applied at the wrong scale mis-maps every
                    # pixel and nothing downstream can notice — abort here.
                    fisheye.check_frame_size(src_w, src_h)
                    rect_map1, rect_map2, k_new = calib_mod.rectify_maps(
                        fisheye, balance=args.rectify_balance,
                        fov_scale=args.rectify_fov_scale,
                    )
                    invalid = calib_mod.invalid_pixel_fraction(
                        fisheye, rect_map1, rect_map2
                    )
                    if invalid > 0.0:
                        print(f"WARNING: {100 * invalid:.4f}% of the rectified "
                              "frame falls outside the source (black border). "
                              "Tighten it with --rectify-fov-scale < 1.",
                              file=sys.stderr)
                    # --resize runs after rectification, so the emitted
                    # intrinsics must describe the resized frames.
                    out_size = tuple(args.resize) if args.resize else (src_w, src_h)
                    k_out = calib_mod.scale_intrinsics(
                        k_new, (src_w, src_h), out_size
                    )
                    calib_path = calib_out_dir / (
                        f"{RECTIFIED_CALIB_PREFIX}{camera_group(topic)}.yaml"
                    )
                    block = write_rectified_calib(
                        calib_path, k_out, out_size, camera_group(topic),
                        fisheye, args.rectify_balance, args.rectify_fov_scale,
                        invalid,
                    )
                    print(f"Rectified intrinsics -> {calib_path}\n{block}")
                frame = cv2.remap(frame, rect_map1, rect_map2, cv2.INTER_LINEAR)

            if args.resize:
                frame = cv2.resize(frame, (args.resize[0], args.resize[1]))
            if width == 0:
                height, width = frame.shape[:2]
                if mode == "video":
                    ff = spawn_ffmpeg(video_path, width, height, fps,
                                      args.crf, args.preset)

            if mode == "video":
                try:
                    ff.stdin.write(np.ascontiguousarray(frame).tobytes())
                except BrokenPipeError:
                    print("\nERROR: ffmpeg exited early; aborting.", file=sys.stderr)
                    return 1
            else:
                cv2.imwrite(str(frames_dir / f"frame_{n:06d}.png"), frame[:, :, ::-1])

            t_header = stamp_seconds(msg, t_bag_ns)
            writer.writerow([
                n,
                f"{t_header:.9f}",
                f"{timeutil.ns_to_epoch(t_bag_ns):.9f}",
                timeutil.format_msec(t_header),
            ])
            n += 1
            if n % PROGRESS_EVERY == 0:
                print(f"  {n}/{total} frames", end="\r", flush=True)

    if ff is not None:
        ff.stdin.close()
        ff.wait()
    if n == 0:
        print(f"ERROR: no frames on topic '{topic}'.", file=sys.stderr)
        return 1
    rect_desc = " [rectified to pinhole]" if args.rectify else ""
    print(f"\nDone: {n} frames @ {fps} fps ({width}x{height}){rect_desc} "
          f"-> {target_desc}")
    print(f"      {frames_csv}")
    return 0


# --------------------------------------------------------------------------- #
def main() -> int:
    p = argparse.ArgumentParser(
        description="Extract MUN-FRL camera imagery from ROS 1 bags in a single pass.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("path", help="A .bag file, or a dataset/split-bag directory.")
    p.add_argument("-t", "--topic", required=True,
                   help="Image topic, e.g. /camera/image_color")
    p.add_argument("-o", "--output",
                   help="Explicit output .mp4 path (default: <out>/<topic-name>.mp4).")
    p.add_argument("--out", default=str(DEFAULT_OUTPUT_ROOT),
                   help="Output directory for the video and the frame CSV.")
    p.add_argument("--frames", metavar="DIR",
                   help="Dump PNG frames to DIR instead of encoding a video.")
    p.add_argument("--frames-csv", metavar="PATH",
                   help="Explicit frame-timestamp CSV path "
                        "(default: <out>/frames_<camera>.csv).")
    p.add_argument("--fps", type=float,
                   help="Force output fps (default: median dt from the bag index).")
    p.add_argument("--resize", nargs=2, type=int, metavar=("W", "H"),
                   help="Resize each frame to W H after rectification.")
    p.add_argument("--rectify", action="store_true",
                   help="Undistort the fisheye frames to a pinhole model "
                        "(needs --calib). Writes the resulting intrinsics to "
                        f"<out>/{RECTIFIED_CALIB_PREFIX}<camera>.yaml.")
    p.add_argument("--calib", metavar="PATH",
                   help="Fisheye calibration YAML (camodocal KANNALA_BRANDT or "
                        "Kalibr/SVO equidistant). Required with --rectify.")
    p.add_argument("--rectify-balance", type=float, default=RECTIFY_BALANCE,
                   help="0 = crop to the all-valid region (no black border), "
                        "1 = keep the whole FOV with black corners.")
    p.add_argument("--rectify-fov-scale", type=float, default=RECTIFY_FOV_SCALE,
                   help="Extra FOV multiplier for the rectified view (>1 widens).")
    p.add_argument("--max-frames", type=int, default=0,
                   help="Stop after N frames (0 = all). Useful for smoke tests.")
    p.add_argument("--crf", default=X264_CRF, help="libx264 CRF quality.")
    p.add_argument("--preset", default=X264_PRESET, help="libx264 speed preset.")
    args = p.parse_args()

    try:
        return run(args)
    except (FileNotFoundError, KeyError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
