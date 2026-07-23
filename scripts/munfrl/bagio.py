"""Thin, ROS-free wrappers around `rosbags.highlevel.AnyReader`.

Everything here works on a plain ROS 1 bag file (or a directory of split bags)
without a ROS installation. The custom `xsens_mti_driver/msg/IMUwithTimeRef`
message needs no special handling: rosbags auto-registers it from the message
definition embedded in the bag's connection records.
"""

from __future__ import annotations

import glob
import os
from contextlib import contextmanager
from pathlib import Path
from typing import Iterator, Sequence

import numpy as np

from rosbags.highlevel import AnyReader

# sensor_msgs/PointField datatype enum -> numpy dtype (ROS PointField.msg).
POINTFIELD_DTYPES = {
    1: np.dtype(np.int8),
    2: np.dtype(np.uint8),
    3: np.dtype(np.int16),
    4: np.dtype(np.uint16),
    5: np.dtype(np.int32),
    6: np.dtype(np.uint32),
    7: np.dtype(np.float32),
    8: np.dtype(np.float64),
}

BAG_GLOB = "*.bag"


def collect_bag_paths(path: str | os.PathLike | Sequence) -> list[Path]:
    """Return the bag files behind `path`.

    `path` may be a single .bag, a directory of split bags, or an explicit
    sequence of paths. Directory contents are sorted by name so the
    `-001, -002, ...` suffix order is preserved.
    """
    if isinstance(path, (list, tuple)):
        paths = [Path(p) for p in path]
        if not paths:
            raise FileNotFoundError("empty bag path list")
        for p in paths:
            if not p.is_file():
                raise FileNotFoundError(f"'{p}' is not a file")
        return sorted(paths)
    p = Path(path)
    if p.is_dir():
        bags = sorted(glob.glob(str(p / BAG_GLOB)))
        if not bags:
            raise FileNotFoundError(f"no {BAG_GLOB} files in directory '{p}'")
        return [Path(b) for b in bags]
    if p.is_file():
        return [p]
    raise FileNotFoundError(f"'{p}' is not a file or a directory")


@contextmanager
def open_bags(path: str | os.PathLike | Sequence) -> Iterator[AnyReader]:
    """Context manager yielding an opened `AnyReader` over `path`.

    `path` may be a single .bag, a directory of split bags, or an explicit list;
    AnyReader merges them into one time-ordered stream.
    """
    reader = AnyReader(collect_bag_paths(path))
    reader.open()
    try:
        yield reader
    finally:
        reader.close()


def topic_index(reader: AnyReader) -> dict[str, dict]:
    """Summarise the reader's topics.

    Returns ``{topic: {"msgtype", "count", "connections"}}`` sorted by topic.
    """
    out: dict[str, dict] = {}
    for conn in reader.connections:
        entry = out.setdefault(
            conn.topic, {"msgtype": conn.msgtype, "count": 0, "connections": []}
        )
        entry["count"] += conn.msgcount
        entry["connections"].append(conn)
    return dict(sorted(out.items()))


def connections_for(reader: AnyReader, topic: str) -> list:
    """All connections carrying `topic`; raises KeyError with a topic list."""
    conns = [c for c in reader.connections if c.topic == topic]
    if not conns:
        available = ", ".join(sorted({c.topic for c in reader.connections}))
        raise KeyError(f"topic '{topic}' not in bag. Available: {available}")
    return conns


def topic_bag_times(reader: AnyReader, topic: str) -> np.ndarray:
    """Bag log times (int64 nanoseconds) for `topic`, WITHOUT reading messages.

    ROS 1 bags carry a per-connection index that `rosbags` parses at open time,
    so this is essentially free — no chunk is touched. Used to derive the video
    frame rate before the single encode pass.

    Returns an empty array when the underlying reader exposes no index (e.g. a
    ROS 2 bag); callers should then fall back to accumulating stamps.
    """
    connections_for(reader, topic)  # raises with a helpful message if absent
    times: list[int] = []
    for sub in getattr(reader, "readers", []):
        indexes = getattr(sub, "indexes", None)
        if indexes is None:
            return np.empty(0, dtype=np.int64)
        for conn in sub.connections:
            if conn.topic == topic:
                times.extend(entry.time for entry in indexes.get(conn.id, ()))
    return np.asarray(sorted(times), dtype=np.int64)


def iter_topic(reader: AnyReader, topic: str, start=None, stop=None):
    """Yield ``(t_bag_ns, msg)`` for every message on `topic`, in log-time order."""
    conns = connections_for(reader, topic)
    for conn, t_ns, raw in reader.messages(connections=conns, start=start, stop=stop):
        yield t_ns, reader.deserialize(raw, conn.msgtype)


def first_message(reader: AnyReader, topic: str):
    """Return ``(t_bag_ns, msg)`` for the first message on `topic`, or None.

    Cheap: the reader seeks straight to the first index entry.
    """
    for t_ns, msg in iter_topic(reader, topic):
        return t_ns, msg
    return None


# --------------------------------------------------------------------------- #
# sensor_msgs/Image
# --------------------------------------------------------------------------- #
def _image_buffer(msg) -> np.ndarray:
    data = msg.data
    if isinstance(data, (bytes, bytearray, memoryview)):
        return np.frombuffer(bytes(data), dtype=np.uint8)
    return np.asarray(data, dtype=np.uint8).reshape(-1)


def image_msg_to_array(msg) -> np.ndarray:
    """Decode a sensor_msgs/Image into a numpy array, honouring `msg.step`.

    Returns ``(H, W)`` uint8 for mono8 and ``(H, W, 3)`` uint8 **BGR** for every
    colour/Bayer encoding — i.e. the OpenCV convention used across this repo.
    Raises ValueError for unsupported encodings.
    """
    enc = msg.encoding.lower()
    h, w, step = msg.height, msg.width, msg.step
    buf = _image_buffer(msg)
    expected = h * step
    if buf.size < expected:
        raise ValueError(
            f"image payload too small: {buf.size} bytes, need {expected} "
            f"({h} rows x step {step})"
        )
    rows = buf[:expected].reshape(h, step)

    if enc == "mono8":
        return np.ascontiguousarray(rows[:, :w])

    if enc in ("rgb8", "bgr8"):
        img = rows[:, : w * 3].reshape(h, w, 3)
        if enc == "rgb8":
            img = img[:, :, ::-1]
        return np.ascontiguousarray(img)

    if enc in ("rgba8", "bgra8"):
        img = rows[:, : w * 4].reshape(h, w, 4)[:, :, :3]
        if enc == "rgba8":
            img = img[:, :, ::-1]
        return np.ascontiguousarray(img)

    if enc.startswith("bayer"):
        try:
            import cv2
        except ImportError as exc:  # pragma: no cover - depends on env
            raise ValueError(f"encoding '{enc}' needs opencv-python") from exc
        code = {
            "bayer_rggb8": cv2.COLOR_BayerBG2BGR,
            "bayer_bggr8": cv2.COLOR_BayerRG2BGR,
            "bayer_gbrg8": cv2.COLOR_BayerGR2BGR,
            "bayer_grbg8": cv2.COLOR_BayerGB2BGR,
        }.get(enc)
        if code is None:
            raise ValueError(f"unsupported Bayer encoding '{enc}'")
        return cv2.cvtColor(np.ascontiguousarray(rows[:, :w]), code)

    raise ValueError(f"unsupported image encoding '{enc}'")


def image_msg_to_rgb(msg) -> np.ndarray:
    """Decode a sensor_msgs/Image to a contiguous ``(H, W, 3)`` **RGB** array.

    Mono images are replicated across the three channels. This is the layout
    ffmpeg's `rgb24` raw input expects.
    """
    img = image_msg_to_array(msg)
    if img.ndim == 2:
        return np.ascontiguousarray(np.repeat(img[:, :, None], 3, axis=2))
    return np.ascontiguousarray(img[:, :, ::-1])


# --------------------------------------------------------------------------- #
# sensor_msgs/PointCloud2
# --------------------------------------------------------------------------- #
def pointcloud2_dtype(msg) -> np.dtype:
    """Build the packed numpy dtype for a PointCloud2, from `msg.fields`.

    MUN-FRL's Velodyne clouds are byte-UNALIGNED (`x@0 y@4 z@8 intensity@12`
    float32, `ring@16` uint16, `time@18` float32, `point_step = 22`), so the
    dtype must be built with explicit ``offsets`` and ``itemsize`` — letting
    numpy pick its own padding silently corrupts every point.
    """
    names, formats, offsets = [], [], []
    for f in msg.fields:
        np_dt = POINTFIELD_DTYPES.get(int(f.datatype))
        if np_dt is None:
            raise ValueError(f"unknown PointField datatype {f.datatype} for '{f.name}'")
        count = max(1, int(f.count))
        names.append(f.name)
        formats.append(np_dt if count == 1 else (np_dt, count))
        offsets.append(int(f.offset))
    dt = np.dtype(
        {"names": names, "formats": formats, "offsets": offsets,
         "itemsize": int(msg.point_step)}
    )
    if msg.is_bigendian:
        dt = dt.newbyteorder(">")
    return dt


def pointcloud2_to_array(msg) -> np.ndarray:
    """Decode a sensor_msgs/PointCloud2 into a structured numpy array.

    Shape is ``(height, width)`` for organised clouds and ``(n,)`` otherwise.
    """
    dt = pointcloud2_dtype(msg)
    data = msg.data
    raw = bytes(data) if isinstance(data, (bytes, bytearray, memoryview)) else np.asarray(
        data, dtype=np.uint8
    ).tobytes()
    n_points = int(msg.width) * int(msg.height)
    arr = np.frombuffer(raw, dtype=dt, count=n_points)
    if int(msg.height) > 1:
        arr = arr.reshape(int(msg.height), int(msg.width))
    return arr


def pointcloud2_xyzi(msg, intensity_field: str = "intensity") -> np.ndarray:
    """Decode a PointCloud2 into a plain ``(n, 4)`` float32 ``x y z intensity``.

    Non-finite points (Velodyne no-returns) are dropped. When the cloud has no
    intensity field the fourth column is zero.
    """
    arr = np.asarray(pointcloud2_to_array(msg)).reshape(-1)
    xyz = np.stack([arr["x"], arr["y"], arr["z"]], axis=1).astype(np.float32)
    if intensity_field in arr.dtype.names:
        inten = arr[intensity_field].astype(np.float32)
    else:
        inten = np.zeros(xyz.shape[0], dtype=np.float32)
    out = np.concatenate([xyz, inten[:, None]], axis=1)
    return np.ascontiguousarray(out[np.isfinite(out).all(axis=1)])


def sequence_or_empty(value: Sequence | None) -> Sequence:
    """Normalise an optional sequence to something iterable."""
    return () if value is None else value
