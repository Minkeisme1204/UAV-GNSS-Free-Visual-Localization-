"""Fisheye (Kannala-Brandt) camera calibration + pinhole rectification.

The MUN-FRL nadir camera is a **fisheye**; its official calibration is published
in camodocal format:

    model_type: KANNALA_BRANDT
    image_width: 1440
    image_height: 1080
    projection_parameters:
       k2: -0.0764245     k3: 0.0322856     k4: -0.0445168    k5: 0.0163317
       mu: 829.224        mv: 829.454       u0: 833.937       v0: 562.509

camodocal's `EquidistantCamera` uses `theta_d = theta (1 + k2 th^2 + k3 th^4 +
k4 th^6 + k5 th^8)` and `u = mu * x_d + u0`, which is exactly OpenCV's
`cv::fisheye` model. The mapping is therefore a pure rename:

    K = [[mu, 0, u0], [0, mv, v0], [0, 0, 1]]
    D = [k2, k3, k4, k5]                       # cv::fisheye's (k1..k4)

Sanity anchor: `mu * 3.45 um` (Sony IMX273 pixel pitch) = 2.861 mm, i.e. the
lens' 2.8 mm wide end.

uavloc's `Camera:` config block only models **pinhole + radtan**
(`fx fy cx cy k1 k2 p1 p2 k3`) and the VO module's `undistort_image` cannot
represent an equidistant lens, so frames must be rectified to a pinhole model
**at extraction time** (`bag_to_video.py --rectify`). After that the raw
camodocal numbers are obsolete: the rectified stream's intrinsics are `K_new`,
with all distortion coefficients zero.
"""

from __future__ import annotations

import datetime
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Optional, Sequence

import numpy as np

# Every spelling of the equidistant / Kannala-Brandt model we accept. Anything
# else (MEI, PINHOLE, SCARAMUZZA, radtan, ...) is rejected rather than silently
# mis-modelled — applying the wrong lens model is a quiet, corrupting error.
KANNALA_BRANDT_ALIASES = frozenset(
    {"KANNALA_BRANDT", "KANNALABRANDT", "KANNALA-BRANDT", "EQUIDISTANT", "FISHEYE"}
)
CANONICAL_MODEL = "KANNALA_BRANDT"

# camodocal `projection_parameters` key names, in cv::fisheye order.
CAMODOCAL_DIST_KEYS = ("k2", "k3", "k4", "k5")
CAMODOCAL_PROJ_KEYS = ("mu", "mv", "u0", "v0")

N_DIST_COEFFS = 4


# --------------------------------------------------------------------------- #
# YAML reading — no new dependency
# --------------------------------------------------------------------------- #
def _scalar(token: str) -> Any:
    """Convert a bare YAML scalar to int / float / bool / str."""
    t = token.strip()
    if len(t) >= 2 and t[0] == t[-1] and t[0] in "\"'":
        return t[1:-1]
    low = t.lower()
    if low in ("true", "false"):
        return low == "true"
    if low in ("null", "~", ""):
        return None
    try:
        return int(t)
    except ValueError:
        pass
    try:
        return float(t)
    except ValueError:
        return t


def _parse_simple_yaml(text: str) -> dict:
    """Minimal indentation-aware parser for calibration YAML files.

    Supports exactly what calibration files use: nested `key:` mappings, scalar
    `key: value` pairs and inline lists `key: [a, b, c]`. `#` comments, blank
    lines, a leading `---` and OpenCV's non-standard `%YAML:1.0` directive are
    skipped. Anything richer (anchors, block lists, multi-line strings) is out
    of scope — use `ruamel.yaml` if such a file ever shows up.
    """
    root: dict = {}
    stack: list[tuple[int, dict]] = [(-1, root)]
    for lineno, raw in enumerate(text.splitlines(), start=1):
        line = raw.split("#", 1)[0].rstrip()
        if not line.strip() or line.strip() == "---" or line.lstrip().startswith("%YAML"):
            continue
        indent = len(line) - len(line.lstrip())
        if ":" not in line:
            raise ValueError(f"line {lineno}: expected 'key: value', got '{raw.strip()}'")
        key, _, value = line.strip().partition(":")
        key = key.strip()
        value = value.strip()
        while stack and indent <= stack[-1][0]:
            stack.pop()
        if not stack:
            raise ValueError(f"line {lineno}: inconsistent indentation")
        parent = stack[-1][1]
        if not value:
            child: dict = {}
            parent[key] = child
            stack.append((indent, child))
        elif value.startswith("[") and value.endswith("]"):
            inner = value[1:-1].strip()
            parent[key] = [_scalar(tok) for tok in inner.split(",")] if inner else []
        else:
            parent[key] = _scalar(value)
    return root


def load_yaml_mapping(path: str | Path) -> dict:
    """Read a calibration YAML into a plain nested dict.

    Uses `ruamel.yaml` when it is importable (it copes with anything a real YAML
    emitter produces) and falls back to `_parse_simple_yaml` otherwise, so the
    module has no hard dependency beyond numpy.
    """
    text = Path(path).read_text(encoding="utf-8")
    # OpenCV FileStorage writes a non-standard `%YAML:1.0` directive that no
    # conforming parser accepts; drop it before handing the text over.
    text = "\n".join(
        ln for ln in text.splitlines() if not ln.lstrip().startswith("%YAML")
    )
    try:
        from ruamel.yaml import YAML  # type: ignore
    except ImportError:
        return _parse_simple_yaml(text)
    try:
        data = YAML(typ="safe").load(text)
    except Exception:  # malformed for a strict parser -> try the tolerant one
        return _parse_simple_yaml(text)
    if not isinstance(data, dict):
        raise ValueError(f"'{path}' does not contain a YAML mapping")
    return data


# --------------------------------------------------------------------------- #
# Calibration
# --------------------------------------------------------------------------- #
@dataclass
class FisheyeCalib:
    """A Kannala-Brandt (equidistant) fisheye calibration, in OpenCV terms."""

    model_type: str            # always CANONICAL_MODEL once constructed
    width: int                 # calibrated image width in pixels
    height: int                # calibrated image height in pixels
    K: np.ndarray              # 3x3 float64
    D: np.ndarray              # (4,) float64, cv::fisheye k1..k4
    camera_name: str = "camera"
    source: str = ""           # file the numbers came from

    # -- construction ------------------------------------------------------ #
    @classmethod
    def from_yaml(cls, path: str | Path) -> "FisheyeCalib":
        """Load a camodocal- or Kalibr/SVO-style fisheye calibration file."""
        return cls.from_mapping(load_yaml_mapping(path), source=str(path))

    @classmethod
    def from_mapping(cls, node: Mapping, source: str = "") -> "FisheyeCalib":
        """Build from an already-parsed mapping. Raises ValueError on any doubt."""
        if _looks_camodocal(node):
            return cls._from_camodocal(node, source)
        cam_key, cam = _find_kalibr_cam(node)
        if cam is not None:
            return cls._from_kalibr(cam, source, cam_key)
        raise ValueError(
            f"'{source}': unrecognised calibration layout. Expected camodocal "
            "(model_type + projection_parameters) or Kalibr/SVO "
            "(intrinsics + distortion_coeffs)."
        )

    @classmethod
    def _from_camodocal(cls, node: Mapping, source: str) -> "FisheyeCalib":
        _check_model(node.get("model_type"), source, "model_type")
        proj = node.get("projection_parameters") or {}
        if not isinstance(proj, Mapping):
            raise ValueError(f"'{source}': 'projection_parameters' is not a mapping")
        missing = [k for k in CAMODOCAL_PROJ_KEYS + CAMODOCAL_DIST_KEYS if k not in proj]
        if missing:
            raise ValueError(
                f"'{source}': projection_parameters is missing {', '.join(missing)}"
            )
        mu, mv, u0, v0 = (float(proj[k]) for k in CAMODOCAL_PROJ_KEYS)
        d = [float(proj[k]) for k in CAMODOCAL_DIST_KEYS]
        return cls(
            model_type=CANONICAL_MODEL,
            width=_positive_int(node.get("image_width"), "image_width", source),
            height=_positive_int(node.get("image_height"), "image_height", source),
            K=_build_k(mu, mv, u0, v0),
            D=np.asarray(d, dtype=np.float64),
            camera_name=str(node.get("camera_name", "camera")),
            source=source,
        )

    @classmethod
    def _from_kalibr(cls, cam: Mapping, source: str, cam_key: str) -> "FisheyeCalib":
        where = f"{source}:{cam_key}" if cam_key else source
        _check_model(cam.get("distortion_model"), where, "distortion_model")
        intr = _float_list(cam.get("intrinsics"), 4, "intrinsics", where)
        dist = _float_list(cam.get("distortion_coeffs"), N_DIST_COEFFS,
                           "distortion_coeffs", where)
        res = _float_list(cam.get("resolution"), 2, "resolution", where)
        fx, fy, cx, cy = intr
        return cls(
            model_type=CANONICAL_MODEL,
            width=_positive_int(res[0], "resolution[0]", where),
            height=_positive_int(res[1], "resolution[1]", where),
            K=_build_k(fx, fy, cx, cy),
            D=np.asarray(dist, dtype=np.float64),
            camera_name=str(cam.get("rostopic", cam_key or "camera")),
            source=source,
        )

    # -- queries ----------------------------------------------------------- #
    @property
    def size(self) -> tuple[int, int]:
        """`(width, height)` — the `cv2` image-size convention."""
        return (self.width, self.height)

    def check_frame_size(self, width: int, height: int) -> None:
        """Abort unless the frames match the calibrated resolution.

        A calibration applied at the wrong scale is a silent, corrupting error:
        every pixel is mis-mapped and nothing downstream can notice.
        """
        if (int(width), int(height)) != self.size:
            raise ValueError(
                f"calibration '{self.source}' was made for "
                f"{self.width}x{self.height} but the frames are {width}x{height}. "
                "Rescale the calibration (or extract at the calibrated "
                "resolution) — do NOT apply it as-is."
            )

    def describe(self) -> str:
        """One-line human summary, for CLI output."""
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]
        d = ", ".join(f"{v:+.7g}" for v in self.D)
        return (
            f"{self.model_type} '{self.camera_name}' {self.width}x{self.height}, "
            f"fx {fx:.3f} fy {fy:.3f} cx {cx:.3f} cy {cy:.3f}, D [{d}]"
        )


# -- construction helpers --------------------------------------------------- #
def _build_k(fx: float, fy: float, cx: float, cy: float) -> np.ndarray:
    return np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)


def _looks_camodocal(node: Mapping) -> bool:
    return "model_type" in node or "projection_parameters" in node


def _find_kalibr_cam(node: Mapping) -> tuple[str, Optional[Mapping]]:
    """Locate the camera block of a Kalibr/SVO file (flat, or under `camN`)."""
    if "intrinsics" in node:
        return "", node
    for key in sorted(node):
        value = node[key]
        if isinstance(value, Mapping) and "intrinsics" in value:
            return key, value
    return "", None


def _check_model(value, source: str, field: str) -> None:
    if value is None:
        raise ValueError(f"'{source}': missing '{field}'")
    name = str(value).strip().upper()
    if name not in KANNALA_BRANDT_ALIASES:
        raise ValueError(
            f"'{source}': {field} is '{value}', but only the Kannala-Brandt / "
            "equidistant fisheye model is supported here "
            f"(accepted spellings: {', '.join(sorted(KANNALA_BRANDT_ALIASES))}). "
            "Rectifying with the wrong lens model would silently corrupt the "
            "geometry — refusing."
        )


def _positive_int(value, field: str, source: str) -> int:
    if value is None:
        raise ValueError(f"'{source}': missing '{field}'")
    try:
        n = int(round(float(value)))
    except (TypeError, ValueError) as exc:
        raise ValueError(f"'{source}': '{field}' is not a number ({value!r})") from exc
    if n <= 0:
        raise ValueError(f"'{source}': '{field}' must be positive, got {n}")
    return n


def _float_list(value, n: int, field: str, source: str) -> list[float]:
    if value is None:
        raise ValueError(f"'{source}': missing '{field}'")
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        raise ValueError(f"'{source}': '{field}' must be a list of {n} numbers")
    if len(value) != n:
        raise ValueError(
            f"'{source}': '{field}' must have {n} entries, got {len(value)}"
        )
    try:
        return [float(v) for v in value]
    except (TypeError, ValueError) as exc:
        raise ValueError(f"'{source}': '{field}' holds a non-number") from exc


# --------------------------------------------------------------------------- #
# Rectification
# --------------------------------------------------------------------------- #
def _require_cv2():
    try:
        import cv2  # noqa: PLC0415 - optional dependency, imported on demand
    except ImportError as exc:  # pragma: no cover - depends on env
        raise ValueError("fisheye rectification needs opencv-python-headless") from exc
    return cv2


def rectify_maps(calib: FisheyeCalib, balance: float = 0.0,
                 fov_scale: float = 1.0,
                 out_size: Optional[tuple[int, int]] = None):
    """Build the fisheye -> pinhole remap tables.

    Returns ``(map1, map2, K_new)``; feed the maps straight to
    ``cv2.remap(img, map1, map2, cv2.INTER_LINEAR)``. The maps are `CV_16SC2`
    (fixed-point), which is the fast path in `remap`.

    `balance` interpolates the new focal length between the tightest and the
    widest fit of the source FOV:

    * ``0.0`` — largest focal, crops to the all-valid region: **no black
      borders**, which is what VO wants.
    * ``1.0`` — smallest focal, keeps the whole FOV and leaves black corners.

    `fov_scale` further divides the focal length (``> 1`` widens the view),
    and `out_size` overrides the output resolution (defaults to the calibrated
    one).
    """
    cv2 = _require_cv2()
    if not 0.0 <= balance <= 1.0:
        raise ValueError(f"balance must be in [0, 1], got {balance}")
    if fov_scale <= 0.0:
        raise ValueError(f"fov_scale must be > 0, got {fov_scale}")
    size = tuple(int(v) for v in (out_size or calib.size))
    eye = np.eye(3, dtype=np.float64)
    k_new = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
        calib.K, calib.D, calib.size, eye,
        balance=float(balance), new_size=size, fov_scale=float(fov_scale),
    )
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(
        calib.K, calib.D, eye, k_new, size, cv2.CV_16SC2
    )
    return map1, map2, np.asarray(k_new, dtype=np.float64)


def scale_intrinsics(k: np.ndarray, src_size: Sequence[int],
                     dst_size: Sequence[int]) -> np.ndarray:
    """Rescale a pinhole `K` for an image resized from `src_size` to `dst_size`.

    Uses the half-pixel-correct mapping `x_dst = s * (x_src + 0.5) - 0.5`, which
    is what `cv2.resize` actually implements — the naive `K' = S K` is off by
    half a pixel per axis.
    """
    sx = float(dst_size[0]) / float(src_size[0])
    sy = float(dst_size[1]) / float(src_size[1])
    out = np.array(k, dtype=np.float64, copy=True)
    out[0, 0] *= sx
    out[1, 1] *= sy
    out[0, 2] = sx * (out[0, 2] + 0.5) - 0.5
    out[1, 2] = sy * (out[1, 2] + 0.5) - 0.5
    return out


def invalid_pixel_fraction(calib: FisheyeCalib, map1, map2) -> float:
    """Fraction (0..1) of rectified pixels that sample outside the source image.

    `cv2.fisheye.estimateNewCameraMatrixForUndistortRectify` fits the new focal
    length through the **four edge midpoints only**, so `balance = 0` guarantees
    a black-free border only when the principal point is centred. This camera's
    is 114 px off centre, which leaves a thin residual arc; measuring it is the
    only way to know. Costs one remap of a white frame.
    """
    cv2 = _require_cv2()
    white = np.full((calib.height, calib.width), 255, dtype=np.uint8)
    out = cv2.remap(white, map1, map2, cv2.INTER_LINEAR)
    return float((out == 0).mean())


def pinhole_fov_deg(k: np.ndarray, size: Sequence[int]) -> tuple[float, float]:
    """Horizontal and vertical field of view (degrees) of a pinhole `K`.

    The principal point is off-centre on this camera, so each axis is the sum of
    the two half-angles rather than `2 * atan(w / 2f)`.
    """
    width, height = int(size[0]), int(size[1])
    fx, fy = float(k[0, 0]), float(k[1, 1])
    cx, cy = float(k[0, 2]), float(k[1, 2])
    hfov = math.degrees(math.atan(cx / fx) + math.atan((width - cx) / fx))
    vfov = math.degrees(math.atan(cy / fy) + math.atan((height - cy) / fy))
    return hfov, vfov


def camera_block_yaml(k_new: np.ndarray, size: Sequence[int], camera_id: str,
                      calib: FisheyeCalib, balance: float, fov_scale: float,
                      invalid_fraction: Optional[float] = None) -> str:
    """Render the rectified intrinsics as a uavloc `Camera:` block.

    The frames this describes are already rectified, so every distortion
    coefficient is zero — `undistort_image` must stay off (or be a no-op).
    """
    width, height = int(size[0]), int(size[1])
    hfov, vfov = pinhole_fov_deg(k_new, (width, height))
    today = datetime.date.today().isoformat()
    border = (
        "" if invalid_fraction is None else
        f"# Black border       : {100.0 * invalid_fraction:.4f} % of the frame\n"
    )
    return (
        "# uavloc Camera block for the RECTIFIED stream — paste into the mission YAML.\n"
        "#\n"
        f"# Source calibration : {calib.source}\n"
        f"#                      {calib.describe()}\n"
        f"# Rectified with     : bag_to_video.py --rectify "
        f"--rectify-balance {balance:g} --rectify-fov-scale {fov_scale:g}\n"
        f"# Generated          : {today}\n"
        "#\n"
        "# The frames are already rectified to this pinhole model, so k1..k3/p1/p2\n"
        "# are all zero. Do NOT use the raw fisheye numbers above with these frames.\n"
        f"# Field of view      : {hfov:.2f} deg horizontal, {vfov:.2f} deg vertical\n"
        f"{border}"
        "Camera:\n"
        f"  camera_id: {camera_id}\n"
        f"  width:  {width}\n"
        f"  height: {height}\n"
        f"  fx: {k_new[0, 0]:.4f}\n"
        f"  fy: {k_new[1, 1]:.4f}\n"
        f"  cx: {k_new[0, 2]:.4f}\n"
        f"  cy: {k_new[1, 2]:.4f}\n"
        "  k1: 0.0\n"
        "  k2: 0.0\n"
        "  p1: 0.0\n"
        "  p2: 0.0\n"
        "  k3: 0.0\n"
        f"  fov: {hfov:.2f}\n"
    )
