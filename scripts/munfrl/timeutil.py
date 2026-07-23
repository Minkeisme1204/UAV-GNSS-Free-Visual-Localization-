"""Time-base helpers for the MUN-FRL dataset.

Two clocks appear in the dataset and they are NOT the same:

  * The ROS bag (message header stamps, bag log times) and the NMEA sentences
    are on **UTC**.
  * The RTKLIB `.pos` PPK solution is stamped in **GPS time (GPST)**.

GPST does not observe leap seconds, so it runs ahead of UTC by the accumulated
leap-second count. Verified on `bell412_dataset6`: the first NMEA GGA reads
`213512.40` UTC while the first `.pos` row reads `2022/05/25 21:35:30.400`
GPST, and the bag/`.pos` spans differ by exactly 18 s.

    t_utc = t_gpst - GPS_UTC_LEAP_SECONDS
"""

from __future__ import annotations

import calendar
import datetime as _dt
import re

# GPST - UTC offset in seconds. The IERS leap-second table has been at 18 s
# since 2017-01-01; this dataset was recorded 2022-05-25, so 18 s is correct.
# If you decode a dataset recorded after a future leap second, bump this.
GPS_UTC_LEAP_SECONDS = 18.0

_DATE_RE = re.compile(r"^\s*(\d{4})[/-](\d{1,2})[/-](\d{1,2})\s*$")
_TIME_RE = re.compile(r"^\s*(\d{1,2}):(\d{1,2}):(\d{1,2}(?:\.\d+)?)\s*$")


def gpst_str_to_utc_epoch(date_str: str, time_str: str) -> float:
    """Convert an RTKLIB ``YYYY/MM/DD`` + ``HH:MM:SS.sss`` GPST pair to a UTC
    POSIX epoch in seconds (float).

    Raises ValueError when either token does not match the expected shape.
    """
    return gpst_str_to_gpst_epoch(date_str, time_str) - GPS_UTC_LEAP_SECONDS


def gpst_str_to_gpst_epoch(date_str: str, time_str: str) -> float:
    """Same as :func:`gpst_str_to_utc_epoch` but keeps the GPST time base.

    The returned number is "seconds since 1970-01-01 00:00:00 **GPST**", i.e.
    the calendar fields interpreted as if they were UTC. It is only meaningful
    as a companion column next to the true UTC epoch.
    """
    dm = _DATE_RE.match(date_str)
    tm = _TIME_RE.match(time_str)
    if not dm or not tm:
        raise ValueError(f"unparsable GPST timestamp: {date_str!r} {time_str!r}")
    year, month, day = (int(x) for x in dm.groups())
    hour, minute = int(tm.group(1)), int(tm.group(2))
    second = float(tm.group(3))
    whole = int(second)
    frac = second - whole
    base = calendar.timegm((year, month, day, hour, minute, whole, 0, 0, 0))
    return float(base) + frac


def ros_stamp_to_epoch(stamp) -> float:
    """Convert a ROS ``builtin_interfaces/Time``-like object to POSIX seconds.

    Accepts anything exposing ``sec``/``nanosec`` (rosbags) or ``secs``/``nsecs``
    (classic rospy naming), or a plain ``(sec, nsec)`` pair.
    """
    if isinstance(stamp, (tuple, list)) and len(stamp) == 2:
        return float(stamp[0]) + float(stamp[1]) * 1e-9
    sec = getattr(stamp, "sec", None)
    if sec is None:
        sec = getattr(stamp, "secs")
    nsec = getattr(stamp, "nanosec", None)
    if nsec is None:
        nsec = getattr(stamp, "nsecs", 0)
    return float(sec) + float(nsec) * 1e-9


def ns_to_epoch(t_ns: int) -> float:
    """Bag log time (integer nanoseconds) -> POSIX seconds."""
    return float(t_ns) * 1e-9


def epoch_to_msec(t_epoch: float) -> float:
    """POSIX seconds -> milliseconds, rounded to 3 decimals (microsecond grain)."""
    return round(float(t_epoch) * 1000.0, 3)


def format_msec(t_epoch: float) -> str:
    """POSIX seconds -> a fixed-precision millisecond string for CSV output."""
    return f"{float(t_epoch) * 1000.0:.3f}"


def epoch_to_utc_string(t_epoch: float) -> str:
    """POSIX seconds -> ``YYYY/MM/DD HH:MM:SS.sss`` UTC, for human-readable logs."""
    dt = _dt.datetime.fromtimestamp(float(t_epoch), tz=_dt.timezone.utc)
    return dt.strftime("%Y/%m/%d %H:%M:%S.") + f"{dt.microsecond // 1000:03d}"


def nmea_utc_to_epoch(hhmmss: str, date_epoch_hint: float) -> float:
    """Convert an NMEA ``hhmmss.ss`` UTC time-of-day to a full POSIX epoch.

    NMEA GGA carries no date, so the calendar day is taken from
    ``date_epoch_hint`` (typically the message's bag log time). A +/- 12 h
    day-rollover correction is applied so a sentence recorded just after
    midnight is not thrown a day off.
    """
    if not hhmmss:
        raise ValueError("empty NMEA time field")
    hour = int(hhmmss[0:2])
    minute = int(hhmmss[2:4])
    second = float(hhmmss[4:])
    day_start = float(int(date_epoch_hint // 86400) * 86400)
    t = day_start + hour * 3600 + minute * 60 + second
    if t - date_epoch_hint > 43200.0:
        t -= 86400.0
    elif date_epoch_hint - t > 43200.0:
        t += 86400.0
    return t
