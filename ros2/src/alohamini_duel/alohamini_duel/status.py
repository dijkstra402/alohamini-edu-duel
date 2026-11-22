"""车内状态消息供裁判和只读预检共同读取。"""

import json
import math
from dataclasses import dataclass

from .arena import Arena, Pose, coordinates

MAX_POSE_AGE = 0.35
MAX_SCAN_AGE = 0.30
SCAN_TIMEOUT = 0.21


def fresh_stamp(header, frame, now_ns, max_age):
    stamp_ns = header.stamp.sec * 1_000_000_000 + header.stamp.nanosec
    if header.frame_id == frame and stamp_ns > 0 and -0.05 <= (now_ns - stamp_ns) / 1e9 <= max_age:
        return stamp_ns
    return None


def scan_has_valid_ranges(scan):
    return sum(
        math.isfinite(value) and scan.range_min <= value <= scan.range_max
        for value in scan.ranges
    ) > 10


def wheel_is_stationary(twist):
    return math.hypot(twist.linear.x, twist.linear.y) < 0.01 and abs(twist.angular.z) < 0.03


def yaw_from_quaternion(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def angle_error(angle, yaw):
    difference = angle - yaw
    return abs(math.atan2(math.sin(difference), math.cos(difference)))


@dataclass(frozen=True)
class VehicleStatus:
    session_id: str
    seq: int
    pose: Pose | None
    online: bool
    motion_enabled: bool | None

    @classmethod
    def decode(cls, raw: str, robot_id: str, arena: Arena):
        try:
            data = json.loads(raw)
            if data["robot_id"] != robot_id or data["arena_id"] != arena.fingerprint:
                return None
            session, seq, online = data["session_id"], data["seq"], data["online"]
            enabled = data.get("motion_enabled")
            if (
                not isinstance(session, str)
                or not 1 <= len(session) <= 64
                or type(seq) is not int
                or seq < 1
                or type(online) is not bool
                or (enabled is not None and type(enabled) is not bool)
            ):
                return None
            pose = coordinates(data["pose"], 3, "本车位姿") if data["pose"] is not None else None
        except (KeyError, TypeError, ValueError, RecursionError):
            return None
        if pose is None and online:
            return None
        if pose is not None and (not arena.contains(pose[:2]) or abs(pose[2]) > math.pi):
            return None
        return cls(session, seq, pose, online, enabled)
