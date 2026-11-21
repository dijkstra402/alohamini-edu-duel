"""场地标定与底盘扫掠区域计算。"""

import hashlib
import math
from dataclasses import dataclass
from pathlib import Path

import yaml
from alohamini._validation import finite_number
from alohamini_navigation.maps import validate_map_revision

Point = tuple[float, float]
Pose = tuple[float, float, float]
Rectangle = tuple[float, float, float, float]
ROBOT_IDS = ("sword_a", "shield_b")
# 按名义收拢方形底盘的外接圆加余量。实测时须核对底盘包络和定位误差。
BASE_RADIUS = 0.45


def coordinates(values, length, name):
    if not isinstance(values, (list, tuple)) or len(values) != length:
        raise ValueError(f"{name}必须包含 {length} 个数值")
    for value in values:
        finite_number(value, name)
    return tuple(values)


def local_point(origin: Pose, point: Point) -> Point:
    dx, dy = point[0] - origin[0], point[1] - origin[1]
    c, s = math.cos(origin[2]), math.sin(origin[2])
    return dx * c + dy * s, -dx * s + dy * c


def segment_distance(start: Point, end: Point, point: Point) -> float:
    dx, dy = end[0] - start[0], end[1] - start[1]
    px, py = point[0] - start[0], point[1] - start[1]
    length_squared = dx * dx + dy * dy
    if length_squared == 0:
        return math.hypot(px, py)
    fraction = min(1.0, max(0.0, (dx * px + dy * py) / length_squared))
    return math.hypot(px - fraction * dx, py - fraction * dy)


def intersects_box(start: Point, end: Point, box: Rectangle, margin: float) -> bool:
    enter, leave = 0.0, 1.0
    for axis in range(2):
        low, high = box[axis] - margin, box[axis + 2] + margin
        delta = end[axis] - start[axis]
        if delta == 0:
            if not low <= start[axis] <= high:
                return False
            continue
        crossings = sorted(((low - start[axis]) / delta, (high - start[axis]) / delta))
        enter, leave = max(enter, crossings[0]), min(leave, crossings[1])
        if enter > leave:
            return False
    return True


@dataclass(frozen=True)
class Arena:
    origin: Pose
    size: Point
    obstacles: tuple[Rectangle, ...]
    starts: dict[str, Pose]
    fingerprint: str

    @classmethod
    def load(cls, arena_file, map_file):
        map_path = Path(map_file)
        validate_map_revision(map_path, "alohamini2edu")
        arena_bytes = Path(arena_file).read_bytes()
        data = yaml.safe_load(arena_bytes)
        if not isinstance(data, dict) or data.get("schema_version") != 1:
            raise ValueError("场地标定文件必须声明 schema_version: 1")
        origin = coordinates(data.get("map_origin"), 3, "场地原点及朝向")
        size = coordinates(data.get("size"), 2, "场地尺寸")
        if min(size) <= 2 * BASE_RADIUS:
            raise ValueError("场地尺寸必须大于底盘直径")
        boxes, positions = data.get("obstacles"), data.get("starts")
        if not isinstance(boxes, list) or not isinstance(positions, dict):
            raise ValueError("标定文件必须提供箱体边界列表和双车起点")
        obstacles = tuple(coordinates(box, 4, "箱体边界") for box in boxes)
        for xmin, ymin, xmax, ymax in obstacles:
            if not (0 <= xmin < xmax <= size[0] and 0 <= ymin < ymax <= size[1]):
                raise ValueError("箱体边界必须位于场地内且面积为正")
        starts = {
            robot_id: coordinates(positions.get(robot_id), 3, f"{robot_id} 起点")
            for robot_id in ROBOT_IDS
        }
        map_bytes = map_path.read_bytes()
        map_data = yaml.safe_load(map_bytes)
        image = map_data.get("image") if isinstance(map_data, dict) else None
        if not isinstance(image, str) or not image:
            raise ValueError("地图 YAML 必须指定栅格图像")
        image_path = map_path.parent / image
        digest = hashlib.sha256()
        for content in (arena_bytes, map_bytes, image_path.read_bytes()):
            digest.update(hashlib.sha256(content).digest())
        arena = cls(origin, size, obstacles, starts, digest.hexdigest())
        for x, y, yaw in starts.values():
            if abs(yaw) > math.pi or not arena.path_clear((x, y), (x, y), BASE_RADIUS):
                raise ValueError("起点的朝向或底盘净空不符合要求")
        if math.dist(starts[ROBOT_IDS[0]][:2], starts[ROBOT_IDS[1]][:2]) <= 2 * BASE_RADIUS:
            raise ValueError("两个起点的底盘保持区重叠")
        return arena

    def local(self, point: Point) -> Point:
        return local_point(self.origin, point)

    def contains(self, point: Point) -> bool:
        x, y = self.local(point)
        return 0 <= x <= self.size[0] and 0 <= y <= self.size[1]

    def path_clear(self, start: Point, end: Point, margin: float) -> bool:
        start, end = self.local(start), self.local(end)
        return (
            all(
                margin < point[0] < self.size[0] - margin
                and margin < point[1] < self.size[1] - margin
                for point in (start, end)
            )
            and not any(intersects_box(start, end, box, margin) for box in self.obstacles)
        )
