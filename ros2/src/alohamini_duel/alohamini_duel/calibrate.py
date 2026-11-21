"""将平面现场测点转换为双车共用的场地配置。"""

import argparse
import math
import os
import sys
from pathlib import Path
from tempfile import TemporaryDirectory

import yaml

from .arena import ROBOT_IDS, Arena, coordinates, local_point


def fit_rectangle(points, tolerance, name):
    if not isinstance(points, list) or len(points) != 4:
        raise ValueError(f"{name}需要按逆时针填写四个角点")
    corners = tuple(coordinates(point, 2, name) for point in points)
    edges = tuple(
        (b[0] - a[0], b[1] - a[1])
        for a, b in zip(corners, corners[1:] + corners[:1], strict=True)
    )
    if any(
        a[0] * b[1] - a[1] * b[0] <= 0
        for a, b in zip(edges, edges[1:] + edges[:1], strict=True)
    ):
        raise ValueError(f"{name}的角点顺序有误或存在重合角点")
    first, second, *_ = corners
    yaw = math.atan2(second[1] - first[1], second[0] - first[0])
    provisional = (*first, yaw)
    projected = tuple(local_point(provisional, point) for point in corners)
    lower_left, lower_right, upper_right, upper_left = projected
    left = (lower_left[0] + upper_left[0]) / 2
    right = (lower_right[0] + upper_right[0]) / 2
    bottom = (lower_left[1] + lower_right[1]) / 2
    top = (upper_left[1] + upper_right[1]) / 2
    expected = ((left, bottom), (right, bottom), (right, top), (left, top))
    if right <= left or top <= bottom or any(
        math.dist(point, target) > tolerance
        for point, target in zip(projected, expected, strict=True)
    ):
        raise ValueError(f"{name}的角点顺序或矩形误差不符合要求")
    # 墙边按测点取内侧包络，拟合误差不能扩大可通行区域。
    left, right = max(lower_left[0], upper_left[0]), min(lower_right[0], upper_right[0])
    bottom, top = max(lower_left[1], lower_right[1]), min(upper_left[1], upper_right[1])
    c, s = math.cos(yaw), math.sin(yaw)
    origin = (first[0] + left * c - bottom * s, first[1] + left * s + bottom * c, yaw)
    return origin, (right - left, top - bottom), corners


def calibrate(measurements, map_file, output, tolerance):
    if not isinstance(measurements, dict) or measurements.get("frame") != "map":
        raise ValueError("测点文件必须声明 frame: map")
    origin, size, _ = fit_rectangle(measurements.get("wall_corners"), tolerance, "场地墙角")
    obstacles, poses = measurements.get("obstacle_corners"), measurements.get("start_poses")
    if not isinstance(obstacles, list) or not isinstance(poses, dict):
        raise ValueError("测点文件必须提供障碍物角点和双车起点")
    boundaries = []
    for number, points in enumerate(obstacles, 1):
        _, _, corners = fit_rectangle(points, tolerance, f"{number} 号箱")
        xs, ys = zip(*(local_point(origin, point) for point in corners), strict=True)
        boundaries.append([min(xs), min(ys), max(xs), max(ys)])
    starts = {
        robot: list(coordinates(poses.get(robot), 3, f"{robot} 起点")) for robot in ROBOT_IDS
    }
    config = {
        "schema_version": 1,
        "map_origin": list(origin),
        "size": list(size),
        "obstacles": boundaries,
        "starts": starts,
    }
    map_data = yaml.safe_load(Path(map_file).read_bytes())
    if not isinstance(map_data, dict):
        raise ValueError("地图 YAML 必须包含栅格参数")
    resolution, occupied, free, negate = coordinates(
        [map_data.get(key) for key in ("resolution", "occupied_thresh", "free_thresh", "negate")],
        4,
        "地图栅格参数",
    )
    coordinates(map_data.get("origin"), 3, "地图原点")
    if resolution <= 0 or not 0 <= free < occupied <= 1 or negate not in (0, 1):
        raise ValueError("地图分辨率、阈值或灰度反转参数无效")
    output = Path(output)
    with TemporaryDirectory(dir=output.parent) as directory:
        staging = Path(directory) / "arena.yaml"
        staging.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")
        arena = Arena.load(staging, map_file)
        os.link(staging, output)
    return arena


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--measurements", type=Path, required=True)
    parser.add_argument("--map", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--tolerance", type=float, default=0.03, help="矩形测点容差（米）")
    args = parser.parse_args(argv)
    if not math.isfinite(args.tolerance) or args.tolerance <= 0:
        parser.error("测点容差必须为有限正数")
    try:
        arena = calibrate(
            yaml.safe_load(args.measurements.read_bytes()), args.map, args.output, args.tolerance
        )
    except FileExistsError:
        print(f"输出已存在：{args.output}", file=sys.stderr)
        return 1
    except (OSError, ValueError, yaml.YAMLError) as error:
        print(f"场地标定失败：{error}", file=sys.stderr)
        return 1
    print(f"已生成 {args.output}\n场地指纹 {arena.fingerprint}")
    return 0
