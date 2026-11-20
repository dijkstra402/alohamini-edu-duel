"""将一键验收的双车绕箱测点生成共享仿真地图和预设路线。"""

from pathlib import Path

import yaml

from .arena import BASE_RADIUS, ROBOT_IDS
from .calibrate import calibrate
from .route import load_route


def prepare_round(example: Path, output: Path):
    data = yaml.safe_load(example.read_bytes())
    size = data["wall_corners"][2]
    resolution = 0.025
    width, height = (round(axis / resolution) + 2 for axis in size)
    pixels = bytearray()
    boxes = [
        (corner[0][0], corner[0][1], corner[2][0], corner[2][1])
        for corner in data["obstacle_corners"]
    ]
    for row in reversed(range(height)):
        y = (row - 0.5) * resolution
        for column in range(width):
            x = (column - 0.5) * resolution
            wall = not (0 <= x <= size[0] and 0 <= y <= size[1])
            box = any(x0 <= x <= x1 and y0 <= y <= y1 for x0, y0, x1, y1 in boxes)
            pixels.append(0 if wall or box else 255)
    (output / "round_map.pgm").write_bytes(f"P5\n{width} {height}\n255\n".encode() + pixels)
    map_file = output / "round_map.yaml"
    map_file.write_text(
        yaml.safe_dump(dict(
            image="round_map.pgm", mode="trinary", resolution=resolution,
            origin=[-resolution, -resolution, 0.0], negate=0,
            occupied_thresh=0.65, free_thresh=0.196,
        )),
        encoding="utf-8",
    )
    arena_file = output / "round_arena.yaml"
    arena = calibrate(data, map_file, arena_file, 0.03)
    routes = {}
    for robot in ROBOT_IDS:
        route = output / f"round_{robot}.yaml"
        route.write_text(yaml.safe_dump(data["routes"][robot]), encoding="utf-8")
        path = (arena.starts[robot], *load_route(route, arena))
        if arena.path_clear(path[0][:2], path[-1][:2], BASE_RADIUS):
            raise ValueError(f"{robot} 的起点和终点之间需要有箱体阻隔")
        if not all(arena.path_clear(a[:2], b[:2], BASE_RADIUS) for a, b in zip(path, path[1:])):
            raise ValueError(f"{robot} 的预设绕箱路线净空不足")
        routes[robot] = route
    return map_file, arena_file, arena, routes
