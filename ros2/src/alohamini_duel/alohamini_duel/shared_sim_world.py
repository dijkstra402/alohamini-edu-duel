"""同步步进两个底盘，并为每个激光雷达提供另一辆车的几何回波。"""

import math

from alohamini_nav_sim.world import World

from .arena import BASE_RADIUS, ROBOT_IDS, Arena, segment_distance

HALF_WIDTH = 0.30


class SharedVehicle(World):
    opponent: "SharedVehicle"

    def scan(self, xyz, rpy, count=360, maximum=8.0, *, dynamic_segments=()):
        opponent = self.opponent
        c, s = math.cos(opponent.yaw), math.sin(opponent.yaw)
        corners = [
            (
                opponent.x + HALF_WIDTH * (x * c - y * s),
                opponent.y + HALF_WIDTH * (x * s + y * c),
            )
            for x, y in ((-1, -1), (1, -1), (1, 1), (-1, 1))
        ]
        edges = list(zip(corners, corners[1:] + corners[:1], strict=True))
        return super().scan(xyz, rpy, count, maximum, dynamic_segments=(*dynamic_segments, *edges))


class SharedDuelWorld:
    def __init__(self, arena: Arena):
        self.pair_collisions = 0
        self.vehicles = tuple(
            SharedVehicle(
                {
                    "bounds": (0.0, 0.0, *arena.size),
                    "obstacles": arena.obstacles,
                    "initial_pose": (*arena.local(pose[:2]), pose[2] - arena.origin[2]),
                    "robot_radius": BASE_RADIUS,
                }
            )
            for pose in (arena.starts[robot_id] for robot_id in ROBOT_IDS)
        )
        first, second = self.vehicles
        first.opponent, second.opponent = second, first

    def step(self, commands, dt):
        first, second = self.vehicles
        previous = ((first.x, first.y), (second.x, second.y))
        feedback = tuple(
            vehicle.step(*command, dt)
            for vehicle, command in zip(self.vehicles, commands, strict=True)
        )
        start_delta = (previous[0][0] - previous[1][0], previous[0][1] - previous[1][1])
        end_delta = (first.x - second.x, first.y - second.y)
        if segment_distance(start_delta, end_delta, (0.0, 0.0)) <= 2 * BASE_RADIUS:
            self.pair_collisions += 1
            # 两车提案来源于同一时刻，检测相对扫掠后一起撤回平移。
            for vehicle, position in zip(self.vehicles, previous, strict=True):
                vehicle.x, vehicle.y = position
            return tuple((0.0, 0.0, velocity[2]) for velocity in feedback)
        return feedback
