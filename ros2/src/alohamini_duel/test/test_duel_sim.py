import json
import math
from dataclasses import replace
from pathlib import Path

import pytest
import yaml
from alohamini.schema import BodyVelocity
from alohamini_duel.arena import BASE_RADIUS, ROBOT_IDS, Arena
from alohamini_duel.cmd_vel_arbiter import STOP, VelocityArbiter
from alohamini_duel.peer_sync import PeerLink
from alohamini_duel.referee_node import Referee
from alohamini_duel.shared_sim_world import SharedDuelWorld

A, B = ROBOT_IDS
OPEN_STARTS = {A: (1.0, 2.0, 0.0), B: (2.4, 2.0, math.pi)}
SCORING_STARTS = {A: (1.0, 2.0, 0.0), B: (2.0, 2.0, math.pi)}
CONFIG = Path(__file__).parents[1] / "config"


def report(referee, robot, seq, now, pose, *, online=True, session="test"):
    referee.observe(
        robot,
        json.dumps({
            "robot_id": robot,
            "arena_id": referee.arena.fingerprint,
            "session_id": session,
            "seq": seq,
            "online": online,
            "pose": pose,
        }),
        now,
    )


@pytest.fixture
def arena():
    return Arena((0.0, 0.0, 0.0), (4.0, 4.0), (), OPEN_STARTS, "示例场地")


def front_range(world):
    return world.vehicles[0].scan((0, 0, 0), (0, 0, 0), count=361)[180]


def test_packaged_map_matches_example_walls_and_boxes():
    arena = Arena.load(CONFIG / "arena_example.yaml", CONFIG / "duel_arena_map.yaml")
    config = yaml.safe_load((CONFIG / "duel_arena_map.yaml").read_text())
    _, dimensions, _, pixels = (CONFIG / config["image"]).read_bytes().split(b"\n", 3)
    width, height = map(int, dimensions.split())
    assert len(pixels) == width * height

    def occupied(x, y):
        column = int((x - config["origin"][0]) / config["resolution"])
        row = height - 1 - int((y - config["origin"][1]) / config["resolution"])
        return pixels[row * width + column] == 0

    assert occupied(-0.01, 1.5)
    assert not occupied(1.5, 1.5)
    for xmin, ymin, xmax, ymax in arena.obstacles:
        assert occupied((xmin + xmax) / 2, (ymin + ymax) / 2)
    assert len(SharedDuelWorld(arena).vehicles) == 2


def test_dynamic_lidar_returns_remain_independent_and_respect_occlusion(arena):
    world = SharedDuelWorld(arena)
    assert front_range(world) == pytest.approx(1.1)
    world.step(((0, 0, 0), (0.4, 0, 0)), 0.1)
    assert front_range(world) == pytest.approx(1.06)
    behind_box = Arena(arena.origin, arena.size, ((1.6, 1.9, 1.7, 2.1),), arena.starts, "遮挡")
    assert front_range(SharedDuelWorld(behind_box)) == pytest.approx(0.6)


def test_shared_step_rejects_simultaneous_overlap(arena):
    world = SharedDuelWorld(arena)
    for _ in range(8):
        result = world.step(((0.4, 0, 0), (0.4, 0, 0)), 0.1)
        a, b = world.vehicles
        assert math.hypot(a.x - b.x, a.y - b.y) > 2 * BASE_RADIUS
        assert (a.x + b.x) / 2 == pytest.approx(1.7)
    assert result == ((0.0, 0.0, 0), (0.0, 0.0, 0))
    assert world.pair_collisions > 0
    assert all(vehicle.geometry_collisions == 0 for vehicle in world.vehicles)


def test_world_counts_rejected_wall_and_box_motion_without_counting_stops(arena):
    world = SharedDuelWorld(replace(arena, obstacles=((1.46, 1.9, 1.6, 2.1),)))
    first, _ = world.vehicles
    world.step(((0.4, 0, 0), (0, 0, 0)), 0.1)
    assert first.x == 1.0 and first.geometry_collisions == 1
    first.x, first.y = 0.46, 0.55
    world.step(((-0.4, 0, 0), (0, 0, 0)), 0.1)
    assert first.x == 0.46 and first.geometry_collisions == 2
    world.step(((0, 0, 0), (0, 0, 0)), 0.1)
    assert first.geometry_collisions == 2 and world.pair_collisions == 0


@pytest.mark.parametrize(
    ("choice", "pose_b", "expected"),
    [
        ("right", (2.0, 2.0, math.pi), (0, 1)),
        ("left", (2.0, 2.0, math.pi), (1, 0)),
        ("right", (2.0, 2.0, math.pi / 2), (2, 0)),
        ("right", (2.2, 2.0, math.pi), (0, 0)),
        ("right", (1.0, 3.0, -math.pi / 2), (0, 0)),
    ],
)
def test_scores_virtual_round_once(arena, choice, pose_b, expected):
    referee = Referee(arena)
    for seq in range(1, 4):
        report(referee, A, seq, 0.05 * seq, SCORING_STARTS[A])
        report(referee, B, seq, 0.05 * seq, pose_b)
    referee.choose(A, "1:left", 0.2)
    referee.choose(A, "1:center", 0.21)
    referee.choose(B, f"1:{choice}", 0.22)
    referee.choose(B, f"1:{choice}", 0.23)
    assert referee.ready and referee.round == 2
    assert tuple(referee.scores.values()) == expected


def test_referee_rejects_scoring_through_box(arena):
    referee = Referee(replace(arena, obstacles=((1.48, 1.9, 1.52, 2.1),)))
    for seq in range(1, 4):
        for robot in ROBOT_IDS:
            report(referee, robot, seq, 0.05 * seq, SCORING_STARTS[robot])
    referee.choose(A, "1:left", 0.2)
    referee.choose(B, "1:left", 0.2)
    assert tuple(referee.scores.values()) == (0, 0)
    assert referee.result == "双方未进入判分条件"


def test_referee_pauses_and_invalidates_delayed_round_input(arena):
    referee = Referee(arena)
    for seq in range(1, 4):
        for robot in ROBOT_IDS:
            report(referee, robot, seq, 0.05 * seq, SCORING_STARTS[robot])
    referee.choose(A, "1:left", 0.2)
    report(referee, B, 4, 0.22, None, online=False)
    assert not referee.ready and referee.round == 2 and not referee.choices
    for seq in range(1, 4):
        report(referee, A, seq + 3, 0.25 + 0.05 * seq, SCORING_STARTS[A])
        report(referee, B, seq, 0.25 + 0.05 * seq, SCORING_STARTS[B], session="new")
        assert referee.ready == (seq == 3)
    referee.choose(B, "1:right", 0.43)
    assert not referee.choices
    report(referee, B, 99, 0.45, SCORING_STARTS[B])
    assert referee.players[B].session_id == "new"
    referee.refresh(1.0)
    assert not referee.ready and referee.round == 3


def test_cutting_heartbeat_stops_velocity_and_keeps_lidar_echo(arena):
    first, second = (PeerLink(robot, arena) for robot in ROBOT_IDS)
    referee = Referee(arena)
    world = SharedDuelWorld(arena)
    arbiter = VelocityArbiter(arena)
    arbiter.enable(True)
    ranges = []
    for frame in range(21):
        now = frame * 0.05
        a, b = world.vehicles
        pose_a, pose_b = (a.x, a.y, a.yaw), (b.x, b.y, b.yaw)
        sent_a = first.encode(pose_a, 0.0, 0.0, now)
        sent_b = second.encode(pose_b, 0.0, 0.0, now)
        if frame < 8:
            first.accept(sent_b, now)
            second.accept(sent_a, now)
        first_online, second_online = first.online(now), second.online(now)
        report(referee, A, frame + 1, now, pose_a, online=first_online is not None)
        report(referee, B, frame + 1, now, pose_b, online=second_online is not None)
        arbiter.submit("duel", BodyVelocity(0.1, 0.0, 0.0), now)
        command = arbiter.select(pose_a, first_online, now)
        world.step(((command.x_m_s, command.y_m_s, command.yaw_rad_s), (0.0, 0.0, 0.0)), 0.05)
        ranges.append(front_range(world))
        if frame == 7:
            assert referee.ready and command.x_m_s > 0.0
    assert command == STOP and not referee.ready
    assert ranges[-1] == ranges[-2]
    assert ranges[-1] < ranges[0] < 2.0
