"""场地标定校验和失败后重试。"""

import math
from pathlib import Path

import pytest
import yaml

from alohamini_duel.arena import Arena
from alohamini_duel.calibrate import calibrate

CONFIG = Path(__file__).parents[1] / "config"
MAP = CONFIG / "duel_arena_map.yaml"


@pytest.fixture
def measurements():
    return yaml.safe_load((CONFIG / "measurements_example.yaml").read_bytes())


def test_calibrate_rotated_arena(measurements, tmp_path):
    def rotate(point):
        return [4 - point[1], -3 + point[0]]

    measurements["wall_corners"] = [rotate(point) for point in measurements["wall_corners"]]
    measurements["obstacle_corners"] = [
        [rotate(point) for point in box] for box in measurements["obstacle_corners"]
    ]
    measurements["start_poses"] = {
        name: [*rotate(pose), pose[2] + math.pi / 2]
        for name, pose in measurements["start_poses"].items()
    }
    output = tmp_path / "arena.yaml"
    arena = calibrate(measurements, MAP, output, 0.03)
    assert arena == Arena.load(output, MAP)
    assert arena.origin == pytest.approx((4, -3, math.pi / 2))
    assert arena.size == pytest.approx((3, 3))
    assert arena.obstacles[0] == pytest.approx((0.625, 1.725, 0.875, 1.975))
    assert arena.starts["sword_a"] == pytest.approx((3.45, -2.45, 3 * math.pi / 4))


@pytest.mark.parametrize(
    ("pose", "message"),
    [
        ([0.45, 0.55, 0], "净空"),
        ([0.9, 1.8, 0], "净空"),
        ([1.1, 1.1, 0], "重叠"),
    ],
)
def test_calibrate_rejects_start_and_allows_retry(measurements, tmp_path, pose, message):
    output = tmp_path / "arena.yaml"
    original = measurements["start_poses"]["shield_b"]
    measurements["start_poses"]["shield_b"] = pose
    with pytest.raises(ValueError, match=message):
        calibrate(measurements, MAP, output, 0.03)
    assert not output.exists()
    measurements["start_poses"]["shield_b"] = original
    calibrate(measurements, MAP, output, 0.03)
    written = output.read_bytes()
    with pytest.raises(FileExistsError):
        calibrate(measurements, MAP, output, 0.03)
    assert output.read_bytes() == written


def test_calibrate_checks_corner_order_and_measurement_tolerance(measurements, tmp_path):
    output = tmp_path / "arena.yaml"
    wall = measurements["wall_corners"]
    wall[2], wall[3] = wall[3], wall[2]
    with pytest.raises(ValueError, match="角点顺序"):
        calibrate(measurements, MAP, output, 0.03)
    wall[2], wall[3] = wall[3], wall[2]
    wall[2][0] += 0.10
    with pytest.raises(ValueError, match="矩形误差"):
        calibrate(measurements, MAP, output, 0.03)
    assert not output.exists()
    calibrate(measurements, MAP, output, 0.08)
