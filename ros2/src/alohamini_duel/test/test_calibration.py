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


def test_calibrate_rejects_start_outside_measured_wall_clearance(measurements, tmp_path):
    output = tmp_path / "arena.yaml"
    measurements["wall_corners"][3][0] = 0.05
    measurements["start_poses"]["sword_a"] = [0.477, 2.50, 0]
    with pytest.raises(ValueError, match="净空"):
        calibrate(measurements, MAP, output, 0.03)
    assert not output.exists()
    measurements["start_poses"]["sword_a"][0] = 0.55
    assert calibrate(measurements, MAP, output, 0.03).origin[0] == pytest.approx(0.05)


def test_calibrate_rejects_repeated_corners(measurements, tmp_path):
    output = tmp_path / "arena.yaml"
    measurements["obstacle_corners"] = [[[1, 1], [1, 1], [1.03, 1.05], [1, 1.05]]]
    with pytest.raises(ValueError, match="重合角点"):
        calibrate(measurements, MAP, output, 0.03)
    assert not output.exists()


def test_calibrate_rejects_map_without_grid_metadata(measurements, tmp_path):
    output = tmp_path / "arena.yaml"
    incomplete_map = tmp_path / "map.yaml"
    incomplete_map.write_text(yaml.safe_dump({"image": str(MAP.with_suffix(".pgm"))}))
    with pytest.raises(ValueError, match="地图栅格参数"):
        calibrate(measurements, incomplete_map, output, 0.03)
    assert not output.exists()
