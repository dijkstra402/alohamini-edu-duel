"""静态回波、临时障碍、遮挡与定位偏差。"""

import math

import pytest

from alohamini_duel.radar import StaticGrid, compare_scan


def test_radar_ignores_known_surfaces_hidden_regions_and_invalid_returns():
    grid = StaticGrid((0, 0, 0), 1, 6, 3, (
        0, 0, 0, 100, 0, 0,
        0, 0, 0, 0, -1, 0,
        0, 0, 0, 0, 0, 0,
    ))
    points, counts = compare_scan(
        grid, (0.5, 0.5, 0), [1, 2.5, 4, 6, math.inf, math.nan, 0.05, 11],
        0, 0, (0.1, 10), 0.12,
    )
    assert len(points) == 1
    assert points[0] == pytest.approx((1.5, 0.5))
    assert counts == {"background": 1, "unexpected": 1, "occluded": 1, "unmapped": 1, "invalid": 4}
    assert grid.classify((0.5, 1.5), (5.5, 1.5), 0.12) == "unmapped"
    assert grid.classify((0.5, 1.5), (4.5, 1.5), 0.12) == "unmapped"


def test_radar_uses_map_orientation_and_pose_tolerance():
    grid = StaticGrid((10, -4, math.pi / 2), 0.1, 5, 3, (
        0, 0, 0, 100, 0,
        0, 0, 0, 100, 0,
        0, 0, 0, 100, 0,
    ))
    points, counts = compare_scan(
        grid, (9.85, -3.86, math.pi / 2), [0.11], 0, 0, (0.01, 10), 0.12,
    )
    # 同一回波的真实端点在墙内，改变定位偏差检验墙面容差。
    assert not points
    assert counts["background"] == 1
    points, counts = compare_scan(
        grid, (9.85, -3.95, math.pi / 2), [0.11], 0, 0, (0.01, 10), 0.12,
    )
    assert len(points) == 1
    assert points[0] == pytest.approx((9.85, -3.84))
    assert counts["unexpected"] == 1


def test_radar_checks_cells_crossed_near_a_corner():
    grid = StaticGrid((0, 0, 0), 1, 4, 2, (0, 0, 0, 0, 0, 100, 0, 0))
    assert grid.classify((0.5, 0.16), (3.5, 1.96), 0.12) == "occluded"
    assert grid.classify((3.5, 1.96), (0.5, 0.16), 0.12) == "occluded"
