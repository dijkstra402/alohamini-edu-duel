"""展开实际启动图检查底盘联调链路。"""

import importlib.util
from pathlib import Path

import pytest


@pytest.mark.parametrize(
    ("simulation", "referee_status"),
    [(False, None), (False, "true"), (True, None)],
)
def test_vehicle_graph_keeps_one_speed_chain(tmp_path, simulation, referee_status):
    pytest.importorskip("rclpy", reason="需要项目的 ROS 2 运行环境")
    spec = importlib.util.spec_from_file_location(
        "duel_launch_graph",
        Path(__file__).parents[2] / "alohamini_bringup/test/test_launch_graph.py",
    )
    helpers = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helpers)
    map_file = tmp_path / "map.yaml"
    map_file.write_text("image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n")
    (tmp_path / "map.pgm").write_bytes(b"P5\n1 1\n255\n\0")
    launch_options = {"referee_status": referee_status} if referee_status is not None else {}
    nodes, processes = helpers.launch_graph(
        "alohamini_duel",
        "duel_vehicle_sim.launch.py" if simulation else "duel_hw.launch.py",
        map=str(map_file),
        arena=str(Path(__file__).parents[1] / "config/arena_example.yaml"),
        host="192.0.2.10",
        peer_ip="192.0.2.20",
        robot_id="shield_b",
        **launch_options,
    )
    assert not processes
    packages = [package for package, _, _ in nodes]
    for package in ("nav2_collision_monitor", "nav2_amcl", "alohamini_duel"):
        assert packages.count(package) == 1
    assert "nav2_controller" not in packages
    assert "nav2_velocity_smoother" not in packages
    by_executable = {executable: params for _, executable, params in nodes}
    if simulation:
        assert "alohamini_bridge" not in packages
        assert "alohamini_rplidar_c1" not in packages and "alohamini_imu" not in packages
        assert all(params["use_sim_time"] for _, _, params in nodes)
    if not simulation:
        assert packages.count("alohamini_bridge") == 1
        assert by_executable["bridge_node"]["base_only"] is True
        assert by_executable["bridge_node"]["cmd_vel_topic"] == "/cmd_vel_safe"
    assert by_executable["collision_monitor"]["cmd_vel_in_topic"] == "/cmd_vel"
    assert by_executable["collision_monitor"]["cmd_vel_out_topic"] == "/cmd_vel_safe"
    assert by_executable["amcl"]["set_initial_pose"] is True
    assert by_executable["amcl"]["laser_model_type"] == "likelihood_field_prob"
    assert by_executable["amcl"]["do_beamskip"] is True
    assert by_executable["amcl"]["initial_pose.x"] == 2.45
    assert by_executable["duel_base"]["peer_port"] == 5588
    assert by_executable["duel_base"]["simulation_controls"] is simulation
    assert by_executable["duel_base"]["referee_status"] is (simulation or referee_status == "true")
