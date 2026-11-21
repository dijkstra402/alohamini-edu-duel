"""展开实际启动图检查底盘联调链路。"""

import importlib.util
from pathlib import Path

import pytest
from alohamini_duel.cmd_vel_arbiter import COMMAND_TIMEOUT


@pytest.mark.parametrize(
    ("simulation", "referee_status", "use_nav2"),
    [(False, None, False), (False, "true", False), (True, None, False), (True, None, True)],
)
def test_vehicle_graph_keeps_one_speed_chain(tmp_path, simulation, referee_status, use_nav2):
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
    details = []
    nodes, processes = helpers.launch_graph(
        "alohamini_duel",
        "duel_vehicle_sim.launch.py" if simulation else "duel_hw.launch.py",
        map=str(map_file),
        arena=str(Path(__file__).parents[1] / "config/arena_example.yaml"),
        host="192.0.2.10",
        peer_ip="192.0.2.20",
        robot_id="shield_b",
        use_nav2=str(use_nav2).lower(),
        details=details,
        **launch_options,
    )
    assert not processes
    packages = [package for package, _, _ in nodes]
    for package in ("nav2_collision_monitor", "nav2_amcl", "alohamini_duel"):
        assert packages.count(package) == 1
    assert ("nav2_controller" in packages) is use_nav2
    assert ("nav2_velocity_smoother" in packages) is use_nav2
    assert "nav2_waypoint_follower" not in packages
    if use_nav2:
        smoother = next(item for item in details if item["package"] == "nav2_velocity_smoother")
        assert smoother["remappings"]["cmd_vel_smoothed"] == "/cmd_vel_nav_out"
        standalone_details = []
        standalone, _ = helpers.launch_graph(
            "alohamini_navigation", "navigation.launch.py",
            map=str(map_file), details=standalone_details,
        )
        assert any(package == "nav2_waypoint_follower" for package, _, _ in standalone)
        standalone_smoother = next(
            item for item in standalone_details if item["package"] == "nav2_velocity_smoother"
        )
        assert standalone_smoother["remappings"]["cmd_vel_smoothed"] == "cmd_vel"
    by_executable = {executable: params for _, executable, params in nodes}
    if simulation:
        assert "alohamini_bridge" not in packages
        assert "alohamini_rplidar_c1" not in packages and "alohamini_imu" not in packages
        assert all(params["use_sim_time"] for _, _, params in nodes)
    if not simulation:
        assert packages.count("alohamini_bridge") == 1
        assert by_executable["bridge_node"]["base_only"] is True
        assert by_executable["bridge_node"]["cmd_vel_topic"] == "/cmd_vel_safe"
        assert by_executable["bridge_node"]["command_timeout_sec"] == COMMAND_TIMEOUT == 0.15
        if referee_status is None:
            standalone, _ = helpers.launch_graph("alohamini_navigation", "runtime.launch.py")
            assert next(params for _, node, params in standalone if node == "bridge_node")[
                "command_timeout_sec"
            ] == 0.5
            params_file = tmp_path / "bridge.yaml"
            params_file.write_text(
                "alohamini_lerobot_bridge:\n  ros__parameters:\n    command_timeout_sec: 0.4\n"
            )
            custom, _ = helpers.launch_graph(
                "alohamini_bridge", "bridge.launch.py", params_file=str(params_file)
            )
            assert custom[0][2]["command_timeout_sec"] == 0.4
    assert by_executable["collision_monitor"]["cmd_vel_in_topic"] == "/cmd_vel"
    assert by_executable["collision_monitor"]["cmd_vel_out_topic"] == "/cmd_vel_safe"
    assert by_executable["amcl"]["set_initial_pose"] is True
    assert by_executable["amcl"]["laser_model_type"] == "likelihood_field_prob"
    assert by_executable["amcl"]["do_beamskip"] is True
    assert by_executable["amcl"]["initial_pose.x"] == 2.45
    assert by_executable["duel_base"]["peer_port"] == 5588
    assert by_executable["duel_base"]["simulation_controls"] is simulation
    assert by_executable["duel_base"]["referee_status"] is (simulation or referee_status == "true")
