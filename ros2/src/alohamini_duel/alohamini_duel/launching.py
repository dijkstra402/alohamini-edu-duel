"""组合单车定位、碰撞监测和底盘通信。"""

from pathlib import Path

from alohamini_navigation.launching import configured_file, include, model_config, navigation_nodes
from alohamini_navigation.maps import validate_map_revision
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile

from .arena import ROBOT_IDS, Arena
from .cmd_vel_arbiter import COMMAND_TIMEOUT


def _nodes(context, *, simulation):
    def value(key):
        return LaunchConfiguration(key).perform(context)

    robot_id = value("robot_id")
    if robot_id not in ROBOT_IDS:
        raise ValueError(f"robot_id 必须是 {ROBOT_IDS}")
    if not value("peer_ip"):
        raise ValueError("必须指定对端 PC 的地址")
    map_path = Path(value("map")).expanduser().resolve()
    arena_path = Path(value("arena")).expanduser().resolve()
    if not map_path.is_file() or map_path.suffix not in (".yaml", ".yml"):
        raise ValueError("map 必须指向双车共用的地图 YAML 文件")
    validate_map_revision(map_path, "alohamini2edu")
    arena = Arena.load(arena_path, map_path)
    x, y, yaw = arena.starts[robot_id]
    params, cleanup = configured_file(model_config("alohamini2edu", "nav2"), "nav2")
    configured = ParameterFile(params, allow_substs=True)
    sim_time = {"use_sim_time": simulation}
    nav_enabled = value("use_nav2").lower() == "true"
    return [
        cleanup,
        include(
            "alohamini_navigation",
            "runtime.launch.py",
            robot_model="alohamini2edu",
            host=value("host"),
            start_sensors="false" if simulation else value("start_sensors"),
            use_rviz=value("use_rviz"),
            base_only="true",
            runtime_mode="simulation" if simulation else "hardware",
            command_timeout_sec="" if simulation else str(COMMAND_TIMEOUT),
            use_sim_time=str(simulation).lower(),
        ),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            parameters=[configured, sim_time, {"yaml_filename": str(map_path)}],
        ),
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            parameters=[
                configured,
                sim_time,
                {
                    "set_initial_pose": True,
                    "laser_model_type": "likelihood_field_prob",
                    "do_beamskip": True,
                    "initial_pose.x": float(x),
                    "initial_pose.y": float(y),
                    "initial_pose.yaw": float(yaw),
                },
            ],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            parameters=[sim_time, {"autostart": True, "node_names": ["map_server", "amcl"]}],
        ),
        *(
            [
                Node(
                    package="alohamini_duel",
                    executable="duel_base",
                    name="alohamini_duel_base",
                    parameters=[
                        sim_time,
                        {
                            "simulation_controls": simulation,
                            "referee_status": value("referee_status").lower() == "true",
                            "robot_id": robot_id,
                            "arena": str(arena_path),
                            "map": str(map_path),
                            "bind_ip": value("bind_ip"),
                            "bind_port": int(value("bind_port")),
                            "peer_ip": value("peer_ip"),
                            "peer_port": int(value("peer_port")),
                        }
                    ],
                    output="screen",
                )
            ]
            if not simulation or value("start_base").lower() == "true"
            else []
        ),
        *(
            navigation_nodes(
                configured, simulation, cmd_vel_out="/cmd_vel_nav_out",
                waypoint_follower=False,
            )
            if nav_enabled
            else []
        ),
        *(
            [
                Node(
                    package="alohamini_duel",
                    executable="duel_radar",
                    parameters=[sim_time],
                    output="screen",
                )
            ]
            if value("radar_diagnostics").lower() == "true"
            else []
        ),
    ]


def vehicle_launch(*, simulation):
    host_argument = (
        DeclareLaunchArgument("host", default_value="127.0.0.1")
        if simulation
        else DeclareLaunchArgument("host")
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_id"),
            DeclareLaunchArgument("map"),
            DeclareLaunchArgument("arena"),
            host_argument,
            DeclareLaunchArgument("peer_ip"),
            DeclareLaunchArgument("bind_ip", default_value="0.0.0.0"),
            DeclareLaunchArgument("bind_port", default_value="5588"),
            DeclareLaunchArgument("peer_port", default_value="5588"),
            DeclareLaunchArgument("start_sensors", default_value="false"),
            DeclareLaunchArgument("referee_status", default_value=str(simulation).lower()),
            DeclareLaunchArgument("use_nav2", default_value="false"),
            DeclareLaunchArgument("radar_diagnostics", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            *([DeclareLaunchArgument("start_base", default_value="true")] if simulation else []),
            OpaqueFunction(function=_nodes, kwargs={"simulation": simulation}),
        ]
    )
