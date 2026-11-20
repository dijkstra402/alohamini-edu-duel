"""启动两辆仿真车、共享世界和软件裁判。"""

import os
from pathlib import Path

from alohamini_duel.arena import ROBOT_IDS
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def _nodes(context):
    def value(key):
        return LaunchConfiguration(key).perform(context)

    arena, map_file = (str(Path(value(key)).expanduser().resolve()) for key in ("arena", "map"))
    domains = value("domain_a"), value("domain_b")
    ports = value("port_a"), value("port_b")
    if any(not 0 <= int(domain) <= 101 for domain in domains) or len(set(map(int, domains))) != 2:
        raise ValueError("两辆仿真车需要不同的 ROS 域编号，范围为 0 至 101")
    if any(not 1024 <= int(port) <= 65535 for port in ports) or len(set(map(int, ports))) != 2:
        raise ValueError("两辆仿真车需要不同的 UDP 端口，范围为 1024 至 65535")
    cyclone_config = Path(get_package_share_directory("alohamini_duel"))
    local_env = {
        "ROS_LOCALHOST_ONLY": "1",
        "CYCLONEDDS_URI": os.environ.get("CYCLONEDDS_URI")
        or (cyclone_config / "config/cyclonedds_sim.xml").read_text(encoding="utf-8"),
    }
    world = Node(
        package="alohamini_duel",
        executable="shared_duel_sim",
        arguments=["--arena", arena, "--map", map_file, "--domains", *domains],
        additional_env=local_env,
        output="screen",
    )
    referee = Node(
        package="alohamini_duel",
        executable="duel_referee",
        arguments=["--arena", arena, "--map", map_file, "--domains", *domains],
        additional_env=local_env,
        output="screen",
    )
    vehicles = [
        ExecuteProcess(
            cmd=[
                "ros2", "launch", "alohamini_duel", "duel_vehicle_sim.launch.py",
                f"robot_id:={robot_id}", f"arena:={arena}", f"map:={map_file}",
                "bind_ip:=127.0.0.1", f"bind_port:={ports[index]}",
                "peer_ip:=127.0.0.1", f"peer_port:={ports[1 - index]}",
                f"use_nav2:={value('use_nav2')}",
                f"radar_diagnostics:={value('radar_diagnostics')}",
                f"use_rviz:={value('use_rviz')}",
            ],
            name=f"duel_{robot_id}",
            additional_env=local_env | {"ROS_DOMAIN_ID": domains[index]},
            output="screen",
        )
        for index, robot_id in enumerate(ROBOT_IDS)
    ]
    processes = (world, referee, *vehicles)
    return [
        *[
            RegisterEventHandler(
                OnProcessExit(
                    target_action=process,
                    on_exit=[EmitEvent(event=Shutdown(reason="双车仿真组件已退出"))],
                )
            )
            for process in processes
        ],
        *processes,
    ]


def generate_launch_description():
    config = Path(get_package_share_directory("alohamini_duel")) / "config"
    defaults = {
        "arena": str(config / "arena_example.yaml"),
        "map": str(config / "duel_arena_map.yaml"),
        "domain_a": "45",
        "domain_b": "46",
        "port_a": "5588",
        "port_b": "5589",
        "use_rviz": "false",
        "use_nav2": "false",
        "radar_diagnostics": "false",
    }
    return LaunchDescription(
        [
            *[DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()],
            OpaqueFunction(function=_nodes),
        ]
    )
