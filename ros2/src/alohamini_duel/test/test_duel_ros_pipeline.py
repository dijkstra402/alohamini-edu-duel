"""在已构建的 ROS 环境中实测双车仿真启动和断联恢复。"""

import json
import os
import shutil
import signal
import socket
import subprocess
import time
from contextlib import ExitStack
from pathlib import Path

import pytest


def test_full_simulation_fault_and_recovery(tmp_path, monkeypatch):
    pytest.importorskip("rclpy", reason="需要项目的 ROS 2 运行环境")
    from ament_index_python.packages import PackageNotFoundError, get_package_prefix
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool, String
    from std_srvs.srv import SetBool

    try:
        get_package_prefix("alohamini_duel")
    except PackageNotFoundError:
        pytest.skip("需要已构建并加载的 alohamini_duel")
    if not shutil.which("ros2"):
        pytest.skip("需要 ROS 2 命令行入口")

    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    domains = (73, 74)
    with ExitStack() as ports:
        reserved = [
            ports.enter_context(socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
            for _ in domains
        ]
        for udp in reserved:
            udp.bind(("127.0.0.1", 0))
        port_a, port_b = (udp.getsockname()[1] for udp in reserved)

    log_path = tmp_path / "duel_sim.log"
    with ExitStack() as cleanup:
        log = cleanup.enter_context(log_path.open("w"))
        process = subprocess.Popen(
            [
                "ros2", "launch", "alohamini_duel", "duel_sim.launch.py",
                f"domain_a:={domains[0]}", f"domain_b:={domains[1]}",
                f"port_a:={port_a}", f"port_b:={port_b}",
            ],
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            env={key: value for key, value in os.environ.items() if key != "CYCLONEDDS_URI"},
        )

        def stop_launch():
            if process.poll() is not None:
                return
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)

        cleanup.callback(stop_launch)
        cyclone_config = Path(__file__).parents[1] / "config/cyclonedds_sim.xml"
        monkeypatch.setenv("CYCLONEDDS_URI", cyclone_config.read_text(encoding="utf-8"))
        telemetry = {}
        executors = []
        nodes = []
        for domain in domains:
            context = Context()
            context.init(args=[], domain_id=domain)
            cleanup.callback(context.try_shutdown)
            node = Node(f"duel_pipeline_test_{domain}", context=context)
            cleanup.callback(node.destroy_node)
            executor = SingleThreadedExecutor(context=context)
            cleanup.callback(executor.shutdown)
            executor.add_node(node)
            nodes.append(node)
            executors.append(executor)
            for kind, topic in (
                (Bool, "/duel/peer_online"),
                (Odometry, "/simulation/ground_truth"),
                (LaserScan, "/scan"),
                (String, "/duel/scoreboard"),
            ):
                node.create_subscription(
                    kind, topic,
                    lambda msg, key=(domain, topic): telemetry.__setitem__(key, msg),
                    qos_profile_sensor_data,
                )

        def wait_for(condition, timeout, description, action=None):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline and process.poll() is None:
                for executor in executors:
                    for _ in range(6):
                        executor.spin_once(timeout_sec=0.0)
                if condition():
                    return
                if action is not None:
                    action()
                time.sleep(0.04)
            pytest.fail(f"{description}\n{log_path.read_text()[-10000:]}")

        def both_online():
            return all(
                telemetry.get((domain, "/duel/peer_online"), Bool()).data for domain in domains
            )

        def ready():
            status = telemetry.get((domains[0], "/duel/scoreboard"))
            return status is not None and json.loads(status.data)["ready"]

        def switch_service(topic, value):
            client = nodes[0].create_client(SetBool, topic)
            wait_for(client.service_is_ready, 5, f"未发现 {topic}")
            future = client.call_async(SetBool.Request(data=value))
            wait_for(future.done, 5, f"{topic} 调用超时")
            assert future.result().success

        wait_for(
            lambda: both_online() and ready()
            and (domains[0], "/scan") in telemetry
            and (domains[0], "/simulation/ground_truth") in telemetry,
            40,
            "双车定位或心跳未就绪",
        )
        config = Path(__file__).parents[1] / "config"
        preflight = [
            "ros2", "run", "alohamini_duel", "duel_check", "--simulation",
            "--arena", str(config / "arena_example.yaml"),
            "--map", str(config / "duel_arena_map.yaml"),
            "--domains", *(str(domain) for domain in domains),
            "--duration", "2",
        ]
        checked = subprocess.run(preflight, capture_output=True, text=True, timeout=8, check=False)
        assert checked.returncode == 0, checked.stdout + checked.stderr
        assert json.loads(checked.stdout)["通过"]

        unexpected_publisher = nodes[0].create_publisher(Twist, "/cmd_vel", 1)
        checked = subprocess.run(preflight, capture_output=True, text=True, timeout=8, check=False)
        assert checked.returncode == 1, checked.stdout + checked.stderr
        # Humble 的 ros2 run 会在失败退出时向报告后追加退出说明。
        response, _ = json.JSONDecoder().raw_decode(checked.stdout)
        report = response["双车"]
        assert not report["sword_a"]["检查"]["速度链路只有一个仲裁和一个监测出口"]
        assert "duel_pipeline_test_73" in report["sword_a"]["速度源"]
        nodes[0].destroy_publisher(unexpected_publisher)

        switch_service("/alohamini_nav_sim/command_enable", True)
        switch_service("/alohamini_duel_base/command_enable", True)
        publisher = nodes[0].create_publisher(Twist, "/cmd_vel_duel", 1)
        command = Twist()
        command.linear.x = 0.10

        def drive():
            publisher.publish(command)

        def measured_speed():
            return telemetry[(domains[0], "/simulation/ground_truth")].twist.twist.linear.x

        wait_for(lambda: measured_speed() > 0.05, 8, "仿真车未收到底盘速度", drive)
        switch_service("/alohamini_duel_base/peer_link_enable", False)
        wait_for(
            lambda: all(not telemetry[(domain, "/duel/peer_online")].data for domain in domains)
            and not ready() and measured_speed() == 0.0,
            4,
            "断联后速度或裁判未停止",
            drive,
        )
        stopped_at = time.monotonic()

        def stopped_under_command():
            assert measured_speed() == 0.0
            return time.monotonic() - stopped_at > 0.25

        wait_for(stopped_under_command, 2, "持续输入速度时停车状态丢失", drive)
        assert telemetry[(domains[0], "/scan")].ranges[0] < 2.5
        switch_service("/alohamini_duel_base/peer_link_enable", True)
        wait_for(lambda: both_online() and ready(), 8, "恢复后双车未重新进入联机状态")

        def scoreboard(domain):
            return json.loads(telemetry[(domain, "/duel/scoreboard")].data)

        current_round = scoreboard(domains[0])["round"]
        publishers = [node.create_publisher(String, "/duel/choice", 1) for node in nodes]

        def submit_choices():
            for source, choice in zip(publishers, ("left", "right"), strict=True):
                source.publish(String(data=f"{current_round}:{choice}"))

        wait_for(
            lambda: all(scoreboard(domain)["round"] == current_round + 1 for domain in domains),
            5,
            "两域未收到同一回合结果",
            submit_choices,
        )
        assert scoreboard(domains[0]) == scoreboard(domains[1])
        assert not any(scoreboard(domains[0])["scores"].values())

        for domain in domains:
            stopped = subprocess.run(
                [
                    "ros2", "run", "alohamini_navigation", "map_session",
                    "stop", "--runtime-mode", "simulation", "--timeout", "5",
                ],
                env=os.environ | {"ROS_DOMAIN_ID": str(domain)},
                capture_output=True,
                text=True,
                timeout=15,
                check=False,
            )
            assert stopped.returncode == 0, stopped.stdout + stopped.stderr
        stop_launch()
        assert process.returncode == 0
        assert "process has died" not in log_path.read_text()
