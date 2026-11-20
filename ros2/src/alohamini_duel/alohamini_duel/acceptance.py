"""一键运行双域仿真并保存可复查的验收报告。"""

import argparse
import fcntl
import json
import math
import os
import signal
import socket
import subprocess
import tempfile
import time
from contextlib import ExitStack
from pathlib import Path

import yaml

from .calibrate import calibrate
from .route import load_route
from .status import wheel_is_stationary


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def stop_process(process):
    if process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=5)


def run_acceptance(output, domains, use_nav2, results):
    from alohamini_navigation.launching import model_config
    from ament_index_python.packages import (
        PackageNotFoundError,
        get_package_prefix,
        get_package_share_directory,
    )
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud
    from std_msgs.msg import Bool, String
    from std_srvs.srv import SetBool

    from .ros_domains import attach_domain

    try:
        get_package_prefix("rmw_cyclonedds_cpp")
        if use_nav2:
            get_package_prefix("nav2_bt_navigator")
        config = Path(get_package_share_directory("alohamini_duel")) / "config"
    except PackageNotFoundError as error:
        raise RuntimeError("缺少验收依赖，请加载含 Cyclone DDS 和所需导航包的 ROS 环境") from error
    map_file = config / "duel_arena_map.yaml"
    arena_file = config / "arena_example.yaml"
    expected_scores = {"sword_a": 0, "shield_b": 0}
    if use_nav2:
        measurements = yaml.safe_load((config / "measurements_example.yaml").read_bytes())
        measurements["start_poses"]["shield_b"][:2] = [1.80, 1.80]
        arena_file = output / "acceptance_arena.yaml"
        arena = calibrate(measurements, map_file, arena_file, 0.03)
        expected_scores["shield_b"] = 1
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    os.environ["ROS_LOG_DIR"] = str(output / "ros")
    os.environ["CYCLONEDDS_URI"] = (config / "cyclonedds_sim.xml").read_text(encoding="utf-8")
    sim_args = ["--simulation", "--arena", str(arena_file), "--map", str(map_file)]
    domain_args = ["--domains", *(str(domain) for domain in domains)]

    def command(name, *arguments):
        return ["ros2", "run", "alohamini_duel", name, *sim_args, *arguments]

    def run(argv, timeout, *, domain=None, expected=0):
        env = os.environ if domain is None else os.environ | {"ROS_DOMAIN_ID": str(domain)}
        with subprocess.Popen(
            argv, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            text=True, start_new_session=True,
        ) as process:
            try:
                stdout, stderr = process.communicate(timeout=timeout)
            except (subprocess.TimeoutExpired, KeyboardInterrupt):
                os.killpg(process.pid, signal.SIGKILL)
                process.communicate()
                raise
            require(process.returncode == expected, f"{argv[3]} 退出异常\n{stdout}{stderr}")
        return stdout

    with ExitStack() as resources:
        for domain in domains:
            lock = Path(tempfile.gettempdir()) / f"alohamini-duel-acceptance-{domain}.lock"
            handle = resources.enter_context(lock.open("a"))
            try:
                fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as error:
                raise RuntimeError(f"ROS 域 {domain} 已有双车验收在运行") from error
        endpoints = [
            attach_domain(
                resources, domain,
                lambda ctx, domain=domain: Node(
                    f"duel_acceptance_{domain}_{os.getpid()}", context=ctx
                ),
            )
            for domain in domains
        ]
        nodes, executors = zip(*endpoints, strict=True)
        telemetry = {}
        radar_topic = "/duel/radar_diagnostics"
        cloud_topic = "/duel/radar_unexpected"
        for node, domain in zip(nodes, domains, strict=True):
            for kind, topic in (
                (Bool, "/duel/peer_online"),
                (Odometry, "/simulation/ground_truth"),
                (String, "/duel/scoreboard"),
                (String, radar_topic),
                (PointCloud, cloud_topic),
            ):
                node.create_subscription(
                    kind, topic,
                    lambda msg, key=(domain, topic): telemetry.__setitem__(key, msg),
                    qos_profile_sensor_data,
                )

        def spin():
            for executor in executors:
                for _ in range(6):
                    executor.spin_once(timeout_sec=0.0)

        # 仿真验收使用本机空闲域，启动前留出发现其他节点的时间。
        discovery_deadline = time.monotonic() + 0.75
        while time.monotonic() < discovery_deadline:
            spin()
            time.sleep(0.04)
        for node, domain in zip(nodes, domains, strict=True):
            require(
                node.get_node_names() == [node.get_name()],
                f"ROS 域 {domain} 已有其他节点，需选用空闲域",
            )

        with ExitStack() as ports:
            sockets = [
                ports.enter_context(socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
                for _ in domains
            ]
            for udp in sockets:
                udp.bind(("127.0.0.1", 0))
            port_a, port_b = [udp.getsockname()[1] for udp in sockets]
        log = resources.enter_context((output / "duel_sim.log").open("w", encoding="utf-8"))
        launch = subprocess.Popen(
            [
                "ros2", "launch", "alohamini_duel", "duel_sim.launch.py",
                f"domain_a:={domains[0]}", f"domain_b:={domains[1]}",
                f"port_a:={port_a}", f"port_b:={port_b}",
                f"use_nav2:={str(use_nav2).lower()}", "radar_diagnostics:=true",
                f"arena:={arena_file}", f"map:={map_file}",
            ],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
        )
        resources.callback(stop_process, launch)

        def deactivate_navigation():
            if launch.poll() is not None:
                return False
            succeeded = True
            with (output / "map_session_stop.log").open("a", encoding="utf-8") as stop_log:
                for domain in domains:
                    try:
                        log_result = run(
                            [
                                "ros2", "run", "alohamini_navigation", "map_session", "stop",
                                "--runtime-mode", "simulation", "--timeout", "5",
                            ],
                            15, domain=domain,
                        )
                    except (OSError, RuntimeError, subprocess.SubprocessError) as error:
                        log_result = str(error)
                        succeeded = False
                    print(f"ROS 域 {domain}\n{log_result}", file=stop_log)
            return succeeded

        def wait_for(condition, timeout, description, action=None):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline and launch.poll() is None:
                spin()
                if condition():
                    return
                if action is not None:
                    action()
                time.sleep(0.04)
            raise RuntimeError(description)

        def both_online():
            return all(
                telemetry.get((domain, "/duel/peer_online"), Bool()).data for domain in domains
            )

        def read_json(domain, topic):
            message = telemetry.get((domain, topic))
            return json.loads(message.data) if message is not None else {}

        def scoreboard(domain):
            return read_json(domain, "/duel/scoreboard")

        def truth(domain):
            return telemetry[(domain, "/simulation/ground_truth")]

        def stationary():
            return all(wheel_is_stationary(truth(domain).twist.twist) for domain in domains)

        def switch(topic, enabled):
            client = nodes[0].create_client(SetBool, topic)
            try:
                wait_for(client.service_is_ready, 5, f"未发现仿真服务 {topic}")
                request = client.call_async(SetBool.Request(data=enabled))
                wait_for(request.done, 5, f"仿真服务调用超时 {topic}")
                require(request.result().success, f"仿真服务拒绝请求 {topic}")
            finally:
                nodes[0].destroy_client(client)

        wait_for(
            lambda: both_online() and all(scoreboard(domain).get("ready") for domain in domains)
            and all((domain, "/simulation/ground_truth") in telemetry for domain in domains),
            40, "双车定位或心跳未就绪",
        )
        results["双车上线"] = True
        resources.callback(deactivate_navigation)

        def radar_ready(domain):
            cloud = telemetry.get((domain, cloud_topic))
            summary = read_json(domain, radar_topic)
            if cloud is None or not cloud.points:
                return False
            stamp = cloud.header.stamp
            return (
                cloud.header.frame_id == "map"
                and summary.get("stamp_ns") == stamp.sec * 10**9 + stamp.nanosec
                and summary["unexpected"] == len(cloud.points)
            )

        wait_for(
            lambda: all(radar_ready(domain) for domain in domains),
            8, "双域未取得与雷达诊断匹配的地图坐标点云",
        )
        results["雷达差分"] = {str(domain): read_json(domain, radar_topic) for domain in domains}

        preflight = command("duel_check", *domain_args, "--duration", "2")
        require(json.loads(run(preflight, 8))["通过"], "正常状态预检未通过")
        extra = nodes[0].create_publisher(Twist, "/cmd_vel", 1)
        # Humble 的 ros2 run 会在失败码之后为标准输出追加退出说明。
        conflict, _ = json.JSONDecoder().raw_decode(run(preflight, 8, expected=1))
        first = conflict["双车"]["sword_a"]
        require(
            not first["检查"]["速度链路只有一个仲裁和一个监测出口"]
            and nodes[0].get_name() in first["速度源"],
            "预检未识别出额外的速度发布者",
        )
        nodes[0].destroy_publisher(extra)
        results["只读预检与速度冲突检测"] = True

        switch("/alohamini_nav_sim/command_enable", True)
        switch("/alohamini_duel_base/command_enable", True)
        route = command("duel_route", "--robot-id", "sword_a", "--timeout", "35")
        if use_nav2:
            route_file = config / "route_example.yaml"
            x, y, _ = load_route(route_file, arena)[-1]
            params = yaml.safe_load(Path(model_config("alohamini2edu", "nav2")).read_bytes())
            goal_checker = params["controller_server"]["ros__parameters"]["general_goal_checker"]

            def goal_distance():
                position = truth(domains[0]).pose.pose.position
                return math.hypot(position.x - x, position.y - y)

            run([*route, "--route", str(route_file)], 40, domain=domains[0])
            wait_for(
                lambda: goal_distance() <= goal_checker["xy_goal_tolerance"] and stationary(),
                3, "仿真车未驶入目标航点邻域并停稳",
            )
            position = truth(domains[0]).pose.pose.position
            results["固定航点终点"] = [position.x, position.y]
            results["固定航点平面误差_m"] = goal_distance()

        velocity = nodes[0].create_publisher(Twist, "/cmd_vel_duel", 1)
        move = Twist()
        move.linear.x = 0.1

        def drive():
            velocity.publish(move)

        watch_path = output / "stop_watch.log"
        watch_report = output / "stop_watch.json"
        watch_log = resources.enter_context(watch_path.open("w", encoding="utf-8"))
        watch_stdout = resources.enter_context(watch_report.open("w", encoding="utf-8"))
        watcher = subprocess.Popen(
            command("duel_stop_watch", *domain_args, "--duration", "15"),
            stdout=watch_stdout, stderr=watch_log, start_new_session=True,
        )
        resources.callback(stop_process, watcher)
        wait_for(
            lambda: "等待断联事件" in watch_path.read_text(encoding="utf-8"),
            6, "断联记录器未收到双车输入", drive if not use_nav2 else None,
        )
        nav_goal = None
        route_log_path = output / "route_cancel.log"
        if use_nav2:
            long_route = output / "fault_route.yaml"
            long_route.write_text("- [1.48, 1.48, 0.7853981634]\n", encoding="utf-8")
            route_log = resources.enter_context(route_log_path.open("w", encoding="utf-8"))
            nav_goal = subprocess.Popen(
                [*route, "--route", str(long_route)],
                env=os.environ | {"ROS_DOMAIN_ID": str(domains[0])},
                stdout=route_log, stderr=subprocess.STDOUT, start_new_session=True,
            )
            resources.callback(stop_process, nav_goal)
            wait_for(
                lambda: "固定航点导航已启动" in route_log_path.read_text(encoding="utf-8"),
                5, "第二条路线未启动",
            )
        def moving():
            return truth(domains[0]).twist.twist.linear.x > (0.025 if use_nav2 else 0.05)

        wait_for(moving, 8, "故障注入前未观察到仿真车运动", drive if not use_nav2 else None)
        if not use_nav2:
            moving_at = time.monotonic()
            wait_for(
                lambda: moving() and time.monotonic() - moving_at >= 0.25,
                2, "故障注入前仿真车未保持运动", drive,
            )
        switch("/alohamini_duel_base/peer_link_enable", False)
        wait_for(
            lambda: all(not telemetry[(domain, "/duel/peer_online")].data for domain in domains)
            and all(not scoreboard(domain).get("ready") for domain in domains)
            and stationary(),
            4, "断联后底盘或裁判未停", drive,
        )
        stopped_at = time.monotonic()

        def sustained_stop():
            require(stationary(), "停稳后观察到异常速度")
            return time.monotonic() - stopped_at >= 0.25

        wait_for(sustained_stop, 2, "持续速度输入下未保持停车", drive)
        wait_for(lambda: watcher.poll() is not None, 16, "断联停车记录器未结束", drive)
        recorded = watch_report.read_text(encoding="utf-8")
        require(watcher.returncode == 0, recorded + watch_path.read_text(encoding="utf-8"))
        evidence = json.loads(recorded)
        require(
            evidence["记录完整"] and evidence["双车"]["sword_a"]["断联前轮速显示运动"],
            "缺少断联停车证据",
        )
        for car in evidence["双车"].values():
            require(
                car["检测断联_s"] > 0 and car["检测到零速_s"] >= 0 and car["检测到停稳_s"] > 0,
                "断联时间或停稳时间无效",
            )
        results["断联停稳"] = evidence
        if nav_goal is not None:
            require(
                nav_goal.wait(timeout=8) == 1
                and "Nav2 已取消固定航点任务" in route_log_path.read_text(encoding="utf-8"),
                "Nav2 未确认断联取消",
            )
            results["断联取消航点"] = True
        radar_stamps = [read_json(domain, radar_topic).get("stamp_ns", 0) for domain in domains]
        wait_for(
            lambda: all(
                read_json(domain, radar_topic).get("stamp_ns", 0) > stamp
                and radar_ready(domain)
                for domain, stamp in zip(domains, radar_stamps, strict=True)
            ),
            4, "断联后雷达诊断未继续更新",
        )
        results["断联时雷达仍更新"] = True
        switch("/alohamini_duel_base/peer_link_enable", True)
        wait_for(
            lambda: both_online() and all(scoreboard(domain).get("ready") for domain in domains),
            8, "重连后双车未重新上线",
        )
        if use_nav2:
            stopped_at = time.monotonic()
            wait_for(sustained_stop, 2, "重连后被取消的任务恢复运动")
        results["重连"] = True

        round_id = scoreboard(domains[0])["round"]
        publishers = [node.create_publisher(String, "/duel/choice", 1) for node in nodes]

        def submit_choices():
            for source, choice in zip(publishers, ("left", "right"), strict=True):
                source.publish(String(data=f"{round_id}:{choice}"))

        wait_for(
            lambda: all(scoreboard(domain).get("round") == round_id + 1 for domain in domains),
            5, "两域未收到相同回合编号", submit_choices,
        )
        first_score = scoreboard(domains[0])
        require(first_score == scoreboard(domains[1]), "两域裁判结果不一致")
        require(first_score["scores"] == expected_scores, f"电子判分异常：{first_score}")
        results["电子判分"] = first_score

        deactivated = deactivate_navigation()
        stop_process(launch)
        require(deactivated, "仿真导航生命周期停机失败，详见 map_session_stop.log")
        require(
            launch.returncode == 0
            and "process has died" not in (output / "duel_sim.log").read_text(encoding="utf-8"),
            "仿真存在异常退出",
        )
        results["有序停机"] = True


def main(argv=None):
    started = time.monotonic()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="新增的报告目录")
    parser.add_argument("--domains", nargs=2, type=int, default=(73, 74))
    parser.add_argument("--no-nav2", action="store_true", help="验收纯定位模式")
    args = parser.parse_args(argv)
    if len(set(args.domains)) != 2 or not all(0 <= domain <= 101 for domain in args.domains):
        parser.error("需要两个不同的 ROS 域编号，范围为 0 至 101")
    output = Path(tempfile.mkdtemp(prefix="duel-acceptance-")) if args.output is None else args.output
    if args.output is not None:
        try:
            output.mkdir(parents=True)
        except OSError as error:
            parser.error(f"无法创建报告目录：{error}")
    output = output.resolve()
    results = {}
    report = {"通过": False, "ROS域": args.domains, "启用Nav2": not args.no_nav2, "检查": results}
    try:
        run_acceptance(output, args.domains, not args.no_nav2, results)
        report["通过"] = True
    except (OSError, RuntimeError, ValueError, subprocess.SubprocessError) as error:
        report["失败原因"] = str(error)
    except KeyboardInterrupt:
        report["失败原因"] = "验收由操作者中断"
    report["耗时_s"] = round(time.monotonic() - started, 3)
    report_file = output / "report.json"
    report_file.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"{'验收通过' if report['通过'] else '验收未通过'}，报告位于 {report_file}")
    return 0 if report["通过"] else 1
