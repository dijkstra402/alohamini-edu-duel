"""一键运行双域仿真并保存可复查的验收报告。"""

import argparse
import fcntl
import json
import math
import os
import signal
import subprocess
import tempfile
import time
from contextlib import ExitStack
from pathlib import Path

import yaml

from .arena import ROBOT_IDS
from .calibrate import calibrate
from .route import load_route
from .sim_evidence import GroundStop, RoundTrace
from .sim_link import DELAY_S, SimulationLink
from .sim_round import prepare_round
from .status import wheel_is_stationary, yaw_from_quaternion


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


def run_acceptance(output, domains, use_nav2, full_round, results):
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
    round_trace = None
    if full_round:
        map_file, arena_file, arena, round_routes = prepare_round(
            config / "round_example.yaml", output
        )
    elif use_nav2:
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
        active_stops = {}
        radar_topic = "/duel/radar_diagnostics"
        cloud_topic = "/duel/radar_unexpected"

        def receive(message, domain, robot, topic):
            telemetry[(domain, topic)] = message
            if topic == "/duel/peer_online" and robot in active_stops:
                active_stops[robot].observe_link(time.monotonic_ns(), message.data)
            if topic != "/simulation/ground_truth":
                return
            header = message.header.stamp
            stamp = header.sec * 10**9 + header.nanosec
            if robot in active_stops:
                active_stops[robot].observe(
                    stamp, time.monotonic_ns(), wheel_is_stationary(message.twist.twist)
                )
            if round_trace is not None:
                position = message.pose.pose.position
                round_trace.observe(robot, stamp, (position.x, position.y))

        for node, domain, robot in zip(nodes, domains, ROBOT_IDS, strict=True):
            for kind, topic in (
                (Bool, "/duel/peer_online"),
                (Odometry, "/simulation/ground_truth"),
                (String, "/duel/local_status"),
                (String, "/duel/scoreboard"),
                (String, radar_topic),
                (PointCloud, cloud_topic),
            ):
                node.create_subscription(
                    kind, topic,
                    lambda msg, d=domain, r=robot, t=topic: receive(msg, d, r, t),
                    qos_profile_sensor_data,
                )

        def spin():
            for executor in executors:
                for _ in range(8):
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

        link = SimulationLink(resources)
        port_a, port_b = (endpoint[1] for endpoint in link.vehicles)
        peer_a, peer_b = link.ports
        log = resources.enter_context((output / "duel_sim.log").open("w", encoding="utf-8"))
        launch = subprocess.Popen(
            [
                "ros2", "launch", "alohamini_duel", "duel_sim.launch.py",
                f"domain_a:={domains[0]}", f"domain_b:={domains[1]}",
                f"port_a:={port_a}", f"port_b:={port_b}",
                f"peer_port_a:={peer_a}", f"peer_port_b:={peer_b}",
                f"start_base_b:={str(use_nav2).lower()}",
                f"use_nav2:={str(use_nav2).lower()}", "radar_diagnostics:=true",
                f"arena:={arena_file}", f"map:={map_file}",
            ],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
        )
        resources.callback(stop_process, launch)

        def start_process(argv, domain, log_file):
            log = resources.enter_context(log_file.open("w", encoding="utf-8"))
            process = subprocess.Popen(
                argv, env=os.environ | {"ROS_DOMAIN_ID": str(domain)},
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
            )
            resources.callback(stop_process, process)
            return process

        def start_second_base(log_file):
            executable = Path(get_package_prefix("alohamini_duel")) / "lib/alohamini_duel/duel_base"
            settings = {
                "use_sim_time": "true", "simulation_controls": "true", "referee_status": "true",
                "robot_id": ROBOT_IDS[1], "arena": str(arena_file), "map": str(map_file),
                "bind_ip": "127.0.0.1", "bind_port": str(port_b),
                "peer_ip": "127.0.0.1", "peer_port": str(peer_b),
            }
            arguments = [str(executable), "--ros-args"]
            for name, value in settings.items():
                arguments.extend(("-p", f"{name}:={value}"))
            return start_process(arguments, domains[1], log_file)

        if not use_nav2:
            second_base = start_second_base(output / "base_b.log")

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
                time.sleep(0.02)
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

        def switch(topic, enabled, index=0):
            client = nodes[index].create_client(SetBool, topic)
            try:
                wait_for(client.service_is_ready, 5, f"未发现仿真服务 {topic}")
                request = client.call_async(SetBool.Request(data=enabled))
                wait_for(request.done, 5, f"仿真服务调用超时 {topic}")
                require(request.result().success, f"仿真服务拒绝请求 {topic}")
            finally:
                nodes[index].destroy_client(client)

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
            if cloud is None or not (full_round or cloud.points):
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

        def play_round(round_id):
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

        def finish(external_base=None):
            deactivated = deactivate_navigation()
            stop_process(launch)
            if external_base is not None:
                stop_process(external_base)
                require(external_base.returncode == 0, "外部底盘进程未正常退出")
            require(deactivated, "仿真导航生命周期停机失败，详见 map_session_stop.log")
            require(
                launch.returncode == 0
                and "process has died" not in (output / "duel_sim.log").read_text(encoding="utf-8"),
                "仿真存在异常退出",
            )
            results["有序停机"] = True

        for index in range(2 if full_round or not use_nav2 else 1):
            switch("/alohamini_nav_sim/command_enable", True, index)
            switch("/alohamini_duel_base/command_enable", True, index)
        if use_nav2:
            params = yaml.safe_load(Path(model_config("alohamini2edu", "nav2")).read_bytes())
            tolerance = params["controller_server"]["ros__parameters"]["general_goal_checker"][
                "xy_goal_tolerance"
            ]

        def goal_distance(domain, goal):
            position = truth(domain).pose.pose.position
            return math.hypot(position.x - goal[0], position.y - goal[1])

        if full_round:
            round_trace = RoundTrace(arena)
            round_id = scoreboard(domains[0])["round"]
            waypoints = {robot: load_route(path, arena) for robot, path in round_routes.items()}
            arrivals = {robot: [] for robot in ROBOT_IDS}
            for stage, goals in enumerate(zip(*waypoints.values(), strict=True), 1):
                routes = []
                for robot, domain, goal in zip(ROBOT_IDS, domains, goals, strict=True):
                    pose_file = output / f"round_{robot}_{stage}.yaml"
                    pose_file.write_text(yaml.safe_dump([list(goal)]), encoding="utf-8")
                    log_file = output / f"round_{robot}_{stage}.log"
                    route_command = command(
                        "duel_route", "--robot-id", robot, "--timeout", "45",
                        "--route", str(pose_file),
                    )
                    routes.append((start_process(route_command, domain, log_file), log_file))
                wait_for(
                    lambda: all(process.poll() is not None for process, _ in routes),
                    52, f"双车第 {stage} 段绕箱路线超时",
                )
                for process, log_file in routes:
                    require(process.returncode == 0, log_file.read_text(encoding="utf-8"))
                wait_for(
                    lambda: stationary() and all(
                        goal_distance(domain, goal) <= tolerance
                        for domain, goal in zip(domains, goals, strict=True)
                    ),
                    3, f"双车第 {stage} 段绕箱路线未到达并停稳",
                )
                for robot, domain, goal in zip(ROBOT_IDS, domains, goals, strict=True):
                    pose = truth(domain).pose.pose
                    arrivals[robot].append({
                        "目标_m": list(goal[:2]), "真值_m": [pose.position.x, pose.position.y],
                        "真值朝向_rad": yaw_from_quaternion(pose.orientation),
                        "平面误差_m": goal_distance(domain, goal),
                    })
            require(
                all(scoreboard(domain).get("round") == round_id for domain in domains),
                "双车绕箱期间回合被中断",
            )
            require(round_trace.count > 50 and round_trace.max_gap <= 0.20,
                    f"双域真值采样间隔过长 {round_trace.max_gap:.3f} 秒")
            results["双车绕箱"] = {
                "回合": round_id, "航点": arrivals, "墙和箱体净空通过": True,
                "最小中心距_m": round_trace.min_distance, "配对真值帧数": round_trace.count,
                "最大采样间隔_s": round_trace.max_gap,
                "判分定位_m_rad": {
                    robot: read_json(domain, "/duel/local_status")["pose"]
                    for robot, domain in zip(ROBOT_IDS, domains, strict=True)
                },
            }
            play_round(round_id)
            require(
                results["电子判分"]["result"] == "双方未进入判分条件",
                "独立路线结束后的判分结果异常",
            )
            finish()
            return

        route = command("duel_route", "--robot-id", ROBOT_IDS[0], "--timeout", "35")
        if use_nav2:
            route_file = config / "route_example.yaml"
            goal = load_route(route_file, arena)[-1]
            run([*route, "--route", str(route_file)], 40, domain=domains[0])
            wait_for(
                lambda: goal_distance(domains[0], goal) <= tolerance and stationary(),
                3, "仿真车未驶入目标航点邻域并停稳",
            )
            position = truth(domains[0]).pose.pose.position
            results["固定航点终点"] = [position.x, position.y]
            results["固定航点平面误差_m"] = goal_distance(domains[0], goal)

        velocity = [
            node.create_publisher(Twist, "/cmd_vel_duel", 1)
            for node in (nodes[:1] if use_nav2 else nodes)
        ]
        move = Twist()
        move.linear.x = 0.1

        def drive():
            for publisher in velocity:
                publisher.publish(move)

        missed_deadlines = []

        def start_timing(injected_ns):
            active_stops.update((robot, GroundStop(injected_ns)) for robot in ROBOT_IDS)

        def stopped_with_evidence():
            return all(evidence.stopped_s is not None for evidence in active_stops.values())

        def timing_report(scenario):
            elapsed = {robot: evidence.stopped_s for robot, evidence in active_stops.items()}
            confirmed = {
                robot: evidence.confirmed_s for robot, evidence in active_stops.items()
            }
            lost = {robot: evidence.link_lost_s for robot, evidence in active_stops.items()}
            active_stops.clear()
            maximum = max(elapsed.values())
            if maximum > 0.5:
                missed_deadlines.append(f"{scenario} {maximum:.3f} 秒")
            return {
                "双车检测断联_s": lost, "双车停稳_s": elapsed, "双车停稳确认_s": confirmed,
                "最迟停稳_s": maximum, "0.5秒内停稳": maximum <= 0.5,
            }

        def referee_paused():
            return all(not scoreboard(domain).get("ready") for domain in domains)

        def both_offline():
            return all(not telemetry[(domain, "/duel/peer_online")].data for domain in domains)

        def recovered():
            return both_online() and all(scoreboard(domain).get("ready") for domain in domains)

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
            return all(
                truth(domain).twist.twist.linear.x > (0.025 if use_nav2 else 0.05)
                for domain in (domains[:1] if use_nav2 else domains)
            )

        wait_for(moving, 8, "故障注入前未观察到仿真车运动", drive if not use_nav2 else None)
        if not use_nav2:
            moving_at = time.monotonic()
            wait_for(
                lambda: moving() and time.monotonic() - moving_at >= 0.25,
                2, "故障注入前仿真车未保持运动", drive,
            )
        start_timing(link.set_mode("cut"))
        wait_for(
            lambda: both_offline() and referee_paused() and stopped_with_evidence(),
            4, "断联后底盘或裁判未停", drive,
        )
        results["故障注入到停稳"] = timing_report("直接断联")
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
        link.set_mode("normal")
        wait_for(recovered, 8, "重连后双车未重新上线")
        if use_nav2:
            stopped_at = time.monotonic()
            wait_for(sustained_stop, 2, "重连后被取消的任务恢复运动")
        results["重连"] = True

        if not use_nav2:
            fault_report = {}
            results["扩展网络故障"] = fault_report
            link.set_mode("loss")

            def confirms_loss():
                require(recovered(), "部分丢包期间联机状态意外失效")
                counts = link.snapshot()
                return counts["丢弃"] >= 10 and counts["转发"] >= 20

            wait_for(confirms_loss, 4, "部分丢包样本不足")
            fault_report["每三包丢一包"] = link.snapshot()
            link.set_mode("normal")

            wait_for(moving, 5, "延迟故障注入前双车未移动", drive)
            start_timing(link.set_mode("delay"))
            wait_for(
                lambda: both_offline() and referee_paused() and stopped_with_evidence(),
                4, "单向延迟后双车或裁判未停", drive,
            )
            delay = timing_report("单向延迟")
            wait_for(
                lambda: link.snapshot()["延迟转发"] >= 3,
                3, "未观察到实际延迟转发", drive,
            )
            require(both_offline() and stationary(), "迟到的心跳使车辆提前恢复")
            fault_report["单向延迟"] = {"延迟_s": DELAY_S, **delay, **link.snapshot()}
            link.set_mode("normal")
            wait_for(recovered, 8, "移除延迟后双车未重新上线")

            status_topic = "/duel/local_status"
            sessions = [read_json(domain, status_topic)["session_id"] for domain in domains]
            wait_for(moving, 5, "进程退出前双车未移动", drive)
            start_timing(time.monotonic_ns())
            second_base.kill()
            wait_for(
                lambda: second_base.poll() is not None
                and not telemetry[(domains[0], "/duel/peer_online")].data
                and referee_paused() and stopped_with_evidence(),
                6, "对端进程退出后双车或裁判未停", drive,
            )
            restart = timing_report("对端底盘进程退出")
            require(second_base.returncode == -signal.SIGKILL, "对端仿真底盘未被故障信号终止")
            new_base = start_second_base(output / "base_b_restart.log")
            wait_for(
                lambda: recovered() and read_json(domains[1], status_topic).get("session_id")
                != sessions[1] and stationary(),
                10, "重启后双车未通过新会话恢复并保持静止",
            )
            current_sessions = [read_json(domain, status_topic)["session_id"] for domain in domains]
            require(current_sessions[0] == sessions[0], "本车底盘进程意外重启")
            require(
                read_json(domains[1], status_topic)["motion_enabled"] is False
                and new_base.poll() is None,
                "重启后的对端底盘未保持默认禁用",
            )
            fault_report["对端底盘进程重启"] = {
                **restart, "旧进程": second_base.pid, "新进程": new_base.pid,
                "旧会话": sessions[1], "新会话": current_sessions[1], "重启后运动默认禁用": True,
            }

        play_round(scoreboard(domains[0])["round"])
        finish(new_base if not use_nav2 else None)
        require(not missed_deadlines, f"停稳超过 0.5 秒。{'、'.join(missed_deadlines)}")


def main(argv=None):
    started = time.monotonic()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="新增的报告目录")
    parser.add_argument("--domains", nargs=2, type=int, default=(73, 74))
    scenario = parser.add_mutually_exclusive_group()
    scenario.add_argument("--no-nav2", action="store_true", help="验收纯定位与扩展网络故障")
    scenario.add_argument("--round", action="store_true", help="验收双车绕箱与完整电子回合")
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
    report = {
        "通过": False, "ROS域": args.domains, "启用Nav2": not args.no_nav2,
        "双车绕箱场景": args.round, "检查": results,
    }
    try:
        run_acceptance(output, args.domains, not args.no_nav2, args.round, results)
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
