"""验证安装后的一键验收入口与落盘报告。"""

import fcntl
import json
import math
import shutil
import signal
import subprocess
import tempfile
import time
from contextlib import ExitStack
from pathlib import Path

import pytest


@pytest.fixture
def duel_package_lookup():
    pytest.importorskip("rclpy", reason="需要项目的 ROS 2 运行环境")
    from ament_index_python.packages import PackageNotFoundError, get_package_prefix

    try:
        get_package_prefix("alohamini_duel")
        get_package_prefix("rmw_cyclonedds_cpp")
    except PackageNotFoundError:
        pytest.skip("需要已构建并加载的双车仿真依赖")
    if not shutil.which("ros2"):
        pytest.skip("需要 ROS 2 命令行入口")
    return get_package_prefix


@pytest.mark.parametrize("use_nav2", [False, True])
def test_full_simulation_fault_and_recovery(tmp_path, use_nav2, duel_package_lookup):
    if use_nav2:
        from ament_index_python.packages import PackageNotFoundError

        try:
            duel_package_lookup("nav2_bt_navigator")
        except PackageNotFoundError:
            pytest.skip("需要已加载的 Nav2 导航包")

    output = tmp_path / "report"
    command = ["ros2", "run", "alohamini_duel", "duel_accept_sim", "--output", str(output)]
    if not use_nav2:
        command.append("--no-nav2")
    process = subprocess.run(command, capture_output=True, text=True, timeout=210, check=False)
    assert (output / "report.json").is_file(), process.stdout + process.stderr
    report = json.loads((output / "report.json").read_text(encoding="utf-8"))
    assert process.returncode == 0, (report, process.stderr)
    assert report["通过"]
    assert report["启用Nav2"] == use_nav2
    assert report["ROS域"] == [73, 74]
    checks = report["检查"]
    assert checks["只读预检与速度冲突检测"]
    preflight = checks["起点预检"]
    assert preflight["通过"] and len(preflight["场地指纹"]) == 64
    assert preflight["起点偏差限值"] == {"位置_m": 0.20, "朝向_deg": 20}
    for vehicle in preflight["双车"].values():
        assert vehicle["检查"]["起点位置偏差在限值内"]
        assert vehicle["检查"]["起点朝向偏差在限值内"]
        assert vehicle["检查"]["本车名义净空"]
        assert vehicle["检查"]["实时栅格元数据与地图文件一致"]
        assert vehicle["检查"]["地图服务器处于活动状态"]
        assert vehicle["检查"]["定位节点处于活动状态"]
        assert vehicle["距标定起点_m"] <= 0.20
        assert vehicle["起点朝向偏差_deg"] <= 20
    assert checks["断联停稳"]["记录完整"]
    marked = checks["断联停稳"]
    assert marked["通过"] and marked["0.5秒内停稳"] and marked["可用于0.5秒验收"]
    assert all(car["故障前轮速显示运动"] for car in marked["双车"].values())
    speeds = checks["故障前双车纵向速度_m_s"]
    threshold = 0.025 if use_nav2 else 0.05
    assert speeds["sword_a"] > threshold
    assert (-speeds["shield_b"] if use_nav2 else speeds["shield_b"]) > threshold
    assert marked["故障标记"]["来源"] == "仿真 UDP 转发器双向断开"
    marker = json.loads((output / "stop_watch_event.json").read_text(encoding="utf-8"))
    assert marked["故障标记"]["单调时间_ns"] == marker["triggered_ns"]
    for observed in marked["双车"].values():
        assert 0 <= observed["故障到轮速停稳_s"] <= 0.5
        assert observed["故障到停稳确认_s"] >= observed["故障到轮速停稳_s"]
    timed = checks["故障注入到停稳"]
    assert set(timed["双车停稳_s"]) == {"sword_a", "shield_b"}
    assert timed["最迟停稳_s"] == max(timed["双车停稳_s"].values())
    assert timed["最迟停稳_s"] <= 0.5
    assert timed["0.5秒内停稳"]
    assert all(0 < value <= 0.5 for value in timed["双车检测断联_s"].values())
    assert checks["断联时雷达仍更新"]
    assert checks["重连"]
    assert set(checks["重连后两路输入下保持禁用和零速"]) == {"73", "74"}
    assert all(count >= 3 for count in checks["重连后两路输入下保持禁用和零速"].values())
    assert checks["有序停机"]
    assert all(
        scan["background"] > 0 and 0 < scan["对端包络命中"] <= scan["unexpected"]
        for scan in checks["雷达差分"].values()
    )
    assert checks["电子判分"]["scores"] == {"sword_a": 0, "shield_b": int(use_nav2)}
    assert ("断联取消航点" in checks) == use_nav2
    assert ("固定航点终点" in checks) == use_nav2
    if use_nav2:
        assert 0 <= checks["固定航点平面误差_m"] <= 0.15
    if not use_nav2:
        faults = checks["扩展网络故障"]
        loss = faults["每三包丢一包"]
        assert loss["丢弃"] == loss["收到"] // 3 >= 10
        assert loss["收到"] == loss["丢弃"] + loss["转发"]
        delay = faults["单向延迟"]
        assert delay["延迟_s"] == 0.75
        assert delay["延迟转发"] >= 3
        assert all(0 < value <= 0.5 for value in delay["双车停稳_s"].values())
        assert delay["0.5秒内停稳"]
        restart = faults["对端底盘进程重启"]
        assert restart["新进程"] != restart["旧进程"]
        assert restart["新会话"] != restart["旧会话"]
        assert restart["重启后运动默认禁用"]
        assert restart["最迟停稳_s"] <= 0.5
        assert restart["0.5秒内停稳"]
        assert restart["双车检测断联_s"]["shield_b"] is None
    assert (output / "stop_watch.log").is_file()


def test_both_prescribed_routes_and_same_round(tmp_path, duel_package_lookup):
    from ament_index_python.packages import PackageNotFoundError

    try:
        duel_package_lookup("nav2_bt_navigator")
    except PackageNotFoundError:
        pytest.skip("需要已加载的 Nav2 导航包")
    output = tmp_path / "report"
    command = [
        "ros2", "run", "alohamini_duel", "duel_accept_sim", "--round", "--output", str(output),
    ]
    process = subprocess.run(command, capture_output=True, text=True, timeout=180, check=False)
    assert (output / "report.json").is_file(), process.stdout + process.stderr
    report = json.loads((output / "report.json").read_text(encoding="utf-8"))
    assert process.returncode == 0, (report, process.stderr)
    assert report["通过"] and report["双车绕箱场景"]
    checks = report["检查"]
    trace = checks["双车绕箱"]
    assert trace["墙和箱体净空通过"]
    assert trace["最小中心距_m"] > 0.9
    assert 0 < trace["最大采样间隔_s"] <= 0.20
    assert set(trace["碰撞计数"]) == {"sword_a", "shield_b"}
    for count in trace["碰撞计数"].values():
        assert count["墙箱回退"] == count["双车回退"] == 0
    assert all(
        scan["background"] > 0 and 0 < scan["对端包络命中"] <= scan["unexpected"]
        for scan in checks["雷达差分"].values()
    )
    assert all(len(path) == 2 for path in trace["航点"].values())
    assert all(
        0 <= visit["平面误差_m"] <= 0.15
        for path in trace["航点"].values() for visit in path
    )
    assert checks["电子判分"]["round"] == trace["回合"] + 1
    assert checks["电子判分"]["scores"] == {"sword_a": 0, "shield_b": 0}
    assert checks["电子判分"]["result"] == "双方未进入判分条件"
    assert checks["有序停机"]


def test_dual_domain_referee_scores_and_discards_partial_choice(
    tmp_path, monkeypatch, duel_package_lookup,
):
    from alohamini_duel.acceptance import stop_process
    from alohamini_duel.arena import ROBOT_IDS, Arena
    from alohamini_duel.ros_domains import attach_domain
    from rclpy.node import Node
    from std_msgs.msg import String

    config = Path(__file__).parents[1] / "config"
    arena_path, map_path = config / "arena_example.yaml", config / "duel_arena_map.yaml"
    arena = Arena.load(arena_path, map_path)
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    monkeypatch.setenv("CYCLONEDDS_URI", (config / "cyclonedds_sim.xml").read_text())
    domains = (81, 82)
    poses = [(1.0, 1.4, 0.0), (2.0, 1.4, math.pi)]
    online = [True, True]
    boards = {}
    zero = {robot: 0 for robot in ROBOT_IDS}
    facing_score = zero | {"sword_a": 1}
    side_score = zero | {"sword_a": 3}

    with ExitStack() as resources:
        log_path = tmp_path / "referee.log"
        log = resources.enter_context(log_path.open("w", encoding="utf-8"))
        process = subprocess.Popen(
            [
                "ros2", "run", "alohamini_duel", "duel_referee",
                "--arena", str(arena_path), "--map", str(map_path), "--domains", *map(str, domains),
            ],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
        )
        resources.callback(stop_process, process)
        endpoints = [
            attach_domain(
                resources, domain, lambda context: Node("duel_referee_test", context=context),
            )
            for domain in domains
        ]
        nodes, executors = zip(*endpoints, strict=True)
        statuses = [node.create_publisher(String, "/duel/local_status", 1) for node in nodes]
        choices = [node.create_publisher(String, "/duel/choice", 10) for node in nodes]
        for index, node in enumerate(nodes):
            node.create_subscription(
                String, "/duel/scoreboard",
                lambda message, slot=index: boards.update({slot: json.loads(message.data)}), 1,
            )
        seq = 0

        def wait_for(round_id, ready, pending, scores, messages=()):
            nonlocal seq
            deadline = time.monotonic() + 5
            expected = {"round": round_id, "ready": ready, "choices_pending": pending, "scores": scores}
            while time.monotonic() < deadline and process.poll() is None:
                seq += 1
                for index, (publisher, robot) in enumerate(zip(statuses, ROBOT_IDS, strict=True)):
                    publisher.publish(String(data=json.dumps({
                        "robot_id": robot, "arena_id": arena.fingerprint, "session_id": "回归",
                        "seq": seq, "pose": poses[index], "online": online[index],
                    })))
                for index, choice in messages:
                    choices[index].publish(String(data=choice))
                for executor in executors:
                    executor.spin_once(timeout_sec=0.02)
                if len(boards) == 2 and all(
                    all(board[key] == value for key, value in expected.items()) for board in boards.values()
                ):
                    return
            pytest.fail(f"未观察到裁判状态 {expected}。收到 {boards}。{log_path.read_text()}")

        wait_for(1, True, 0, zero)
        wait_for(2, True, 0, facing_score, ((0, "1:left"), (1, "1:left")))
        assert all(board["result"] == "A 车获得正向得分" for board in boards.values())

        online[1] = False
        wait_for(3, False, 0, facing_score)
        poses[1], online[1] = (2.0, 1.4, math.pi / 2), True
        wait_for(3, True, 0, facing_score)
        wait_for(4, True, 0, side_score, ((0, "3:left"), (1, "3:right")))
        assert all(board["result"] == "A 车获得侧向得分" for board in boards.values())

        wait_for(4, True, 1, side_score, ((0, "4:right"),))
        online[1] = False
        wait_for(5, False, 0, side_score)
        assert all(board["result"] == "联机中断，回合已暂停" for board in boards.values())
        poses[1], online[1] = (2.0, 1.4, math.pi), True
        wait_for(5, True, 0, side_score)
        wait_for(5, True, 1, side_score, ((0, "4:right"), (1, "5:left")))
        wait_for(6, True, 0, side_score | {"sword_a": 4}, ((0, "5:left"),))


def test_acceptance_rejects_simultaneous_runs_with_one_shared_domain(
    tmp_path, duel_package_lookup,
):
    domain_lock = Path(tempfile.gettempdir()) / "alohamini-duel-acceptance-74.lock"
    output = tmp_path / "report"
    command = [
        "ros2", "run", "alohamini_duel", "duel_accept_sim", "--no-nav2",
        "--output", str(output), "--domains", "75", "74",
    ]
    with domain_lock.open("a") as held:
        fcntl.flock(held, fcntl.LOCK_EX | fcntl.LOCK_NB)
        process = subprocess.run(command, capture_output=True, text=True, timeout=10, check=False)
    assert (output / "report.json").is_file(), process.stdout + process.stderr
    report = json.loads((output / "report.json").read_text(encoding="utf-8"))
    assert process.returncode == 1
    assert not report["通过"]
    assert report["失败原因"] == "ROS 域 74 已有双车验收在运行"
    assert not (output / "duel_sim.log").exists()


@pytest.mark.parametrize("signum", [signal.SIGINT, signal.SIGTERM])
def test_dual_domain_shutdown_waits_for_executor_loop(signum):
    pytest.importorskip("rclpy", reason="需要项目的 ROS 2 运行环境")
    from alohamini_duel.ros_domains import watch_shutdown

    previous = signal.getsignal(signum)
    with ExitStack() as resources:
        requested = watch_shutdown(resources)
        assert not requested()
        signal.raise_signal(signum)
        assert requested()
    assert signal.getsignal(signum) == previous


@pytest.mark.parametrize("signum", [signal.SIGINT, signal.SIGTERM])
def test_single_domain_shutdown_preserves_inflight_ros_publication(signum):
    rclpy = pytest.importorskip("rclpy", reason="需要项目的 ROS 2 运行环境")
    from alohamini_duel.ros_domains import spin_until_shutdown
    from rclpy.node import Node
    from std_msgs.msg import String

    published = []

    def create_node():
        node = Node("duel_shutdown_test")
        publisher = node.create_publisher(String, "/duel/shutdown_test", 1)

        def publish_during_shutdown():
            signal.raise_signal(signum)
            publisher.publish(String(data="关停回调完成"))
            published.append(rclpy.ok())

        node.create_timer(0.02, publish_during_shutdown)
        return node

    spin_until_shutdown(create_node, [])
    assert published == [True]
    assert not rclpy.ok()
