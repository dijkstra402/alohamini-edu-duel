import json
import math
import select
import socket
import time
from contextlib import ExitStack
from copy import deepcopy
from dataclasses import replace
from pathlib import Path

import pytest
from alohamini.schema import BodyVelocity
from alohamini_duel.arena import BASE_RADIUS, ROBOT_IDS, Arena
from alohamini_duel.cmd_vel_arbiter import MAX_SPEED, STOP, VelocityArbiter
from alohamini_duel.peer_sync import PeerLink, PeerState, open_peer_socket
from alohamini_duel.route import load_route
from alohamini_duel.status import VehicleStatus

POSE_A = (1.45, 1.4, 0.0)
POSE_B = (2.4, 2.45, math.pi)


@pytest.fixture
def arena(tmp_path):
    config = Path(__file__).parents[1] / "config/arena_example.yaml"
    map_file = tmp_path / "map.yaml"
    map_file.write_text("image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n")
    (tmp_path / "map.pgm").write_bytes(b"P5\n1 1\n255\n\0")
    return Arena.load(config, map_file)


@pytest.fixture
def map_metadata(arena, tmp_path):
    pytest.importorskip("rclpy", reason="需要项目的 ROS 2 运行环境")
    from alohamini_duel.preflight import load_map_metadata

    return load_map_metadata(tmp_path / "map.yaml")


def exchange(a, b, now):
    sent_a = a.encode(POSE_A, 0.0, 0.0, now)
    sent_b = json.loads(b.encode(POSE_B, 0.0, 0.0, now))
    # 对端报文字段的时钟原点不同，本机只使用自己的发送与接收时间。
    sent_b["tx_mono_s"] = now + 100_000
    a.accept(json.dumps(sent_b).encode(), now)
    b.accept(sent_a, now)
    return sent_a


def test_geometry_uses_rotated_map_and_sweeps_entire_segment(arena):
    rotated = Arena((8, -4, math.pi / 2), arena.size, arena.obstacles, {}, arena.fingerprint)
    assert rotated.local((7.5, -2)) == pytest.approx((2, 0.5))
    assert rotated.contains((7.5, -2))
    assert not rotated.contains((8.1, -2))
    assert not arena.path_clear((0.65, 1.3), (0.9, 2.2), 0.05)
    assert not arena.path_clear((2.0, 2.0), (1.0, 2.0), BASE_RADIUS)
    assert arena.path_clear((1.5, 1.2), (1.5, 1.4), BASE_RADIUS)


def test_map_fingerprint_includes_grid_and_invalid_start_is_rejected(arena, tmp_path):
    src = Path(__file__).parents[1] / "config/arena_example.yaml"
    map_file = tmp_path / "map.yaml"
    (tmp_path / "map.pgm").write_bytes(b"P5\n1 1\n255\n\xff")
    assert Arena.load(src, map_file).fingerprint != arena.fingerprint
    config = tmp_path / "arena.yaml"
    config.write_text(src.read_text().replace("0.55, 0.55", "0.05, 0.05"))
    with pytest.raises(ValueError, match="底盘净空"):
        Arena.load(config, map_file)


def test_arena_load_rejects_managed_map_for_another_model(arena, tmp_path):
    config = Path(__file__).parents[1] / "config/arena_example.yaml"
    (tmp_path / "manifest.json").write_text(
        json.dumps({"status": "complete", "robot_model": "alohamini2pro"}),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="another robot model"):
        Arena.load(config, tmp_path / "map.yaml")


def test_vehicle_status_distinguishes_motion_permission_and_validates_identity(arena):
    frame = {
        "robot_id": ROBOT_IDS[0],
        "arena_id": arena.fingerprint,
        "session_id": "本车会话",
        "seq": 1,
        "online": False,
        "pose": POSE_A,
    }
    legacy = VehicleStatus.decode(json.dumps(frame), ROBOT_IDS[0], arena)
    assert legacy.motion_enabled is None and legacy.pose == POSE_A and not legacy.online
    disabled = VehicleStatus.decode(
        json.dumps(frame | {"motion_enabled": False}), ROBOT_IDS[0], arena
    )
    assert disabled.motion_enabled is False
    for invalid in (
        {"arena_id": "不同地图"},
        {"robot_id": ROBOT_IDS[1]},
        {"motion_enabled": "false"},
        {"online": True, "pose": None},
    ):
        assert VehicleStatus.decode(json.dumps(frame | invalid), ROBOT_IDS[0], arena) is None


def test_preflight_start_limits_require_recent_samples_in_the_same_session(arena, map_metadata):
    pytest.importorskip("rclpy", reason="需要项目的 ROS 2 运行环境")
    from rclpy.context import Context
    from std_msgs.msg import String

    from alohamini_duel.preflight import CLEARANCE_CHECK, HEADING_CHECK, POSITION_CHECK, VehicleCheck

    robot = ROBOT_IDS[0]
    arena = replace(
        arena, starts=arena.starts | {robot: (*POSE_A[:2], math.radians(179))}
    )
    frame = {
        "robot_id": robot, "arena_id": arena.fingerprint, "session_id": "会话一",
        "pose": [POSE_A[0] + 0.05, POSE_A[1], math.radians(-179)],
        "online": True, "motion_enabled": False,
    }
    with ExitStack() as resources:
        context = Context()
        context.init(args=[], domain_id=92)
        resources.callback(context.try_shutdown)
        node = VehicleCheck(
            context, arena, robot, False, map_metadata=map_metadata,
            position_limit=0.10, heading_limit=3.0,
        )
        resources.callback(node.destroy_node)
        for sequence in (1, 2, 2):
            node.on_status(String(data=json.dumps(frame | {"seq": sequence})))
        assert not node.results({}, None)["检查"][CLEARANCE_CHECK]
        node.on_status(String(data=json.dumps(frame | {"seq": 3})))
        passed = node.results({}, None)
        assert passed["检查"][POSITION_CHECK]
        assert passed["检查"][CLEARANCE_CHECK]
        assert passed["检查"][HEADING_CHECK]
        assert passed["距标定起点_m"] == pytest.approx(0.05)
        assert passed["起点朝向偏差_deg"] == pytest.approx(2.0)

        displaced = frame | {"seq": 4, "pose": [POSE_A[0] + 0.15, *frame["pose"][1:]]}
        node.on_status(String(data=json.dumps(displaced)))
        displaced_checks = node.results({}, None)["检查"]
        assert not displaced_checks[POSITION_CHECK]
        assert displaced_checks[HEADING_CHECK]

        rotated = frame | {"seq": 5, "pose": [*frame["pose"][:2], math.radians(-170)]}
        node.on_status(String(data=json.dumps(rotated)))
        assert not node.results({}, None)["检查"][HEADING_CHECK]
        for sequence in (6, 7, 8):
            node.on_status(String(data=json.dumps(frame | {"seq": sequence})))
        assert node.results({}, None)["检查"][POSITION_CHECK]
        restarted = frame | {"seq": 1, "session_id": "会话二"}
        node.on_status(String(data=json.dumps(restarted)))
        assert not node.results({}, None)["检查"][POSITION_CHECK]
        assert not node.results({}, None)["检查"][CLEARANCE_CHECK]
        wrong_map = restarted | {"seq": 2, "arena_id": "不同场地"}
        node.on_status(String(data=json.dumps(wrong_map)))
        assert not node.results({}, None)["检查"]["定位和地图一致"]
        assert node.results({}, None)["距标定起点_m"] is None


def test_preflight_requires_retained_map_geometry_and_active_localization(arena, map_metadata):
    from lifecycle_msgs.msg import State
    from lifecycle_msgs.srv import GetState
    from nav_msgs.msg import OccupancyGrid
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile

    from alohamini_duel.preflight import LIFECYCLE_NODES, MAP_CHECK, VehicleCheck

    with ExitStack() as resources:
        context = Context()
        context.init(args=[], domain_id=92)
        resources.callback(context.try_shutdown)
        source = Node("map_server", context=context)
        resources.callback(source.destroy_node)
        publisher = source.create_publisher(
            OccupancyGrid, "/map", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )
        grid = OccupancyGrid()
        grid.header.frame_id = "map"
        grid.info.resolution, (grid.info.width, grid.info.height), _ = map_metadata
        grid.info.origin.orientation.w = 1.0
        grid.data = [0]
        publisher.publish(grid)
        node = VehicleCheck(context, arena, ROBOT_IDS[0], False, map_metadata=map_metadata)
        resources.callback(node.destroy_node)
        executor = SingleThreadedExecutor(context=context)
        resources.callback(executor.shutdown)
        executor.add_node(node)
        executor.add_node(source)
        active = GetState.Response(current_state=State(id=State.PRIMARY_STATE_ACTIVE))
        inactive = GetState.Response(current_state=State(id=State.PRIMARY_STATE_INACTIVE))
        states = dict.fromkeys(LIFECYCLE_NODES, active)
        assert not node.results(states, None)["检查"][MAP_CHECK]
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline and not node.results(states, None)["检查"][MAP_CHECK]:
            executor.spin_once(timeout_sec=0.05)
        checks = node.results(states, None)["检查"]
        assert checks[MAP_CHECK]
        assert all(checks[label] for label in LIFECYCLE_NODES.values())
        checks = node.results({"map_server": inactive}, None)["检查"]
        assert not checks[LIFECYCLE_NODES["map_server"]]
        assert not checks[LIFECYCLE_NODES["amcl"]]

        for mismatch in (
            lambda info: setattr(info, "resolution", 0.025),
            lambda info: setattr(info, "width", 2),
            lambda info: setattr(info.origin.position, "y", 0.1),
            lambda info: setattr(info.origin.orientation, "z", 1.0),
        ):
            changed = deepcopy(grid)
            mismatch(changed.info)
            node.on_map(changed)
            assert not node.results(states, None)["检查"][MAP_CHECK]


@pytest.mark.parametrize("limits", [
    [],
    ["--start-position-tolerance-m", "0.1"],
    ["--start-heading-tolerance-deg", "5"],
])
def test_hardware_preflight_requires_both_start_limits(limits, capsys):
    pytest.importorskip("rclpy", reason="需要项目的 ROS 2 运行环境")
    from alohamini_duel.preflight import main

    with pytest.raises(SystemExit) as error:
        main(["--arena", "场地.yaml", "--map", "地图.yaml", *limits])
    assert error.value.code == 2
    assert "真机预检必须指定" in capsys.readouterr().err


def test_preflight_rejects_legacy_bridge_and_invalid_watchdog(arena, map_metadata):
    pytest.importorskip("rclpy", reason="需要项目的 ROS 2 运行环境")
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from rcl_interfaces.srv import GetParameters
    from rclpy.context import Context
    from rclpy.parameter import Parameter

    from alohamini_duel.preflight import BRIDGE_TIMEOUT_CHECK, BRIDGE_TOPIC_CHECK, VehicleCheck

    with ExitStack() as resources:
        context = Context()
        context.init(args=[], domain_id=92)
        resources.callback(context.try_shutdown)
        node = VehicleCheck(context, arena, ROBOT_IDS[0], False, map_metadata=map_metadata)
        resources.callback(node.destroy_node)
        status = DiagnosticStatus(
            name="AlohaMini Host state bridge",
            level=DiagnosticStatus.OK,
            values=[
                KeyValue(key="host_robot_model", value="alohamini2edu"),
                KeyValue(key="command_enabled", value="false"),
                KeyValue(key="stop_pending", value="false"),
            ],
        )
        message = DiagnosticArray(status=[status])
        node.on_diagnostics(message)
        node.on_diagnostics(message)
        checks = node.results({}, None)["检查"]
        assert checks["真机桥接健康且未使能"]
        assert not checks[BRIDGE_TOPIC_CHECK]
        assert not checks[BRIDGE_TIMEOUT_CHECK]

        values = [
            Parameter("base_only", value=True).get_parameter_value(),
            Parameter("cmd_vel_topic", value="/cmd_vel_safe").get_parameter_value(),
        ]
        for timeout, expected in [
            (None, False), (0.15, True), (0.5, False), (0.0, False),
            (float("nan"), False), ("0.15", False),
        ]:
            response = GetParameters.Response(
                values=values + [Parameter("command_timeout_sec", value=timeout).get_parameter_value()]
            )
            assert node.results({}, response)["检查"][BRIDGE_TIMEOUT_CHECK] is expected


@pytest.mark.parametrize("resolved", ["/cmd_vel_safe", "/cmd_vel"])
def test_preflight_reads_real_bridge_parameters_and_resolved_topic(arena, map_metadata, resolved):
    pytest.importorskip("rclpy", reason="需要项目的 ROS 2 运行环境")
    from alohamini_bridge.bridge_node import AlohaMiniBridge
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter

    from alohamini_duel.preflight import BRIDGE_TIMEOUT_CHECK, BRIDGE_TOPIC_CHECK, VehicleCheck

    with ExitStack() as resources:
        context = Context()
        context.init(args=[], domain_id=92)
        resources.callback(context.try_shutdown)
        node = VehicleCheck(context, arena, ROBOT_IDS[0], False, map_metadata=map_metadata)
        resources.callback(node.destroy_node)
        bridge = AlohaMiniBridge(
            context=context,
            parameter_overrides=[
                Parameter("base_only", value=True),
                Parameter("expected_robot_model", value="alohamini2edu"),
                Parameter("cmd_vel_topic", value="/cmd_vel_safe"),
                Parameter("command_timeout_sec", value=0.15),
            ],
            cli_args=[] if resolved == "/cmd_vel_safe" else [
                "--ros-args", "-r", "/cmd_vel_safe:=/cmd_vel",
            ],
        )
        resources.callback(bridge.destroy_node)
        executor = SingleThreadedExecutor(context=context)
        resources.callback(executor.shutdown)
        executor.add_node(bridge)
        executor.add_node(node)
        assert node.bridge_params.wait_for_service(timeout_sec=3)
        _, pending = node.read_services()
        executor.spin_until_future_complete(pending, timeout_sec=3)
        assert pending.done()
        assert not bridge.set_parameters([Parameter("command_timeout_sec", value=0.5)])[0].successful
        expected = resolved == "/cmd_vel_safe"
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
            if node.resolved_bridge_topic == resolved and (
                not expected or node.recent(BRIDGE_TOPIC_CHECK, count=2, max_age=1.5)
            ):
                break
        report = node.results({}, pending.result())
        assert report["桥接解析后速度入口"] == resolved
        assert report["检查"][BRIDGE_TOPIC_CHECK] is expected
        assert report["检查"][BRIDGE_TIMEOUT_CHECK]


def test_fixed_route_loads_map_poses_and_rejects_outside_footprints(arena, tmp_path):
    example = Path(__file__).parents[1] / "config/route_example.yaml"
    assert load_route(example, arena)[0] == pytest.approx((0.9, 0.9, math.pi / 4))
    route = tmp_path / "route.yaml"
    for invalid in ("- [0.05, 0.05, 0.0]", "- [0.75, 1.85, 0.0]", "- [1.5, 1.5, 9.0]"):
        route.write_text(invalid)
        with pytest.raises(ValueError, match="不满足场地净空或朝向范围"):
            load_route(route, arena)


def test_udp_clock_origin_order_restart_and_old_session(arena):
    a, b = (PeerLink(robot, arena) for robot in ROBOT_IDS)
    assert a.online(0) is None
    for index in range(4):
        old = exchange(a, b, index * 0.05)
    assert a.online(0.15).pose == POSE_B
    assert b.online(0.15).pose == POSE_A
    b.accept(old, 0.6)
    assert b.online(0.66) is None

    restarted = PeerLink(a.robot_id, arena)
    for index in range(3):
        exchange(restarted, b, 1.0 + index * 0.05)
    assert b.online(1.1) is None
    exchange(restarted, b, 1.15)
    assert b.online(1.15).session_id == restarted.session_id
    b.accept(old, 2.0)
    assert b.remote.session_id == restarted.session_id
    assert b.online(2.0) is None


def test_one_way_drop_stops_both_and_requires_recent_acknowledgements(arena):
    a, b = (PeerLink(robot, arena) for robot in ROBOT_IDS)
    for index in range(4):
        exchange(a, b, index * 0.05)
    for index in range(4, 16):
        now = index * 0.05
        a.encode(POSE_A, 0.0, 0.0, now)
        a.accept(b.encode(POSE_B, 0.0, 0.0, now), now)
    assert a.remote.received_at == 0.75
    assert a.online(0.75) is None
    assert b.online(0.75) is None
    for index in range(16, 19):
        exchange(a, b, index * 0.05)
    assert a.online(0.9) is None
    exchange(a, b, 0.95)
    assert a.online(0.95) is not None
    assert b.online(0.95) is not None


def test_heartbeat_confirmation_expires_from_original_send_time(arena):
    a, b = (PeerLink(robot, arena) for robot in ROBOT_IDS)
    for index in range(4):
        exchange(a, b, index * 0.05)
    assert a.online(0.249) is not None
    # 最近接收的是 0.15 秒的位姿，确认对应 0.10 秒发送的报文。
    assert a.online(0.251) is None

    b.accept(a.encode(POSE_A, 0.0, 0.0, 0.2), 0.2)
    delayed = b.encode(POSE_B, 0.0, 0.0, 0.2)
    a.accept(delayed, 0.51)
    assert a.remote.received_at == 0.51
    assert a.acked_sent_at == pytest.approx(0.1)
    assert a.online(0.51) is None


def test_receiver_ignores_bad_geometry_and_different_arena(arena):
    a, b = (PeerLink(robot, arena) for robot in ROBOT_IDS)
    valid = json.loads(a.encode(POSE_A, 0.0, 0.0, 0))
    for changes in (
        {"x": -1.0}, {"x": math.nan}, {"seq": True}, {"arena_id": "其他场地"}, {"session_id": []}
    ):
        b.accept(json.dumps(valid | changes).encode(), 0)
        assert b.remote is None
    for bad in (b"[]", b"\xff", b"{" * 1025):
        b.accept(bad, 0)
        assert b.remote is None


def test_udp_socket_receives_only_the_configured_peer(arena):
    a, b = (PeerLink(robot, arena) for robot in ROBOT_IDS)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as remote:
        remote.bind(("127.0.0.1", 0))
        remote.settimeout(1)
        with open_peer_socket("127.0.0.1", 0, *remote.getsockname()) as local:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as unrelated:
                unrelated.sendto("其他端点".encode(), local.getsockname())
            local.send(a.encode(POSE_A, 0.0, 0.0, 2.0))
            data, endpoint = remote.recvfrom(1024)
            b.accept(data, 15)
            assert b.remote.pose == POSE_A
            remote.sendto(b.encode(POSE_B, 0.0, 0.0, 16), endpoint)
            assert select.select([local], [], [], 1)[0]
            a.accept(local.recv(1024), 3)
            assert a.remote.pose == POSE_B


@pytest.fixture
def udp_duel_base(arena, tmp_path):
    rclpy = pytest.importorskip("rclpy", reason="需要项目的 ROS 2 运行环境")
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.signals import SignalHandlerOptions

    from alohamini_duel.base_node import DuelBase

    config = Path(__file__).parents[1] / "config/arena_example.yaml"
    with ExitStack() as resources:
        remote = resources.enter_context(socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
        remote.bind(("127.0.0.1", 0))
        remote.settimeout(1)
        rclpy.init(
            args=[
                "--ros-args", "-p", f"arena:={config}", "-p", f"map:={tmp_path / 'map.yaml'}",
                "-p", f"robot_id:={ROBOT_IDS[0]}", "-p", "bind_ip:=127.0.0.1",
                "-p", "bind_port:=0", "-p", "peer_ip:=127.0.0.1",
                "-p", f"peer_port:={remote.getsockname()[1]}",
            ],
            domain_id=91,
            signal_handler_options=SignalHandlerOptions.NO,
        )
        resources.callback(rclpy.try_shutdown)
        base = DuelBase()
        resources.callback(base.destroy_node)
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = "map", "base_link"
        transform.transform.translation.x, transform.transform.translation.y = POSE_A[:2]
        transform.transform.rotation.w = 1.0
        odometry = Odometry()

        def refresh_pose():
            transform.header.stamp = odometry.header.stamp = base.get_clock().now().to_msg()
            base.tf.set_transform(transform, "心跳测试")
            base._odom(odometry)

        refresh_pose()
        yield base, remote, refresh_pose


@pytest.mark.parametrize("burst", [1, 32])
def test_heartbeat_tick_acknowledges_latest_packet_already_in_socket(arena, udp_duel_base, burst):
    base, remote, _ = udp_duel_base
    peer = PeerLink(ROBOT_IDS[1], arena)
    for _ in range(burst):
        remote.sendto(peer.encode(POSE_B, 0.0, 0.0, 1.0), base.udp.getsockname())
    assert select.select([base.udp], [], [], 1)[0]
    base._tick()
    heartbeat = json.loads(remote.recv(1024))
    assert heartbeat["ack_session_id"] == peer.session_id
    assert heartbeat["ack_seq"] == peer.seq


def exchange_with_base(base, remote, refresh_pose, peer):
    refresh_pose()
    remote.sendto(peer.encode(POSE_B, 0.0, 0.0, time.monotonic()), base.udp.getsockname())
    assert select.select([base.udp], [], [], 1)[0]
    base._tick()
    peer.accept(remote.recv(1024), time.monotonic())


def scan_sample(base, *, age=0.0, points=11):
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan

    message = LaserScan(range_min=0.15, range_max=8.0, ranges=[1.0] * points)
    message.header.frame_id = "laser_frame"
    message.header.stamp = Time(
        nanoseconds=base.get_clock().now().nanoseconds - round(age * 1e9)
    ).to_msg()
    return message


@pytest.mark.parametrize(("source", "lose_pose"), [("duel", False), ("nav", True)])
def test_base_revokes_motion_permission_until_explicit_reenable(
    arena, udp_duel_base, source, lose_pose,
):
    from geometry_msgs.msg import Twist
    from std_srvs.srv import SetBool

    base, remote, refresh_pose = udp_duel_base
    peer = PeerLink(ROBOT_IDS[1], arena)
    request = SetBool.Request(data=True)
    move = Twist()
    move.linear.x = 0.06

    base._scan(scan_sample(base))
    assert not base._enable(request, SetBool.Response()).success
    for _ in range(5):
        exchange_with_base(base, remote, refresh_pose, peer)
    assert base.online and base._enable(request, SetBool.Response()).success
    base._command(source, move)
    assert base.arbiter.select(POSE_A, base.link.remote, time.monotonic()) != STOP

    if lose_pose:
        base.odometry = None
    if not lose_pose:
        base.link.acked_sent_at = time.monotonic() - 0.17
    assert not base._enable(request, SetBool.Response()).success
    base._tick()
    if not lose_pose:
        peer.accept(remote.recv(1024), time.monotonic())
    assert not base.online and not base.arbiter.enabled

    for _ in range(5):
        base._command(source, move)
        exchange_with_base(base, remote, refresh_pose, peer)
        assert base.arbiter.select(POSE_A, base.link.remote, time.monotonic()) == STOP
    assert base.online and not base.arbiter.enabled
    assert base._enable(request, SetBool.Response()).success
    assert base.arbiter.select(POSE_A, base.link.remote, time.monotonic()) == STOP
    base._command(source, move)
    assert base.arbiter.select(POSE_A, base.link.remote, time.monotonic()) != STOP


@pytest.mark.parametrize(
    ("source", "fault"),
    [("duel", "停止到包"), ("nav", "旧时戳"), ("nav", "有效回波不足")],
)
def test_base_requires_fresh_scan_and_revokes_motion(arena, udp_duel_base, source, fault):
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from std_srvs.srv import SetBool

    base, remote, refresh_pose = udp_duel_base
    peer = PeerLink(ROBOT_IDS[1], arena)
    enable = SetBool.Request(data=True)
    for _ in range(5):
        exchange_with_base(base, remote, refresh_pose, peer)
    assert base.online
    assert not base._enable(enable, SetBool.Response()).success

    for timer in base.timers:
        timer.cancel()
    with ExitStack() as resources:
        probe = rclpy.create_node("duel_scan_guard_probe", use_global_arguments=False)
        resources.callback(probe.destroy_node)
        publisher = probe.create_publisher(LaserScan, "/scan_filtered", qos_profile_sensor_data)
        outputs = []
        probe.create_subscription(Twist, "/cmd_vel", outputs.append, 1)
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline and (
            not publisher.get_subscription_count() or not base.velocity_pub.get_subscription_count()
        ):
            rclpy.spin_once(probe, timeout_sec=0.01)
        assert publisher.get_subscription_count() == 1
        assert base.velocity_pub.get_subscription_count() == 1

        def publish_scan(age=0.0, points=11):
            publisher.publish(scan_sample(base, age=age, points=points))
            rclpy.spin_once(base, timeout_sec=0.1)

        def check_output(speed):
            outputs.clear()
            exchange_with_base(base, remote, refresh_pose, peer)
            deadline = time.monotonic() + 0.1
            while time.monotonic() < deadline and not outputs:
                rclpy.spin_once(probe, timeout_sec=0.01)
            assert outputs and outputs[-1].linear.x == pytest.approx(speed)
            assert base.online

        publish_scan(age=0.14)
        exchange_with_base(base, remote, refresh_pose, peer)
        assert base._enable(enable, SetBool.Response()).success
        move = Twist()
        move.linear.x = 0.06
        base._command(source, move)
        check_output(0.06)

        if fault == "停止到包":
            header, _ = base.scan
            base.scan = header, time.monotonic() - 0.22
        if fault == "旧时戳":
            publish_scan(age=0.31)
        if fault == "有效回波不足":
            publish_scan(points=10)
        base._command(source, move)
        check_output(0.0)
        assert not base.arbiter.enabled
        assert not base._enable(enable, SetBool.Response()).success

        publish_scan()
        base._command(source, move)
        check_output(0.0)
        assert base._enable(enable, SetBool.Response()).success
        check_output(0.0)
        base._command(source, move)
        check_output(0.06)


def test_arbitration_priority_timeout_and_stop_recovery(arena):
    arbiter = VelocityArbiter(arena)
    peer = PeerState("会话", 1, POSE_B, 0, 0)
    nav = BodyVelocity(0.06, 0.0, 0.0)
    duel = BodyVelocity(0.0, -0.05, 0.0)
    arbiter.submit("nav", nav, 0)
    assert arbiter.select(POSE_A, peer, 0) == STOP
    arbiter.enable(True)
    arbiter.submit("nav", nav, 0)
    assert arbiter.select(POSE_A, peer, 0) == nav
    arbiter.submit("duel", duel, 0.01)
    assert arbiter.select(POSE_A, peer, 0.02) == duel
    arbiter.submit("nav", nav, 0.1)
    assert arbiter.select(POSE_A, peer, 0.17) == nav
    assert arbiter.select(POSE_A, None, 0.18) == STOP
    assert arbiter.select(POSE_A, peer, 0.19) == STOP
    arbiter.submit("duel", duel, 0.2)
    assert arbiter.select(None, peer, 0.21) == STOP
    assert arbiter.select(POSE_A, peer, 0.22) == STOP
    arbiter.submit("duel", duel, 0.24)
    assert arbiter.select(POSE_A, peer, 0.40) == STOP


def test_geometry_stops_at_wall_and_when_lateral_motion_closes_gap(arena):
    arbiter = VelocityArbiter(arena)
    arbiter.enable(True)
    arbiter.submit("duel", BodyVelocity(-0.1, 0, 0), 0)
    peer = PeerState("会话", 1, POSE_B, 0, 0)
    assert arbiter.select((0.47, 0.62, 0.0), peer, 0.0) == STOP
    arbiter.submit("duel", BodyVelocity(0.0, 0.1, 0), 0)
    close_peer = PeerState("会话", 1, (1.45, 2.36, 0), 0, 0)
    assert arbiter.select(POSE_A, close_peer, 0.0) == STOP
    arbiter.submit("duel", BodyVelocity(0.3, 0.3, 0), 0)
    command = arbiter.select(POSE_A, peer, 0.0)
    assert math.hypot(command.x_m_s, command.y_m_s) == pytest.approx(MAX_SPEED)


def test_geometry_budgets_tf_and_communication_age_even_for_newly_received_pose(arena):
    arbiter = VelocityArbiter(arena)
    velocity = BodyVelocity(0.0, MAX_SPEED, 0.0)
    peer = PeerState("会话", 2, (1.45, 2.42, 0.0), 1.0, 1.0)
    assert arbiter.limit(velocity, POSE_A, peer, 1.0) == velocity
    assert arbiter.limit(velocity, POSE_A, replace(peer, pose=(1.45, 2.4, 0.0)), 1.0) == STOP
    assert arbiter.limit(velocity, POSE_A, replace(peer, confirmed_send_at=0.8), 1.0) == STOP
