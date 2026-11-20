import json
import math
import select
import socket
from contextlib import ExitStack
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
    assert a.online(0.39) is not None
    # 最近接收的是 0.15 秒的位姿，确认对应 0.10 秒发送的报文。
    assert a.online(0.41) is None

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


def test_heartbeat_tick_acknowledges_packet_already_in_socket(arena, tmp_path):
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
        stamp = base.get_clock().now().to_msg()
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = "map", "base_link"
        transform.header.stamp = stamp
        transform.transform.translation.x, transform.transform.translation.y = POSE_A[:2]
        transform.transform.rotation.w = 1.0
        base.tf.set_transform(transform, "心跳测试")
        odometry = Odometry()
        odometry.header.stamp = stamp
        base._odom(odometry)
        peer = PeerLink(ROBOT_IDS[1], arena)
        remote.sendto(peer.encode(POSE_B, 0.0, 0.0, 1.0), base.udp.getsockname())
        assert select.select([base.udp], [], [], 1)[0]
        base._tick()
        heartbeat = json.loads(remote.recv(1024))
        assert heartbeat["ack_session_id"] == peer.session_id
        assert heartbeat["ack_seq"] == peer.seq


def test_arbitration_priority_timeout_and_stop_recovery(arena):
    arbiter = VelocityArbiter(arena)
    peer = PeerState("会话", 1, POSE_B, 0)
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
    peer = PeerState("会话", 1, POSE_B, 0)
    assert arbiter.select((0.47, 0.62, 0.0), peer, 0.0) == STOP
    arbiter.submit("duel", BodyVelocity(0.0, 0.1, 0), 0)
    close_peer = PeerState("会话", 1, (1.45, 2.36, 0), 0)
    assert arbiter.select(POSE_A, close_peer, 0.0) == STOP
    arbiter.submit("duel", BodyVelocity(0.3, 0.3, 0), 0)
    command = arbiter.select(POSE_A, peer, 0.0)
    assert math.hypot(command.x_m_s, command.y_m_s) == pytest.approx(MAX_SPEED)
