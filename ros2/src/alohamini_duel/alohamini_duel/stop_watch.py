"""记录双车联机状态下降沿、仲裁零速和轮速停稳。"""

import json
import math
import sys
import time
from contextlib import ExitStack
from dataclasses import dataclass, field
from pathlib import Path

from .arena import ROBOT_IDS, Arena
from .cmd_vel_arbiter import COMMAND_TIMEOUT
from .fault_event import STOP_DEADLINE_S, FaultEvent, recorder_boot_id
from .peer_sync import PEER_TIMEOUT
from .status import VehicleStatus, fresh_stamp, wheel_is_stationary

WHEEL_TIMEOUT = 0.25
STOPPED_SAMPLES = 5


@dataclass
class StopObservation:
    status: VehicleStatus | None = None
    status_at: float = -math.inf
    wheel_at: float = -math.inf
    last_wheel_stamp: int = 0
    last_moving_at: float = -math.inf
    command_at: float = -math.inf
    has_zero_command: bool = False
    edge: float | None = None
    zero_at: float | None = None
    zero_since: float | None = None
    stationary_since: float | None = None
    stopped_at: float | None = None
    stopped_count: int = 0
    moved_before_edge: bool = False
    interrupted: bool = False
    # 故障标记可能晚于轮速回调到达，按标记时间回查最后一帧反馈。
    wheel_history: list[tuple[float, bool | None]] = field(default_factory=list)

    def observe_status(self, status: VehicleStatus, now: float):
        previous = self.status
        same_session = previous is not None and previous.session_id == status.session_id
        if same_session and status.seq <= previous.seq:
            return
        if self.edge is not None and (not same_session or status.online or status.pose is None):
            self.interrupted = True
        if (
            self.edge is None
            and same_session
            and previous.online
            and self.ready(now)
            and not status.online
            and status.pose is not None
        ):
            self.edge = now
            self.moved_before_edge = now - self.last_moving_at <= PEER_TIMEOUT
            if self.has_zero_command and now - self.command_at <= COMMAND_TIMEOUT:
                self.zero_at = now
        self.status, self.status_at = status, now

    def observe_command(self, zero: bool, now: float):
        if not zero or now - self.command_at > COMMAND_TIMEOUT:
            self.zero_since = None
        if zero and self.zero_since is None:
            self.zero_since = now
        if self.edge is not None:
            if not zero or now - self.command_at > COMMAND_TIMEOUT:
                self.zero_at = None
            if zero and self.zero_at is None:
                self.zero_at = now
        self.has_zero_command, self.command_at = zero, now

    def observe_wheel(self, stamp: int | None, stationary: bool, now: float):
        if stamp is None:
            self.wheel_history.append((now, None))
            self.stopped_count, self.stopped_at = 0, None
            self.stationary_since = None
            self.wheel_at = -math.inf
            return
        if stamp <= self.last_wheel_stamp:
            return
        self.wheel_history.append((now, stationary))
        if not stationary:
            self.last_moving_at = now
        if now - self.wheel_at > WHEEL_TIMEOUT or not stationary:
            self.stopped_count, self.stopped_at = 0, None
            self.stationary_since = None
        self.last_wheel_stamp, self.wheel_at = stamp, now
        if stationary and self.stationary_since is None:
            self.stationary_since = now
        if self.edge is not None and stationary:
            self.stopped_count += 1
            if self.stopped_count == STOPPED_SAMPLES:
                self.stopped_at = now

    def ready(self, now):
        return (
            self.status is not None
            and self.status.online
            and now - self.status_at <= PEER_TIMEOUT
            and now - self.command_at <= COMMAND_TIMEOUT
            and now - self.wheel_at <= WHEEL_TIMEOUT
        )

    def completed(self, now):
        return (
            self.edge is not None and not self.interrupted
            and self.zero_at is not None and now - self.command_at <= COMMAND_TIMEOUT
            and self.stopped_at is not None and now - self.wheel_at <= WHEEL_TIMEOUT
            and now - self.status_at <= PEER_TIMEOUT
        )

    def results(self, started, now):
        return {
            "检测断联_s": self.edge - started if self.edge is not None else None,
            "检测到零速_s": self.zero_at - self.edge if self.zero_at is not None else None,
            "检测到停稳_s": self.stopped_at - self.edge if self.stopped_at is not None else None,
            "断联前轮速显示运动": self.moved_before_edge,
            "记录完整": self.completed(now),
        }

    def fault_timing(self, event, now):
        triggered = event.triggered_at
        if not self.completed(now) or self.edge < triggered:
            return None
        last_wheel_at, was_stationary = next(
            ((at, stopped) for at, stopped in reversed(self.wheel_history) if at <= triggered),
            (-math.inf, None),
        )
        fresh_wheel = triggered - last_wheel_at <= WHEEL_TIMEOUT
        return {
            "故障到检测断联_s": self.edge - triggered,
            "故障到观测零速_s": max(self.zero_since - triggered, 0.0),
            "故障到轮速停稳_s": max(self.stationary_since - triggered, 0.0),
            "故障到停稳确认_s": self.stopped_at - triggered,
            "故障时已观测静止": fresh_wheel and was_stationary is True,
            "故障前轮速显示运动": fresh_wheel and was_stationary is False,
        }


def mark_report(report, observations, event, marker_error, now):
    report["可用于0.5秒验收"] = False
    report["0.5秒内停稳"] = None
    if event is None:
        report["失败原因"] = marker_error or "未获得故障触发标记"
        return False
    report["故障标记"] = {"来源": event.source, "单调时间_ns": event.triggered_ns}
    timings = {robot: item.fault_timing(event, now) for robot, item in observations.items()}
    for robot, timing in timings.items():
        if timing is not None:
            report["双车"][robot].update(timing)
    if not report["记录完整"]:
        report["失败原因"] = "双车停车证据不完整"
        return False
    if any(timing is None for timing in timings.values()):
        report["失败原因"] = "故障标记时间晚于联机下降沿"
        return False
    if not any(timing["故障前轮速显示运动"] for timing in timings.values()):
        report["失败原因"] = "故障触发时刻未观察到运动"
        return False
    if not report["断联前观察到运动"]:
        report["失败原因"] = "故障前未观察到运动"
        return False
    worst = max(timing["故障到轮速停稳_s"] for timing in timings.values())
    report["最迟轮速停稳_s"] = worst
    report["可用于0.5秒验收"] = True
    report["0.5秒内停稳"] = worst <= STOP_DEADLINE_S
    if not report["0.5秒内停稳"]:
        report["失败原因"] = "最迟轮速停稳超过 0.5 秒"
    return report["0.5秒内停稳"]


def main(args=None):
    from geometry_msgs.msg import Twist, TwistWithCovarianceStamped
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String

    from .ros_domains import attach_domain, dual_domain_parser, parse_dual_domain_args, watch_shutdown

    parser = dual_domain_parser(__doc__, (5, 6))
    parser.add_argument("--duration", type=float, default=30.0, help="最大记录秒数")
    parser.add_argument("--simulation", action="store_true", help="读取仿真时钟")
    parser.add_argument("--fault-event", type=Path, help="由本机故障触发器写入的新标记文件")
    arguments = parse_dual_domain_args(parser, args)
    if not math.isfinite(arguments.duration) or arguments.duration <= 0:
        parser.error("记录时间必须是有限正数")
    marker = arguments.fault_event
    if marker is not None:
        if marker.exists():
            parser.error("故障标记路径必须尚未存在")
        try:
            boot_id = recorder_boot_id()
        except OSError as error:
            parser.error(f"无法读取记录机启动标识：{error}")
    arena = Arena.load(arguments.arena, arguments.map)
    observations = {robot_id: StopObservation() for robot_id in ROBOT_IDS}

    def bind(node, robot_id):
        observation = observations[robot_id]

        def on_status(message):
            status = VehicleStatus.decode(message.data, robot_id, arena)
            if status is not None:
                observation.observe_status(status, time.monotonic())

        def on_wheel(message):
            twist = message.twist.twist
            stamp = fresh_stamp(
                message.header, "base_link", node.get_clock().now().nanoseconds, WHEEL_TIMEOUT
            )
            if not all(math.isfinite(v) for v in (twist.linear.x, twist.linear.y, twist.angular.z)):
                stamp = None
            observation.observe_wheel(
                stamp, wheel_is_stationary(twist), time.monotonic()
            )

        node.create_subscription(String, "/duel/local_status", on_status, qos_profile_sensor_data)
        node.create_subscription(
            Twist, "/cmd_vel",
            lambda msg: observation.observe_command(msg == Twist(), time.monotonic()),
            qos_profile_sensor_data,
        )
        node.create_subscription(
            TwistWithCovarianceStamped, "/wheel/twist", on_wheel, qos_profile_sensor_data
        )

    started = time.monotonic()
    ready_at = None
    event = None
    marker_error = None
    with ExitStack() as resources:
        shutdown_requested = watch_shutdown(resources)
        executors = []
        for robot_id, domain in zip(ROBOT_IDS, arguments.domains, strict=True):
            node, executor = attach_domain(
                resources, domain,
                lambda context, robot=robot_id: Node(
                    f"duel_stop_watch_{robot}", context=context,
                    parameter_overrides=[Parameter("use_sim_time", value=arguments.simulation)],
                ),
            )
            bind(node, robot_id)
            executors.append(executor)
        while time.monotonic() - started < arguments.duration and not shutdown_requested():
            for executor in executors:
                executor.spin_once(timeout_sec=0.01)
            now = time.monotonic()
            if ready_at is None and all(item.ready(now) for item in observations.values()):
                print("已收到双车在线状态与轮速，等待断联事件", file=sys.stderr, flush=True)
                ready_at = now
            if marker is not None and event is None and marker.exists():
                try:
                    recorded = FaultEvent.read(marker, boot_id)
                    if ready_at is None or not ready_at <= recorded.triggered_at <= time.monotonic():
                        raise ValueError("故障触发时刻不在本次记录就绪后的有效时间内")
                    event = recorded
                except (OSError, ValueError) as error:
                    marker_error = str(error)
                    break
            if (marker is None or event is not None) and all(
                item.completed(now) for item in observations.values()
            ):
                break
        now = time.monotonic()
        results = {
            robot_id: observations[robot_id].results(started, now) | {"ROS域": domain}
            for robot_id, domain in zip(ROBOT_IDS, arguments.domains, strict=True)
        }
    complete = all(item["记录完整"] for item in results.values())
    motion_observed = any(item["断联前轮速显示运动"] for item in results.values())
    report = {
        "记录完整": complete, "断联前观察到运动": motion_observed,
        "可用于0.5秒验收": False, "双车": results,
    }
    report["通过"] = (
        mark_report(report, observations, event, marker_error, now)
        if marker is not None else complete and motion_observed
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["通过"] else 1
