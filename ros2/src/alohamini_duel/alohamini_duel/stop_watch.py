"""记录双车联机状态下降沿、仲裁零速和轮速停稳。"""

import json
import math
import sys
import time
from contextlib import ExitStack
from dataclasses import dataclass

from .arena import ROBOT_IDS, Arena
from .cmd_vel_arbiter import COMMAND_TIMEOUT
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
    stopped_at: float | None = None
    stopped_count: int = 0
    moved_before_edge: bool = False
    interrupted: bool = False

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
        if self.edge is not None:
            if not zero or now - self.command_at > COMMAND_TIMEOUT:
                self.zero_at = None
            if zero and self.zero_at is None:
                self.zero_at = now
        self.has_zero_command, self.command_at = zero, now

    def observe_wheel(self, stamp: int | None, stationary: bool, now: float):
        if stamp is None:
            self.stopped_count, self.stopped_at = 0, None
            self.wheel_at = -math.inf
            return
        if stamp <= self.last_wheel_stamp:
            return
        if not stationary:
            self.last_moving_at = now
        if now - self.wheel_at > WHEEL_TIMEOUT or not stationary:
            self.stopped_count, self.stopped_at = 0, None
        self.last_wheel_stamp, self.wheel_at = stamp, now
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


def main(args=None):
    from geometry_msgs.msg import Twist, TwistWithCovarianceStamped
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String

    from .ros_domains import attach_domain, dual_domain_parser, parse_dual_domain_args

    parser = dual_domain_parser(__doc__, (5, 6))
    parser.add_argument("--duration", type=float, default=30.0, help="最大记录秒数")
    parser.add_argument("--simulation", action="store_true", help="读取仿真时钟")
    arguments = parse_dual_domain_args(parser, args)
    if not math.isfinite(arguments.duration) or arguments.duration <= 0:
        parser.error("记录时间必须是有限正数")
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
    announced = False
    with ExitStack() as resources:
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
        while time.monotonic() - started < arguments.duration:
            for executor in executors:
                executor.spin_once(timeout_sec=0.01)
            now = time.monotonic()
            if not announced and all(item.ready(now) for item in observations.values()):
                print("已收到双车在线状态与轮速，等待断联事件", file=sys.stderr, flush=True)
                announced = True
            if all(item.completed(now) for item in observations.values()):
                break
        now = time.monotonic()
        results = {
            robot_id: observations[robot_id].results(started, now) | {"ROS域": domain}
            for robot_id, domain in zip(ROBOT_IDS, arguments.domains, strict=True)
        }
    complete = all(item["记录完整"] for item in results.values())
    motion_observed = any(item["断联前轮速显示运动"] for item in results.values())
    print(
        json.dumps(
            {"记录完整": complete, "断联前观察到运动": motion_observed, "双车": results},
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0 if complete and motion_observed else 1
