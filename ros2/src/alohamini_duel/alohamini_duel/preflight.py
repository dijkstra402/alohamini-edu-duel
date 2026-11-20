"""只读核验两车在现场起点的定位、传感器和速度链路。"""

import json
import math
import time
from contextlib import ExitStack

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String

from .arena import BASE_RADIUS, ROBOT_IDS, Arena
from .ros_domains import attach_domain, dual_domain_parser, parse_dual_domain_args
from .status import VehicleStatus, fresh_stamp, wheel_is_stationary

BRIDGE = "alohamini_lerobot_bridge"
ARBITER = "alohamini_duel_base"
MONITOR = "collision_monitor"
SIMULATOR = "alohamini_nav_sim"


class VehicleCheck(Node):
    def __init__(self, context, arena, robot_id, simulation):
        super().__init__(
            f"duel_check_{robot_id}",
            context=context,
            parameter_overrides=[Parameter("use_sim_time", value=simulation)],
        )
        self.arena, self.robot_id = arena, robot_id
        self.samples: dict[str, tuple[int, float]] = {}
        self.status: VehicleStatus | None = None
        self.safe_velocity_samples = 0
        self.nonzero_safe_velocity = False
        for kind, topic, callback in (
            (String, "/duel/local_status", self.on_status),
            (LaserScan, "/scan_filtered", self.on_scan),
            (Imu, "/imu/data_raw", self.on_imu),
            (TwistWithCovarianceStamped, "/wheel/twist", self.on_wheel),
            (Twist, "/cmd_vel", lambda msg: self.record("仲裁输出零速", msg == Twist())),
            (Twist, "/cmd_vel_safe", self.on_safe_velocity),
        ):
            self.create_subscription(kind, topic, callback, qos_profile_sensor_data)
        self.lifecycle = self.create_client(GetState, f"/{MONITOR}/get_state")
        self.bridge_params = None
        if not simulation:
            self.create_subscription(
                DiagnosticArray, "/diagnostics", self.on_diagnostics, qos_profile_sensor_data
            )
            self.bridge_params = self.create_client(GetParameters, f"/{BRIDGE}/get_parameters")

    def record(self, name, ok, *, max_gap=0.5):
        now = time.monotonic()
        count, received = self.samples.get(name, (0, now))
        if now - received > max_gap:
            count = 0
        self.samples[name] = (count + 1 if ok else 0, now)

    def recent(self, name, *, count=3, max_age=0.5):
        samples, received = self.samples.get(name, (0, -math.inf))
        return samples >= count and time.monotonic() - received <= max_age

    def current_stamp(self, header, frame, max_age=0.5):
        return fresh_stamp(header, frame, self.get_clock().now().nanoseconds, max_age) is not None

    def on_status(self, message):
        status = VehicleStatus.decode(message.data, self.robot_id, self.arena)
        if status is None:
            self.status = None
            self.record("定位和地图一致", False)
            self.record("双向心跳", False)
            self.record("仲裁未使能", False)
            return
        if (
            self.status is not None
            and status.session_id == self.status.session_id
            and status.seq <= self.status.seq
        ):
            return
        self.status = status
        self.record("定位和地图一致", status.pose is not None)
        self.record("双向心跳", status.online)
        self.record("仲裁未使能", status.motion_enabled is False)

    def on_scan(self, message):
        valid = sum(
            math.isfinite(value) and message.range_min <= value <= message.range_max
            for value in message.ranges
        )
        self.record("新鲜雷达", self.current_stamp(message.header, "laser_frame") and valid > 10)

    def on_imu(self, message):
        valid = self.current_stamp(message.header, "imu_link")
        self.record("新鲜惯导", valid and math.isfinite(message.angular_velocity.z))

    def on_wheel(self, message):
        speed = message.twist.twist
        valid = self.current_stamp(message.header, "base_link", max_age=0.25)
        self.record("底盘反馈静止", valid and wheel_is_stationary(speed))

    def on_safe_velocity(self, message):
        # 碰撞监测超过 stop_pub_timeout 后可以停止重复发送零速。
        self.safe_velocity_samples += 1
        self.nonzero_safe_velocity |= message != Twist()

    def on_diagnostics(self, message):
        for status in message.status:
            if status.name != "AlohaMini Host state bridge":
                continue
            values = {item.key: item.value for item in status.values}
            ok = (
                status.level == DiagnosticStatus.OK
                and values.get("host_robot_model") == "alohamini2edu"
                and values.get("command_enabled") == values.get("stop_pending") == "false"
            )
            self.record("真机桥接健康且未使能", ok, max_gap=1.5)

    def read_services(self):
        lifecycle = (
            self.lifecycle.call_async(GetState.Request())
            if self.lifecycle.service_is_ready()
            else None
        )
        parameters = (
            self.bridge_params.call_async(
                GetParameters.Request(names=["base_only", "cmd_vel_topic"])
            )
            if self.bridge_params is not None and self.bridge_params.service_is_ready()
            else None
        )
        return lifecycle, parameters

    def results(self, lifecycle, parameters):
        checks = {
            name: self.recent(name)
            for name in (
                "定位和地图一致", "双向心跳", "仲裁未使能", "新鲜雷达", "新鲜惯导",
                "底盘反馈静止", "仲裁输出零速",
            )
        }
        cmd_publishers = [item.node_name for item in self.get_publishers_info_by_topic("/cmd_vel")]
        safe_publishers = [
            item.node_name for item in self.get_publishers_info_by_topic("/cmd_vel_safe")
        ]
        cmd_consumers = [
            item.node_name for item in self.get_subscriptions_info_by_topic("/cmd_vel")
        ]
        active = lifecycle is not None and lifecycle.current_state.id == State.PRIMARY_STATE_ACTIVE
        checks["碰撞监测处于活动状态"] = active
        checks["监测出口未出现非零指令"] = not self.nonzero_safe_velocity
        checks["速度链路只有一个仲裁和一个监测出口"] = (
            cmd_publishers == [ARBITER]
            and safe_publishers == [MONITOR]
            and MONITOR in cmd_consumers
            and BRIDGE not in cmd_consumers
            and SIMULATOR not in cmd_consumers
        )
        if self.bridge_params is not None:
            checks["真机桥接健康且未使能"] = self.recent(
                "真机桥接健康且未使能", count=2, max_age=1.5
            )
            checks["真机桥接使用底盘模式和监测出口"] = (
                parameters is not None
                and len(parameters.values) == 2
                and parameters.values[0].bool_value is True
                and parameters.values[1].string_value == "/cmd_vel_safe"
            )
        if self.bridge_params is None:
            sinks = self.get_subscriptions_info_by_topic("/cmd_vel_safe")
            checks["仿真底盘订阅监测出口"] = any(
                item.node_name == SIMULATOR for item in sinks
            )
        pose = self.status.pose if checks["定位和地图一致"] else None
        checks["本车名义净空"] = pose is not None and self.arena.path_clear(
            pose[:2], pose[:2], BASE_RADIUS
        )
        return {
            "检查": checks,
            "位姿": pose,
            "距标定起点_m": (
                math.dist(pose[:2], self.arena.starts[self.robot_id][:2]) if pose else None
            ),
            "速度源": cmd_publishers,
            "安全速度源": safe_publishers,
            "安全速度采样数": self.safe_velocity_samples,
        }


def main(args=None):
    parser = dual_domain_parser(__doc__, (5, 6))
    parser.add_argument("--duration", type=float, default=5.0, help="采样秒数")
    parser.add_argument("--simulation", action="store_true", help="读取仿真时钟")
    arguments = parse_dual_domain_args(parser, args)
    if not math.isfinite(arguments.duration) or arguments.duration <= 0:
        parser.error("采样时间必须是有限正数")
    arena = Arena.load(arguments.arena, arguments.map)
    with ExitStack() as resources:
        endpoints = [
            attach_domain(
                resources,
                domain,
                lambda context, robot=robot: VehicleCheck(
                    context, arena, robot, arguments.simulation
                ),
            )
            for robot, domain in zip(ROBOT_IDS, arguments.domains, strict=True)
        ]
        nodes, executors = zip(*endpoints, strict=True)
        deadline = time.monotonic() + arguments.duration
        while time.monotonic() < deadline:
            for executor in executors:
                executor.spin_once(timeout_sec=0.02)
        pending = [node.read_services() for node in nodes]
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline and any(
            future is not None and not future.done() for group in pending for future in group
        ):
            for executor in executors:
                executor.spin_once(timeout_sec=0.02)
        results = {}
        for node, domain, futures in zip(nodes, arguments.domains, pending, strict=True):
            replies = [
                future.result()
                if future is not None and future.done() and future.exception() is None
                else None
                for future in futures
            ]
            results[node.robot_id] = node.results(*replies) | {"ROS域": domain}
    first, second = (item["位姿"] for item in results.values())
    separation = math.dist(first[:2], second[:2]) if first and second else None
    clear = separation is not None and separation > 2 * BASE_RADIUS
    passed = clear and all(all(value["检查"].values()) for value in results.values())
    print(
        json.dumps(
            {"通过": passed, "名义双车净空": clear, "中心距_m": separation, "双车": results},
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0 if passed else 1
