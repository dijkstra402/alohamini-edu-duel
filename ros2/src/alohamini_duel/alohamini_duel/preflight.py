"""只读核验两车在现场起点的定位、传感器和速度链路。"""

import json
import math
import struct
import time
from contextlib import ExitStack
from pathlib import Path

import yaml
from PIL import Image
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String

from .arena import BASE_RADIUS, ROBOT_IDS, Arena, coordinates
from .cmd_vel_arbiter import COMMAND_TIMEOUT
from .ros_domains import attach_domain, dual_domain_parser, parse_dual_domain_args
from .status import (
    MAX_SCAN_AGE,
    SCAN_TIMEOUT,
    VehicleStatus,
    angle_error,
    fresh_stamp,
    scan_has_valid_ranges,
    wheel_is_stationary,
    yaw_from_quaternion,
)

BRIDGE = "alohamini_lerobot_bridge"
ARBITER = "alohamini_duel_base"
MONITOR = "collision_monitor"
SIMULATOR = "alohamini_nav_sim"
POSITION_CHECK = "起点位置偏差在限值内"
HEADING_CHECK = "起点朝向偏差在限值内"
BRIDGE_TOPIC_CHECK = "真机桥接解析后的速度入口正确"
BRIDGE_TIMEOUT_CHECK = "真机桥接速度看门狗在限值内"
CLEARANCE_CHECK = "本车名义净空"
MAP_CHECK = "实时栅格元数据与地图文件一致"
LIFECYCLE_NODES = {
    MONITOR: "碰撞监测处于活动状态", "map_server": "地图服务器处于活动状态",
    "amcl": "定位节点处于活动状态",
}
STATUS_CHECKS = (
    "定位和地图一致", "双向心跳", "仲裁未使能", POSITION_CHECK, HEADING_CHECK, CLEARANCE_CHECK,
)


def load_map_metadata(path):
    path = Path(path)
    data = yaml.safe_load(path.read_bytes())
    resolution, = coordinates([data.get("resolution")], 1, "地图分辨率")
    if resolution <= 0:
        raise ValueError("地图分辨率必须为正数")
    # OccupancyGrid 的分辨率以 float32 表示。
    resolution, = struct.unpack("f", struct.pack("f", resolution))
    with Image.open(path.parent / data["image"]) as image:
        return resolution, image.size, coordinates(data.get("origin"), 3, "地图原点")


class VehicleCheck(Node):
    def __init__(
        self, context, arena, robot_id, simulation, *, map_metadata,
        position_limit=None, heading_limit=None,
    ):
        super().__init__(
            f"duel_check_{robot_id}",
            context=context,
            parameter_overrides=[Parameter("use_sim_time", value=simulation)],
        )
        self.arena, self.robot_id = arena, robot_id
        self.map_metadata = map_metadata
        self.map_matches = False
        self.position_limit, self.heading_limit = position_limit, heading_limit
        self.samples: dict[str, tuple[int, float]] = {}
        self.status: VehicleStatus | None = None
        self.resolved_bridge_topic: str | None = None
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
        self.create_subscription(
            OccupancyGrid, "/map", self.on_map,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.lifecycle_clients = {
            name: self.create_client(GetState, f"/{name}/get_state") for name in LIFECYCLE_NODES
        }
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

    def start_errors(self, pose):
        if pose is None:
            return None, None
        start = self.arena.starts[self.robot_id]
        return math.dist(pose[:2], start[:2]), math.degrees(angle_error(pose[2], start[2]))

    def on_status(self, message):
        status = VehicleStatus.decode(message.data, self.robot_id, self.arena)
        if status is None:
            self.status = None
            for name in STATUS_CHECKS:
                self.samples.pop(name, None)
            return
        if (
            self.status is not None
            and status.session_id == self.status.session_id
            and status.seq <= self.status.seq
        ):
            return
        if self.status is not None and status.session_id != self.status.session_id:
            for name in STATUS_CHECKS:
                self.samples.pop(name, None)
        self.status = status
        self.record("定位和地图一致", status.pose is not None)
        self.record("双向心跳", status.online)
        self.record("仲裁未使能", status.motion_enabled is False)
        self.record(
            CLEARANCE_CHECK,
            status.pose is not None
            and self.arena.path_clear(status.pose[:2], status.pose[:2], BASE_RADIUS),
        )
        if self.position_limit is not None or self.heading_limit is not None:
            position_error, heading_error = self.start_errors(status.pose)
            if self.position_limit is not None:
                self.record(
                    POSITION_CHECK, position_error is not None and position_error <= self.position_limit
                )
            if self.heading_limit is not None:
                self.record(
                    HEADING_CHECK, heading_error is not None and heading_error <= self.heading_limit
                )

    def on_map(self, message):
        info = message.info
        expected_resolution, size, (x, y, yaw) = self.map_metadata
        point, q = info.origin.position, info.origin.orientation
        self.map_matches = (
            message.header.frame_id == "map"
            and info.resolution == expected_resolution
            and (info.width, info.height) == size
            and len(message.data) == info.width * info.height
            and (point.x, point.y, point.z) == (x, y, 0.0)
            and q.x == q.y == 0.0
            and math.isclose(math.hypot(q.z, q.w), 1.0, abs_tol=1e-9)
            and angle_error(yaw, yaw_from_quaternion(q)) <= 1e-9
        )

    def on_scan(self, message):
        self.record(
            "新鲜雷达",
            self.current_stamp(message.header, "laser_frame", MAX_SCAN_AGE)
            and scan_has_valid_ranges(message),
            max_gap=SCAN_TIMEOUT,
        )

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
            resolved = values.get("resolved_cmd_vel_topic")
            self.resolved_bridge_topic = resolved
            self.record(BRIDGE_TOPIC_CHECK, resolved == "/cmd_vel_safe", max_gap=1.5)
            ok = (
                status.level == DiagnosticStatus.OK
                and values.get("host_robot_model") == "alohamini2edu"
                and values.get("command_enabled") == values.get("stop_pending") == "false"
            )
            self.record("真机桥接健康且未使能", ok, max_gap=1.5)

    def read_services(self):
        parameters = (
            self.bridge_params.call_async(
                GetParameters.Request(names=["base_only", "cmd_vel_topic", "command_timeout_sec"])
            )
            if self.bridge_params is not None and self.bridge_params.service_is_ready()
            else None
        )
        return (
            {
                name: client.call_async(GetState.Request()) if client.service_is_ready() else None
                for name, client in self.lifecycle_clients.items()
            },
            parameters,
        )

    def results(self, lifecycle, parameters):
        checks = {
            name: self.recent(name)
            for name in (
                "定位和地图一致", "双向心跳", "仲裁未使能", "新鲜惯导",
                "底盘反馈静止", "仲裁输出零速", CLEARANCE_CHECK,
            )
        }
        checks["新鲜雷达"] = self.recent("新鲜雷达", max_age=SCAN_TIMEOUT)
        cmd_publishers = [item.node_name for item in self.get_publishers_info_by_topic("/cmd_vel")]
        safe_publishers = [
            item.node_name for item in self.get_publishers_info_by_topic("/cmd_vel_safe")
        ]
        cmd_consumers = [
            item.node_name for item in self.get_subscriptions_info_by_topic("/cmd_vel")
        ]
        safe_consumers = [
            item.node_name for item in self.get_subscriptions_info_by_topic("/cmd_vel_safe")
        ]
        for name, label in LIFECYCLE_NODES.items():
            reply = lifecycle.get(name)
            checks[label] = reply is not None and reply.current_state.id == State.PRIMARY_STATE_ACTIVE
        checks[MAP_CHECK] = self.map_matches and [
            item.node_name for item in self.get_publishers_info_by_topic("/map")
        ] == ["map_server"]
        checks["监测出口未出现非零指令"] = not self.nonzero_safe_velocity
        checks["速度链路只有一个仲裁和一个监测出口"] = (
            cmd_publishers == [ARBITER]
            and safe_publishers == [MONITOR]
            and MONITOR in cmd_consumers
            and BRIDGE not in cmd_consumers
            and SIMULATOR not in cmd_consumers
        )
        if self.bridge_params is not None:
            values = parameters.values if parameters is not None else ()
            checks["真机桥接健康且未使能"] = self.recent(
                "真机桥接健康且未使能", count=2, max_age=1.5
            )
            checks["真机桥接使用底盘模式和监测出口"] = (
                len(values) == 3
                and values[0].type == ParameterType.PARAMETER_BOOL
                and values[0].bool_value
                and values[1].type == ParameterType.PARAMETER_STRING
                and values[1].string_value == "/cmd_vel_safe"
            )
            checks[BRIDGE_TIMEOUT_CHECK] = (
                len(values) == 3
                and values[2].type == ParameterType.PARAMETER_DOUBLE
                and 0 < values[2].double_value <= COMMAND_TIMEOUT
            )
            checks[BRIDGE_TOPIC_CHECK] = self.recent(BRIDGE_TOPIC_CHECK, count=2, max_age=1.5)
        if self.bridge_params is None:
            checks["仿真底盘订阅监测出口"] = SIMULATOR in safe_consumers
        pose = self.status.pose if checks["定位和地图一致"] else None
        position_error, heading_error = self.start_errors(pose)
        if self.position_limit is not None:
            checks[POSITION_CHECK] = self.recent(POSITION_CHECK)
        if self.heading_limit is not None:
            checks[HEADING_CHECK] = self.recent(HEADING_CHECK)
        return {
            "检查": checks,
            "位姿": pose,
            "距标定起点_m": position_error,
            "起点朝向偏差_deg": heading_error,
            "桥接解析后速度入口": self.resolved_bridge_topic,
            "速度源": cmd_publishers,
            "安全速度源": safe_publishers,
            "安全速度采样数": self.safe_velocity_samples,
        }


def main(args=None):
    parser = dual_domain_parser(__doc__, (5, 6))
    parser.add_argument("--duration", type=float, default=5.0, help="采样秒数")
    parser.add_argument("--simulation", action="store_true", help="读取仿真时钟")
    parser.add_argument("--start-position-tolerance-m", type=float, help="起点位置偏差限值（米）")
    parser.add_argument("--start-heading-tolerance-deg", type=float, help="起点朝向偏差限值（度）")
    arguments = parse_dual_domain_args(parser, args)
    if not math.isfinite(arguments.duration) or arguments.duration <= 0:
        parser.error("采样时间必须是有限正数")
    position_limit = arguments.start_position_tolerance_m
    heading_limit = arguments.start_heading_tolerance_deg
    if not arguments.simulation and (position_limit is None or heading_limit is None):
        parser.error("真机预检必须指定 --start-position-tolerance-m 和 --start-heading-tolerance-deg")
    if position_limit is not None and (not math.isfinite(position_limit) or position_limit < 0):
        parser.error("起点位置限值必须是有限的非负数")
    if heading_limit is not None and not 0 <= heading_limit <= 180:
        parser.error("起点朝向限值必须在 0 到 180 度之间")
    arena = Arena.load(arguments.arena, arguments.map)
    map_metadata = load_map_metadata(arguments.map)
    with ExitStack() as resources:
        endpoints = [
            attach_domain(
                resources,
                domain,
                lambda context, robot=robot: VehicleCheck(
                    context, arena, robot, arguments.simulation,
                    map_metadata=map_metadata, position_limit=position_limit, heading_limit=heading_limit,
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
        futures = [future for states, params in pending for future in (*states.values(), params)]
        while time.monotonic() < deadline and any(
            future is not None and not future.done() for future in futures
        ):
            for executor in executors:
                executor.spin_once(timeout_sec=0.02)
        results = {}
        def service_reply(future):
            if future is not None and future.done() and future.exception() is None:
                return future.result()
            return None

        for node, domain, (states, params) in zip(nodes, arguments.domains, pending, strict=True):
            lifecycle = {name: service_reply(future) for name, future in states.items()}
            results[node.robot_id] = node.results(lifecycle, service_reply(params)) | {"ROS域": domain}
    first, second = (item["位姿"] for item in results.values())
    separation = math.dist(first[:2], second[:2]) if first and second else None
    clear = separation is not None and separation > 2 * BASE_RADIUS
    passed = clear and all(all(value["检查"].values()) for value in results.values())
    print(
        json.dumps(
            {
                "通过": passed, "场地指纹": arena.fingerprint, "名义双车净空": clear,
                "中心距_m": separation,
                "起点偏差限值": {"位置_m": position_limit, "朝向_deg": heading_limit},
                "双车": results,
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0 if passed else 1
