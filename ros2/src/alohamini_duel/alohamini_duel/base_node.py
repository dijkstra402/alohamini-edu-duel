"""连接定位、UDP 心跳和底盘速度仲裁。"""

import json
import math
import time

from alohamini.schema import BodyVelocity
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Header, String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener

from .arena import ROBOT_IDS, Arena
from .cmd_vel_arbiter import STOP, VelocityArbiter
from .peer_sync import HEARTBEAT_PERIOD, MAX_PACKET_BYTES, PeerLink, open_peer_socket
from .ros_domains import spin_until_shutdown
from .status import (
    MAX_POSE_AGE,
    MAX_SCAN_AGE,
    SCAN_TIMEOUT,
    fresh_stamp,
    scan_has_valid_ranges,
    yaw_from_quaternion,
)

MAX_DATAGRAMS_PER_TICK = 64
UDP_DRAIN_BUDGET = HEARTBEAT_PERIOD / 10


class DuelBase(Node):
    def __init__(self):
        super().__init__("alohamini_duel_base")
        defaults = {
            "robot_id": "",
            "arena": "",
            "map": "",
            "bind_ip": "0.0.0.0",
            "bind_port": 5588,
            "peer_ip": "",
            "peer_port": 5588,
            "simulation_controls": False,
            "referee_status": False,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        settings = {name: self.get_parameter(name).value for name in defaults}
        if settings["robot_id"] not in ROBOT_IDS or not settings["peer_ip"]:
            raise ValueError("必须指定本机编号和对端地址")
        self.arena = Arena.load(settings["arena"], settings["map"])
        self.arbiter = VelocityArbiter(self.arena)
        self.link = PeerLink(settings["robot_id"], self.arena)
        self.tf = Buffer(node=self)
        self.listener = TransformListener(self.tf, self)
        self.odometry: tuple[Odometry, float] | None = None
        self.scan: tuple[Header, float] | None = None
        self.online = False
        self.velocity_pub = self.create_publisher(Twist, "/cmd_vel", 1)
        self.online_pub = self.create_publisher(Bool, "/duel/peer_online", 1)
        self.pose_pub = self.create_publisher(PoseStamped, "/opponent/pose", 1)
        self.create_subscription(Odometry, "/odom", self._odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/scan_filtered", self._scan, qos_profile_sensor_data)
        self.create_subscription(Twist, "/cmd_vel_duel", lambda m: self._command("duel", m), 1)
        self.create_subscription(Twist, "/cmd_vel_nav_out", lambda m: self._command("nav", m), 1)
        self.create_service(SetBool, "~/command_enable", self._enable)
        self.forward_peer_packets = True
        self.status_seq = 0
        self.status_pub = None
        if settings["simulation_controls"]:
            self.create_service(SetBool, "~/peer_link_enable", self._set_sim_link)
        if settings["referee_status"]:
            self.status_pub = self.create_publisher(String, "/duel/local_status", 1)
        # 断联计时独立于 /clock，即使仿真暂停也会清零。
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(HEARTBEAT_PERIOD, self._tick, clock=self.steady_clock)
        self.udp = open_peer_socket(
            settings["bind_ip"], settings["bind_port"], settings["peer_ip"], settings["peer_port"]
        )

    def _odom(self, message):
        self.odometry = message, time.monotonic()

    def _scan(self, message):
        self.scan = (message.header, time.monotonic()) if scan_has_valid_ranges(message) else None

    def _command(self, source, message):
        try:
            velocity = BodyVelocity(message.linear.x, message.linear.y, message.angular.z)
        except ValueError:
            velocity = STOP
        self.arbiter.submit(source, velocity, time.monotonic())

    def _enable(self, request, response):
        if request.data:
            now = time.monotonic()
            now_ns = self.get_clock().now().nanoseconds
            if (
                not self.online or self.link.online(now) is None
                or self._local_state(now_ns, now) is None
            ):
                response.success = False
                response.message = "定位或双向心跳未就绪，恢复后请重新请求使能"
                return response
            if not self._scan_fresh(now_ns, now):
                response.success = False
                response.message = "雷达数据未就绪，恢复后请重新请求使能"
                return response
        self.arbiter.enable(request.data)
        if not request.data:
            self.velocity_pub.publish(Twist())
        response.success = True
        response.message = "已使能，等待新速度指令" if request.data else "已停止并禁用速度输出"
        return response

    def _set_sim_link(self, request, response):
        self.forward_peer_packets = request.data
        response.success = True
        response.message = "仿真心跳链路已接通" if request.data else "仿真心跳链路已切断"
        return response

    def _fresh(self, header, now_ns):
        stamp_ns = header.stamp.sec * 1_000_000_000 + header.stamp.nanosec
        return stamp_ns > 0 and -0.05 <= (now_ns - stamp_ns) / 1e9 <= MAX_POSE_AGE

    def _scan_fresh(self, now_ns, now):
        if self.scan is None:
            return False
        header, received = self.scan
        return (
            now - received <= SCAN_TIMEOUT
            and fresh_stamp(header, "laser_frame", now_ns, MAX_SCAN_AGE) is not None
        )

    def _local_state(self, now_ns, now):
        if self.odometry is None:
            return None
        odometry, received = self.odometry
        if now - received > MAX_POSE_AGE or not self._fresh(odometry.header, now_ns):
            return None
        try:
            transform = self.tf.lookup_transform("map", "base_link", Time())
        except TransformException:
            return None
        if not self._fresh(transform.header, now_ns):
            return None
        point, q = transform.transform.translation, transform.transform.rotation
        yaw = yaw_from_quaternion(q)
        twist = odometry.twist.twist.linear
        if not all(math.isfinite(value) for value in (point.x, point.y, yaw, twist.x, twist.y)):
            return None
        if not self.arena.contains((point.x, point.y)):
            return None
        return (point.x, point.y, yaw), twist.x, twist.y

    def _tick(self):
        previous_remote = self.link.remote
        drain_until = time.monotonic() + UDP_DRAIN_BUDGET
        # 为定位采样和速度仲裁保留执行时间，同时及时确认突发到包。
        for _ in range(MAX_DATAGRAMS_PER_TICK):
            if time.monotonic() >= drain_until:
                break
            try:
                data = self.udp.recv(MAX_PACKET_BYTES + 1)
            except BlockingIOError:
                break
            except OSError:
                self.get_logger().warning("UDP 心跳接收失败", throttle_duration_sec=3.0)
                break
            if self.forward_peer_packets:
                self.link.accept(data, time.monotonic())
        # 先处理本轮确认，再采集位姿。对端用被确认的发送时间估算通信陈旧度。
        now = time.monotonic()
        stamp = self.get_clock().now()
        local = self._local_state(stamp.nanoseconds, now)
        if local is not None and self.forward_peer_packets:
            try:
                self.udp.send(self.link.encode(*local, now=now))
            except OSError:
                self.get_logger().warning("UDP 心跳发送失败", throttle_duration_sec=3.0)
        now = time.monotonic()
        peer = self.link.online(now) if local is not None else None
        online = peer is not None
        if self.arbiter.enabled and (not online or not self._scan_fresh(stamp.nanoseconds, now)):
            self.arbiter.enable(False)
            if online:
                self.get_logger().warning("雷达数据不可用或超时，已禁用速度输出")
        if online != self.online:
            message = "双向心跳已就绪" if online else "本机定位失效，输出零速"
            if not online and local is not None:
                message = (
                    f"双向心跳失效，输出零速。报文年龄 {now - self.link.remote.received_at:.3f} 秒，"
                    f"确认年龄 {now - self.link.acked_sent_at:.3f} 秒，"
                    f"连续收包 {self.link.consecutive}，连续确认 {self.link.ack_consecutive}"
                )
            self.get_logger().info(message)
            self.online = online
        self.online_pub.publish(Bool(data=online))
        if self.status_pub is not None:
            self.status_seq += 1
            status = {
                "robot_id": self.link.robot_id,
                "arena_id": self.arena.fingerprint,
                "session_id": self.link.session_id,
                "seq": self.status_seq,
                "pose": local[0] if local is not None else None,
                "online": online,
                "motion_enabled": self.arbiter.enabled,
            }
            self.status_pub.publish(String(data=json.dumps(status)))
        if peer is not None and peer is not previous_remote:
            opponent = PoseStamped()
            opponent.header.frame_id = "map"
            opponent.header.stamp = stamp.to_msg()
            opponent.pose.position.x, opponent.pose.position.y = peer.pose[:2]
            opponent.pose.orientation.z = math.sin(peer.pose[2] / 2)
            opponent.pose.orientation.w = math.cos(peer.pose[2] / 2)
            self.pose_pub.publish(opponent)
        velocity = self.arbiter.select(local[0] if local is not None else None, peer, now)
        message = Twist()
        message.linear.x, message.linear.y = velocity.x_m_s, velocity.y_m_s
        message.angular.z = velocity.yaw_rad_s
        self.velocity_pub.publish(message)

    def destroy_node(self):
        self.udp.close()
        return super().destroy_node()


def main(args=None):
    spin_until_shutdown(DuelBase, args)
