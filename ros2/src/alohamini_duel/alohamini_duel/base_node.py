"""连接定位、UDP 心跳和底盘速度仲裁。"""

import json
import math
import time

import rclpy
from alohamini.schema import BodyVelocity
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener

from .arena import ROBOT_IDS, Arena
from .cmd_vel_arbiter import STOP, VelocityArbiter
from .peer_sync import HEARTBEAT_PERIOD, MAX_PACKET_BYTES, PeerLink, open_peer_socket
from .status import yaw_from_quaternion

MAX_POSE_AGE = 0.35


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
        self.online = False
        self.velocity_pub = self.create_publisher(Twist, "/cmd_vel", 1)
        self.online_pub = self.create_publisher(Bool, "/duel/peer_online", 1)
        self.pose_pub = self.create_publisher(PoseStamped, "/opponent/pose", 1)
        self.create_subscription(Odometry, "/odom", self._odom, qos_profile_sensor_data)
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

    def _command(self, source, message):
        try:
            velocity = BodyVelocity(message.linear.x, message.linear.y, message.angular.z)
        except ValueError:
            velocity = STOP
        self.arbiter.submit(source, velocity, time.monotonic())

    def _enable(self, request, response):
        self.arbiter.enable(request.data)
        if not request.data:
            self.velocity_pub.publish(Twist())
        response.success = True
        response.message = "已等待双向心跳和速度指令" if request.data else "已停止并禁用速度输出"
        return response

    def _set_sim_link(self, request, response):
        self.forward_peer_packets = request.data
        response.success = True
        response.message = "仿真心跳链路已接通" if request.data else "仿真心跳链路已切断"
        return response

    def _fresh(self, header, now_ns):
        stamp_ns = header.stamp.sec * 1_000_000_000 + header.stamp.nanosec
        return stamp_ns > 0 and -0.05 <= (now_ns - stamp_ns) / 1e9 <= MAX_POSE_AGE

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
        now = time.monotonic()
        stamp = self.get_clock().now()
        local = self._local_state(stamp.nanoseconds, now)
        if local is not None and self.forward_peer_packets:
            try:
                self.udp.send(self.link.encode(*local, now=now))
            except OSError:
                self.get_logger().warning("UDP 心跳发送失败", throttle_duration_sec=3.0)
        for _ in range(16):
            try:
                data = self.udp.recv(MAX_PACKET_BYTES + 1)
            except BlockingIOError:
                break
            except OSError:
                self.get_logger().warning("UDP 心跳接收失败", throttle_duration_sec=3.0)
                break
            if self.forward_peer_packets:
                self.link.accept(data, now)
        peer = self.link.online(now) if local is not None else None
        online = peer is not None
        if online != self.online:
            self.get_logger().info("双向心跳已就绪" if online else "定位或心跳中断，输出零速")
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
        if peer is not None and peer.received_at == now:
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
    rclpy.init(args=args)
    node = None
    try:
        node = DuelBase()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
