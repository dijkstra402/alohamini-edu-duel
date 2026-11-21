"""单路速度输出与无接触底盘净空预检。"""

import math

from alohamini.schema import BodyVelocity

from .arena import BASE_RADIUS, Arena, Pose, segment_distance
from .peer_sync import PeerState
from .status import MAX_POSE_AGE

MAX_SPEED = 0.12
MAX_YAW_SPEED = 0.35
COMMAND_TIMEOUT = 0.15
LOOKAHEAD = 0.3
STOP = BodyVelocity()


class VelocityArbiter:
    def __init__(self, arena: Arena):
        self.arena = arena
        self.enabled = False
        self.commands: dict[str, tuple[BodyVelocity, float]] = {}

    def enable(self, enabled: bool):
        self.commands.clear()
        self.enabled = enabled

    def submit(self, source: str, velocity: BodyVelocity, now: float):
        self.commands[source] = (velocity, now)

    def select(self, pose: Pose | None, peer: PeerState | None, now: float) -> BodyVelocity:
        if not self.enabled or pose is None or peer is None:
            self.commands.clear()
            return STOP
        for source in ("duel", "nav"):
            velocity, received = self.commands.get(source, (STOP, -math.inf))
            if now - received < COMMAND_TIMEOUT:
                return self.limit(velocity, pose, peer, now)
        return STOP

    def limit(
        self, velocity: BodyVelocity, pose: Pose, peer: PeerState, now: float
    ) -> BodyVelocity:
        vx, vy, wz = velocity.x_m_s, velocity.y_m_s, velocity.yaw_rad_s
        speed = math.hypot(vx, vy)
        if speed > MAX_SPEED:
            vx, vy = vx * (MAX_SPEED / speed), vy * (MAX_SPEED / speed)
        wz = min(MAX_YAW_SPEED, max(-MAX_YAW_SPEED, wz))
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        start = (x, y)
        end = (x + (vx * c - vy * s) * LOOKAHEAD, y + (vx * s + vy * c) * LOOKAHEAD)
        # 用转弯偏差上界扩张整条线段，避免只检查预测终点。
        arc_margin = min(speed, MAX_SPEED) * abs(wz) * LOOKAHEAD**2 / 2
        if not self.arena.path_clear(start, end, BASE_RADIUS + arc_margin):
            return STOP
        # 确认对应本机发送时间，覆盖通信和收包排队。定位采样本身还有 TF 陈旧度。
        peer_travel = MAX_SPEED * (now - peer.confirmed_send_at + MAX_POSE_AGE + LOOKAHEAD)
        separation = 2 * BASE_RADIUS + peer_travel + arc_margin
        if segment_distance(start, end, peer.pose[:2]) <= separation:
            return STOP
        return BodyVelocity(vx, vy, wz)
