"""用带会话与双向确认的 UDP 心跳跟踪对端定位。"""

import json
import math
import socket
import uuid
from dataclasses import dataclass

from .arena import ROBOT_IDS, Arena, Pose, coordinates

PEER_TIMEOUT = 0.5
HEARTBEAT_PERIOD = 0.05
MAX_PACKET_BYTES = 1024


@dataclass(frozen=True)
class PeerState:
    session_id: str
    seq: int
    pose: Pose
    received_at: float


class PeerLink:
    def __init__(self, robot_id, arena: Arena):
        self.robot_id = robot_id
        self.peer_id = next(other for other in ROBOT_IDS if other != robot_id)
        self.arena = arena
        self.session_id = uuid.uuid4().hex
        self.seq = 0
        self.remote: PeerState | None = None
        self.retired_sessions: set[str] = set()
        self.consecutive = 0
        self.ack_seq = 0
        self.ack_at = -math.inf
        self.ack_consecutive = 0

    def encode(self, pose: Pose, vx: float, vy: float, now: float) -> bytes:
        self.seq += 1
        return json.dumps(
            {
                "version": 1,
                "robot_id": self.robot_id,
                "arena_id": self.arena.fingerprint,
                "session_id": self.session_id,
                "seq": self.seq,
                "tx_mono_s": now,
                "x": pose[0],
                "y": pose[1],
                "yaw": pose[2],
                "vx": vx,
                "vy": vy,
                "ack_session_id": self.remote.session_id if self.remote else "",
                "ack_seq": self.remote.seq if self.remote else 0,
            },
            separators=(",", ":"),
        ).encode()

    def accept(self, raw: bytes, now: float):
        if len(raw) > MAX_PACKET_BYTES:
            return
        try:
            packet = json.loads(raw)
            if (
                packet["version"] != 1
                or packet["robot_id"] != self.peer_id
                or packet["arena_id"] != self.arena.fingerprint
            ):
                return
            session = packet["session_id"]
            seq = packet["seq"]
            ack_seq = packet["ack_seq"]
            confirmed_session = packet["ack_session_id"]
            if (
                not isinstance(session, str)
                or len(session) != 32
                or uuid.UUID(hex=session).hex != session
                or type(seq) is not int
                or seq < 1
                or type(ack_seq) is not int
                or ack_seq < 0
            ):
                return
            pose = coordinates((packet["x"], packet["y"], packet["yaw"]), 3, "对端位姿")
            coordinates((packet["vx"], packet["vy"], packet["tx_mono_s"]), 3, "对端遥测")
        except (KeyError, TypeError, ValueError, RecursionError):
            return
        if not self.arena.contains(pose[:2]) or abs(pose[2]) > math.pi:
            return
        if session in self.retired_sessions:
            return
        previous = self.remote
        fresh = previous is not None and now - previous.received_at < PEER_TIMEOUT
        if previous and session != previous.session_id:
            if fresh:
                return
            self.retired_sessions.add(previous.session_id)
            self.ack_seq = 0
            self.ack_at = -math.inf
            self.ack_consecutive = 0
        if previous and session == previous.session_id and seq <= previous.seq:
            return
        self.remote = PeerState(session, seq, pose, now)
        self.consecutive = min(self.consecutive + 1, 3) if fresh else 1
        if (
            confirmed_session == self.session_id
            and self.ack_seq < ack_seq <= self.seq
            and (self.seq - ack_seq) * HEARTBEAT_PERIOD < PEER_TIMEOUT
        ):
            self.ack_consecutive = (
                min(self.ack_consecutive + 1, 3) if now - self.ack_at < PEER_TIMEOUT else 1
            )
            self.ack_seq, self.ack_at = ack_seq, now

    def online(self, now: float) -> PeerState | None:
        if (
            self.remote is not None
            and self.consecutive >= 3
            and self.ack_consecutive >= 3
            and now - self.remote.received_at < PEER_TIMEOUT
            and now - self.ack_at < PEER_TIMEOUT
        ):
            return self.remote
        return None


def open_peer_socket(bind_ip, bind_port, peer_ip, peer_port):
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        udp.bind((bind_ip, bind_port))
        # 由内核限制接收端点，避免其他场地的广播或本机回环进入会话。
        udp.connect((peer_ip, peer_port))
        udp.setblocking(False)
    except OSError:
        udp.close()
        raise
    return udp
