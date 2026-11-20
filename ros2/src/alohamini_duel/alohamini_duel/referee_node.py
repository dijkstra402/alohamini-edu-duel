"""为无接触电子博弈运行独立的软件裁判。"""

import json
import math
import time
from contextlib import ExitStack
from dataclasses import dataclass

from .arena import ROBOT_IDS, Arena, Pose, coordinates
from .peer_sync import PEER_TIMEOUT

FIRST, SECOND = ROBOT_IDS
MATCHING_CHOICE = {"left": "right", "center": "center", "right": "left"}


@dataclass(frozen=True)
class PlayerReport:
    session_id: str
    seq: int
    pose: Pose | None
    received_at: float
    consecutive: int


def angle_error(angle, yaw):
    difference = angle - yaw
    return abs(math.atan2(math.sin(difference), math.cos(difference)))


class Referee:
    def __init__(self, arena: Arena):
        self.arena = arena
        self.players: dict[str, PlayerReport] = {}
        self.old_sessions: set[tuple[str, str]] = set()
        self.choices: dict[str, str] = {}
        self.round = 1
        self.ready = False
        self.scores = dict.fromkeys(ROBOT_IDS, 0)
        self.result = "等待双方联机"

    def observe(self, robot_id, status, now):
        try:
            data = json.loads(status)
            if data["robot_id"] != robot_id or data["arena_id"] != self.arena.fingerprint:
                return
            session, seq, online = data["session_id"], data["seq"], data["online"]
            if (
                not isinstance(session, str)
                or not 1 <= len(session) <= 64
                or type(seq) is not int
                or seq < 1
                or type(online) is not bool
            ):
                return
            pose = coordinates(data["pose"], 3, "裁判位姿") if online else None
        except (KeyError, TypeError, ValueError, RecursionError):
            return
        if pose is not None and (not self.arena.contains(pose[:2]) or abs(pose[2]) > math.pi):
            return
        if (robot_id, session) in self.old_sessions:
            return
        self.refresh(now)
        previous = self.players.get(robot_id)
        if previous is not None and session != previous.session_id:
            self.old_sessions.add((robot_id, previous.session_id))
            self.players.pop(robot_id)
            self.refresh(now)
            previous = None
        if previous is not None and seq <= previous.seq:
            return
        streak = (
            previous.consecutive if previous and now - previous.received_at < PEER_TIMEOUT else 0
        )
        self.players[robot_id] = PlayerReport(
            session, seq, pose, now, min(streak + 1, 3) if pose is not None else 0
        )
        self.refresh(now)

    def refresh(self, now):
        active = len(self.players) == len(ROBOT_IDS) and all(
            report.consecutive == 3 and now - report.received_at < PEER_TIMEOUT
            for report in self.players.values()
        )
        if self.ready and not active:
            self.choices.clear()
            self.round += 1
            self.result = "联机中断，回合已暂停"
        if active and not self.ready:
            self.result = "双方在线，等待选择"
        self.ready = active

    def choose(self, robot_id, message, now):
        self.refresh(now)
        round_number, _, choice = message.strip().partition(":")
        if (
            not self.ready
            or round_number != str(self.round)
            or choice not in MATCHING_CHOICE
            or robot_id in self.choices
        ):
            return
        self.choices[robot_id] = choice
        if len(self.choices) < len(ROBOT_IDS):
            return
        self.result = self.score_round()
        self.choices.clear()
        self.round += 1

    def score_round(self):
        first, second = (self.players[robot].pose for robot in ROBOT_IDS)
        dx, dy = second[0] - first[0], second[1] - first[1]
        heading = math.atan2(dy, dx)
        if not 0.85 <= math.hypot(dx, dy) <= 1.15 or angle_error(heading, first[2]) >= math.pi / 9:
            return "双方未进入判分条件"
        if angle_error(heading + math.pi, second[2]) > math.pi / 6:
            self.scores[FIRST] += 2
            return "A 车获得侧向得分"
        if MATCHING_CHOICE[self.choices[FIRST]] == self.choices[SECOND]:
            self.scores[SECOND] += 1
            return "B 车获得匹配得分"
        self.scores[FIRST] += 1
        return "A 车获得正向得分"

    def scoreboard(self):
        return json.dumps(
            {
                "round": self.round,
                "ready": self.ready,
                "scores": self.scores,
                "result": self.result,
            },
            ensure_ascii=False,
        )


def main(args=None):
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from std_msgs.msg import String

    from .ros_domains import attach_domain, parse_dual_domain_args

    arguments = parse_dual_domain_args(__doc__, args, (5, 6))
    referee = Referee(Arena.load(arguments.arena, arguments.map))
    with ExitStack() as resources:
        nodes, executors, publishers = [], [], []
        for robot_id, domain in zip(ROBOT_IDS, arguments.domains, strict=True):
            node, executor = attach_domain(
                resources,
                domain,
                lambda context, robot=robot_id: Node(f"duel_referee_{robot}", context=context),
            )
            node.create_subscription(
                String,
                "/duel/local_status",
                lambda msg, robot=robot_id: referee.observe(robot, msg.data, time.monotonic()),
                1,
            )
            node.create_subscription(
                String,
                "/duel/choice",
                lambda msg, robot=robot_id: referee.choose(robot, msg.data, time.monotonic()),
                10,
            )
            nodes.append(node)
            executors.append(executor)
            publishers.append(node.create_publisher(String, "/duel/scoreboard", 1))

        last_scoreboard = ""
        next_publish = 0.0
        try:
            while all(node.context.ok() for node in nodes):
                for executor in executors:
                    executor.spin_once(timeout_sec=0.01)
                now = time.monotonic()
                referee.refresh(now)
                scoreboard = referee.scoreboard()
                if scoreboard != last_scoreboard:
                    nodes[0].get_logger().info(scoreboard)
                    last_scoreboard = scoreboard
                    next_publish = now
                if now >= next_publish:
                    for publisher in publishers:
                        publisher.publish(String(data=scoreboard))
                    next_publish = now + 0.2
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
