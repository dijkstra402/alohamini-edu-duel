"""记录共享时钟的双车真值和从故障注入开始的停稳时间。"""

from dataclasses import dataclass

from .arena import BASE_RADIUS, ROBOT_IDS, segment_distance

STOP_FRAMES = 5


@dataclass
class GroundStop:
    injected_ns: int
    last_stamp: int = 0
    count: int = 0
    first_ns: int = 0
    stopped_s: float | None = None
    confirmed_s: float | None = None

    def observe(self, stamp, received_ns, stationary):
        if stamp <= self.last_stamp:
            return
        self.last_stamp = stamp
        if not stationary:
            self.count = 0
            self.stopped_s = None
            self.confirmed_s = None
            return
        self.count += 1
        if self.count == 1:
            self.first_ns = received_ns
        if self.count == STOP_FRAMES:
            self.stopped_s = (self.first_ns - self.injected_ns) / 1e9
            self.confirmed_s = (received_ns - self.injected_ns) / 1e9


class RoundTrace:
    def __init__(self, arena):
        self.arena = arena
        self.pending = {}
        self.previous = None
        self.last_stamp = 0
        self.count = 0
        self.min_distance = float("inf")
        self.max_gap = 0.0

    def observe(self, robot, stamp, position):
        if stamp <= self.last_stamp:
            return
        frame = self.pending.setdefault(stamp, {})
        frame[robot] = position
        if len(frame) < 2:
            # 同时保留两域可能错开的近期帧，避免长时间掉线累积数据。
            self.pending = {
                key: data for key, data in self.pending.items() if key > stamp - 250_000_000
            }
            return
        current = tuple(frame[key] for key in ROBOT_IDS)
        previous = self.previous or current
        if not all(
            self.arena.path_clear(a, b, BASE_RADIUS)
            for a, b in zip(previous, current, strict=True)
        ):
            raise RuntimeError("仿真真值越过墙或箱体的净空边界")
        delta_before = (previous[0][0] - previous[1][0], previous[0][1] - previous[1][1])
        delta_after = (current[0][0] - current[1][0], current[0][1] - current[1][1])
        distance = segment_distance(delta_before, delta_after, (0.0, 0.0))
        if distance <= 2 * BASE_RADIUS:
            raise RuntimeError("仿真真值低于双车中心距净空")
        self.min_distance = min(self.min_distance, distance)
        if self.last_stamp:
            self.max_gap = max(self.max_gap, (stamp - self.last_stamp) / 1e9)
        self.previous, self.last_stamp = current, stamp
        self.count += 1
        self.pending = {key: data for key, data in self.pending.items() if key > stamp}
