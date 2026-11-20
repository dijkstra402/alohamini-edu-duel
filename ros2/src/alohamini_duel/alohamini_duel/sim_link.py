"""在本机仿真域之间转发 UDP，并按验收场景注入网络故障。"""

import select
import socket
import time
from collections import deque
from contextlib import ExitStack
from threading import Event, Lock, Thread

from .peer_sync import MAX_PACKET_BYTES

DELAY_S = 0.75


def loopback_socket(resources):
    endpoint = resources.enter_context(socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
    endpoint.bind(("127.0.0.1", 0))
    return endpoint


class SimulationLink:
    def __init__(self, resources):
        self.endpoints = tuple(loopback_socket(resources) for _ in range(2))
        with ExitStack() as reservation:
            self.vehicles = tuple(loopback_socket(reservation).getsockname() for _ in range(2))
        self.lock = Lock()
        self.pending = deque()
        self.mode = "normal"
        self.counts = self.empty_counts()
        self.stop = Event()
        self.worker = Thread(target=self.forward, name="仿真心跳转发", daemon=True)
        self.worker.start()
        resources.callback(self.close)

    @staticmethod
    def empty_counts():
        return dict.fromkeys(("收到", "丢弃", "转发", "延迟转发"), 0)

    @property
    def ports(self):
        return tuple(endpoint.getsockname()[1] for endpoint in self.endpoints)

    def set_mode(self, mode):
        with self.lock:
            self.mode = mode
            self.pending.clear()
            self.counts = self.empty_counts()
            return time.monotonic_ns()

    def snapshot(self):
        with self.lock:
            return self.counts.copy()

    def forward(self):
        while not self.stop.is_set():
            readable, _, _ = select.select(self.endpoints, [], [], 0.01)
            with self.lock:
                for endpoint in readable:
                    source = self.endpoints.index(endpoint)
                    destination = 1 - source
                    packet, sender = endpoint.recvfrom(MAX_PACKET_BYTES + 1)
                    if sender != self.vehicles[source]:
                        continue
                    self.counts["收到"] += 1
                    if self.mode == "cut" or (
                        self.mode == "loss" and self.counts["收到"] % 3 == 0
                    ):
                        self.counts["丢弃"] += 1
                        continue
                    if self.mode == "delay" and source == 0:
                        self.pending.append((time.monotonic() + DELAY_S, destination, packet))
                        continue
                    self.endpoints[destination].sendto(packet, self.vehicles[destination])
                    self.counts["转发"] += 1
                now = time.monotonic()
                while self.pending and self.pending[0][0] <= now:
                    _, destination, packet = self.pending.popleft()
                    self.endpoints[destination].sendto(packet, self.vehicles[destination])
                    self.counts["转发"] += 1
                    self.counts["延迟转发"] += 1

    def close(self):
        self.stop.set()
        self.worker.join()
