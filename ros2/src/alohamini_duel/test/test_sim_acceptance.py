"""仿真验收的时序记录、配对净空和回环报文。"""

import socket
import time
from contextlib import ExitStack
from pathlib import Path

import pytest

from alohamini_duel.arena import ROBOT_IDS
from alohamini_duel.sim_evidence import GroundStop, RoundTrace
from alohamini_duel.sim_link import DELAY_S, SimulationLink
from alohamini_duel.sim_round import prepare_round

CONFIG = Path(__file__).parents[1] / "config"


def test_ground_stop_rejects_duplicate_frames_and_resets_when_motion_returns():
    evidence = GroundStop(1_000_000_000)
    for tick in (1, 2, 2, 3, 4):
        evidence.observe(tick, 1_000_000_000 + tick * 20_000_000, True)
    assert evidence.stopped_s is None
    evidence.observe(5, 1_100_000_000, True)
    assert evidence.stopped_s == pytest.approx(0.02)
    assert evidence.confirmed_s == pytest.approx(0.10)
    evidence.observe(6, 1_120_000_000, False)
    assert evidence.stopped_s is None
    for tick in range(7, 12):
        evidence.observe(tick, 1_000_000_000 + tick * 20_000_000, True)
    assert evidence.stopped_s == pytest.approx(0.14)
    assert evidence.confirmed_s == pytest.approx(0.22)


def test_round_trace_pairs_domains_and_detects_crossing_between_samples(tmp_path):
    _, _, arena, routes = prepare_round(CONFIG / "round_example.yaml", tmp_path)
    assert set(routes) == set(ROBOT_IDS)
    trace = RoundTrace(arena)
    for robot, position in zip(reversed(ROBOT_IDS), ((4, 2.5), (1, 2.5)), strict=True):
        trace.observe(robot, 1_000_000_000, position)
    assert trace.count == 1
    assert trace.min_distance == pytest.approx(3)
    trace.observe(ROBOT_IDS[0], 1_020_000_000, (4, 2.5))
    with pytest.raises(RuntimeError, match="中心距"):
        trace.observe(ROBOT_IDS[1], 1_020_000_000, (1, 2.5))


def test_loopback_relay_applies_loss_delay_and_restores_delivery():
    with ExitStack() as resources:
        link = SimulationLink(resources, (91, 92))
        assert tuple(address[1] for address in link.vehicles) == (5691, 5692)
        peers = []
        for address, port in zip(link.vehicles, link.ports, strict=True):
            endpoint = resources.enter_context(socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
            endpoint.bind(address)
            endpoint.connect(("127.0.0.1", port))
            endpoint.settimeout(2)
            peers.append(endpoint)
        first, second = peers
        link.set_mode("loss")
        for packet in (b"1", b"2", b"3"):
            first.send(packet)
        assert [second.recv(8), second.recv(8)] == [b"1", b"2"]
        second.settimeout(0.05)
        with pytest.raises(TimeoutError):
            second.recv(8)
        assert link.snapshot()["丢弃"] == 1

        injected = link.set_mode("delay")
        first.send(b"4")
        with pytest.raises(TimeoutError):
            second.recv(8)
        second.settimeout(2)
        assert second.recv(8) == b"4"
        assert (time.monotonic_ns() - injected) / 1e9 >= DELAY_S
        assert link.snapshot()["延迟转发"] == 1
        link.set_mode("normal")
        first.send(b"5")
        assert second.recv(8) == b"5"
