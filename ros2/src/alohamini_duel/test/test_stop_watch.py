import pytest
from alohamini_duel.status import VehicleStatus
from alohamini_duel.stop_watch import StopObservation

POSE = (1.5, 1.5, 0.0)


def online_before_cut():
    observation = StopObservation()
    observation.observe_status(VehicleStatus("现场会话", 1, POSE, True, True), 1.0)
    observation.observe_command(False, 1.04)
    observation.observe_wheel(1, False, 1.05)
    return observation


def test_records_zero_and_stop_from_distinct_feedback():
    observation = online_before_cut()
    observation.observe_status(VehicleStatus("现场会话", 2, POSE, False, True), 1.1)
    observation.observe_command(True, 1.11)
    for index in range(2, 6):
        observation.observe_wheel(index, True, 1.12 + (index - 2) * 0.01)
    assert not observation.completed(1.15)
    observation.observe_wheel(6, True, 1.16)
    result = observation.results(1.0, 1.16)
    assert result["记录完整"] and result["断联前轮速显示运动"]
    assert result["检测到零速_s"] == pytest.approx(0.01)
    assert result["检测到停稳_s"] == pytest.approx(0.06)


def test_ignores_loc_loss_and_rejects_recovery_before_stop():
    observation = online_before_cut()
    observation.observe_status(VehicleStatus("现场会话", 2, None, False, True), 1.1)
    assert observation.edge is None
    observation.observe_status(VehicleStatus("现场会话", 3, POSE, True, True), 1.11)
    observation.observe_status(VehicleStatus("现场会话", 4, POSE, False, True), 1.12)
    observation.observe_status(VehicleStatus("现场会话", 5, POSE, True, True), 1.15)
    observation.observe_command(True, 1.15)
    for index in range(2, 7):
        observation.observe_wheel(index, True, 1.16 + (index - 2) * 0.01)
    assert observation.interrupted and not observation.completed(1.2)


def test_times_out_without_fresh_increasing_wheel_stamps():
    observation = online_before_cut()
    observation.observe_command(True, 1.09)
    observation.observe_status(VehicleStatus("现场会话", 2, POSE, False, True), 1.1)
    assert observation.zero_at == 1.1
    for index in range(5):
        observation.observe_wheel(2, True, 1.11 + 0.01 * index)
    assert observation.stopped_count == 1
    assert not observation.completed(1.15)
    observation.observe_wheel(None, False, 1.16)
    assert observation.stopped_count == 0
