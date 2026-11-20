import json

import pytest
from alohamini_duel.fault_event import FaultEvent
from alohamini_duel.status import VehicleStatus
from alohamini_duel.stop_watch import StopObservation, mark_report

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


def test_fault_event_cannot_be_overwritten_or_reused_across_boots(tmp_path):
    path = tmp_path / "故障.json"
    event = FaultEvent(1_060_000_000, "现场触发器")
    event.write(path, "本次启动")
    assert FaultEvent.read(path, "本次启动") == event
    with pytest.raises(FileExistsError):
        FaultEvent(2_000_000_000, "后续触发").write(path, "本次启动")
    assert FaultEvent.read(path, "本次启动") == event
    with pytest.raises(ValueError, match="启动不一致"):
        FaultEvent.read(path, "另一次启动")
    raw = json.loads(path.read_text(encoding="utf-8"))
    path.write_text(json.dumps(raw | {"triggered_ns": True}), encoding="utf-8")
    with pytest.raises(ValueError, match="单调时间"):
        FaultEvent.read(path, "本次启动")


def timed_observation(first_stationary):
    observation = online_before_cut()
    observation.observe_status(VehicleStatus("现场会话", 2, POSE, False, True), 1.1)
    observation.observe_status(
        VehicleStatus("现场会话", 3, POSE, False, True), first_stationary
    )
    observation.observe_wheel(2, False, first_stationary - 0.01)
    observation.observe_command(True, first_stationary)
    for index in range(5):
        observation.observe_wheel(3 + index, True, first_stationary + index * 0.01)
    return observation


@pytest.mark.parametrize(("first_stationary", "passed"), [(1.45, True), (1.61, False)])
def test_marked_report_judges_half_second_from_fault_not_link_loss(first_stationary, passed):
    observation = timed_observation(first_stationary)
    event = FaultEvent(1_060_000_000, "现场触发器")
    now = first_stationary + 0.04
    report = {
        "记录完整": True, "断联前观察到运动": True,
        "双车": {"现场车": observation.results(1.0, now)},
    }
    assert mark_report(report, {"现场车": observation}, event, None, now) is passed
    assert report["0.5秒内停稳"] is passed
    result = report["双车"]["现场车"]
    assert result["故障到检测断联_s"] == pytest.approx(0.04)
    assert result["故障到轮速停稳_s"] == pytest.approx(first_stationary - 1.06)
    assert result["故障到停稳确认_s"] == pytest.approx(first_stationary - 1.02)


def test_fault_timing_keeps_wheel_stop_observed_before_link_edge():
    observation = online_before_cut()
    observation.observe_command(True, 1.07)
    observation.observe_wheel(2, True, 1.08)
    observation.observe_status(VehicleStatus("现场会话", 2, POSE, False, True), 1.1)
    for index in range(5):
        observation.observe_wheel(3 + index, True, 1.11 + index * 0.01)
    earlier = FaultEvent(1_060_000_000, "现场触发器")
    evidence = observation.fault_timing(earlier, 1.15)
    assert evidence["故障到轮速停稳_s"] == pytest.approx(0.02)
    assert evidence["故障到观测零速_s"] == pytest.approx(0.01)
    late = FaultEvent(1_110_000_000, "晚标记")
    assert observation.fault_timing(late, 1.15) is None


def test_missing_fault_marker_is_not_treated_as_pass():
    report = {"记录完整": True, "断联前观察到运动": True, "双车": {}}
    assert not mark_report(report, {}, None, None, 2.0)
    assert report["0.5秒内停稳"] is None
    assert report["失败原因"] == "未获得故障触发标记"
