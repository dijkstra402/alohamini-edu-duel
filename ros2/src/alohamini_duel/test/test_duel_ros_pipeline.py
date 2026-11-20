"""验证安装后的一键验收入口与落盘报告。"""

import fcntl
import json
import shutil
import subprocess
import tempfile
from pathlib import Path

import pytest


@pytest.fixture
def duel_package_lookup():
    pytest.importorskip("rclpy", reason="需要项目的 ROS 2 运行环境")
    from ament_index_python.packages import PackageNotFoundError, get_package_prefix

    try:
        get_package_prefix("alohamini_duel")
        get_package_prefix("rmw_cyclonedds_cpp")
    except PackageNotFoundError:
        pytest.skip("需要已构建并加载的双车仿真依赖")
    if not shutil.which("ros2"):
        pytest.skip("需要 ROS 2 命令行入口")
    return get_package_prefix


@pytest.mark.parametrize("use_nav2", [False, True])
def test_full_simulation_fault_and_recovery(tmp_path, use_nav2, duel_package_lookup):
    if use_nav2:
        from ament_index_python.packages import PackageNotFoundError

        try:
            duel_package_lookup("nav2_bt_navigator")
        except PackageNotFoundError:
            pytest.skip("需要已加载的 Nav2 导航包")

    output = tmp_path / "report"
    command = ["ros2", "run", "alohamini_duel", "duel_accept_sim", "--output", str(output)]
    if not use_nav2:
        command.append("--no-nav2")
    process = subprocess.run(command, capture_output=True, text=True, timeout=160, check=False)
    assert (output / "report.json").is_file(), process.stdout + process.stderr
    report = json.loads((output / "report.json").read_text(encoding="utf-8"))
    assert process.returncode == 0, (report, process.stderr)
    assert report["通过"]
    assert report["启用Nav2"] == use_nav2
    assert report["ROS域"] == [73, 74]
    checks = report["检查"]
    assert checks["只读预检与速度冲突检测"]
    assert checks["断联停稳"]["记录完整"]
    assert checks["断联时雷达仍更新"]
    assert checks["重连"]
    assert checks["有序停机"]
    assert all(scan["unexpected"] > 0 for scan in checks["雷达差分"].values())
    assert checks["电子判分"]["scores"] == {"sword_a": 0, "shield_b": int(use_nav2)}
    assert ("断联取消航点" in checks) == use_nav2
    assert ("固定航点终点" in checks) == use_nav2
    if use_nav2:
        assert 0 <= checks["固定航点平面误差_m"] <= 0.15
    assert (output / "stop_watch.log").is_file()


def test_acceptance_rejects_simultaneous_runs_with_one_shared_domain(
    tmp_path, duel_package_lookup,
):
    domain_lock = Path(tempfile.gettempdir()) / "alohamini-duel-acceptance-74.lock"
    output = tmp_path / "report"
    command = [
        "ros2", "run", "alohamini_duel", "duel_accept_sim", "--no-nav2",
        "--output", str(output), "--domains", "75", "74",
    ]
    with domain_lock.open("a") as held:
        fcntl.flock(held, fcntl.LOCK_EX | fcntl.LOCK_NB)
        process = subprocess.run(command, capture_output=True, text=True, timeout=10, check=False)
    assert (output / "report.json").is_file(), process.stdout + process.stderr
    report = json.loads((output / "report.json").read_text(encoding="utf-8"))
    assert process.returncode == 1
    assert not report["通过"]
    assert report["失败原因"] == "ROS 域 74 已有双车验收在运行"
    assert not (output / "duel_sim.log").exists()
