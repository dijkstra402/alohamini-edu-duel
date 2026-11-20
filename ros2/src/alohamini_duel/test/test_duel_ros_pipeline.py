"""验证安装后的一键验收入口与落盘报告。"""

import json
import shutil
import subprocess

import pytest


@pytest.mark.parametrize("use_nav2", [False, True])
def test_full_simulation_fault_and_recovery(tmp_path, use_nav2):
    pytest.importorskip("rclpy", reason="需要项目的 ROS 2 运行环境")
    from ament_index_python.packages import PackageNotFoundError, get_package_prefix

    try:
        get_package_prefix("alohamini_duel")
        get_package_prefix("rmw_cyclonedds_cpp")
        if use_nav2:
            get_package_prefix("nav2_bt_navigator")
    except PackageNotFoundError:
        pytest.skip("需要已构建并加载的双车仿真依赖")
    if not shutil.which("ros2"):
        pytest.skip("需要 ROS 2 命令行入口")

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
    assert (output / "stop_watch.log").is_file()
