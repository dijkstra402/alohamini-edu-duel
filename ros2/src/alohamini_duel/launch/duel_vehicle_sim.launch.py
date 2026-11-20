"""在当前 ROS 域启动一辆仿真车的定位与控制。"""

from alohamini_duel.launching import vehicle_launch


def generate_launch_description():
    return vehicle_launch(simulation=True)
