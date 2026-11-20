"""启动单车实机定位与双车底盘联调。"""

from alohamini_duel.launching import vehicle_launch


def generate_launch_description():
    return vehicle_launch(simulation=False)
