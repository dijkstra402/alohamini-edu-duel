"""通过 Nav2 执行预先标定的固定航点。"""

import argparse
import math
import sys
import time
from pathlib import Path

import yaml

from .arena import BASE_RADIUS, ROBOT_IDS, Arena, Pose, coordinates
from .peer_sync import PEER_TIMEOUT
from .status import VehicleStatus


def load_route(path, arena: Arena) -> tuple[Pose, ...]:
    entries = yaml.safe_load(Path(path).read_bytes())
    if not isinstance(entries, list) or not entries:
        raise ValueError("航点文件必须是非空的坐标列表")
    poses = tuple(coordinates(entry, 3, f"航点 {index}") for index, entry in enumerate(entries, 1))
    for index, (x, y, yaw) in enumerate(poses, 1):
        if abs(yaw) > math.pi or not arena.path_clear((x, y), (x, y), BASE_RADIUS):
            raise ValueError(f"航点 {index} 不满足场地净空或朝向范围")
    return poses


def main(args=None):
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import NavigateThroughPoses
    from rclpy.action import ActionClient
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from rclpy.utilities import remove_ros_args
    from std_msgs.msg import String

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arena", required=True)
    parser.add_argument("--map", required=True)
    parser.add_argument("--robot-id", choices=ROBOT_IDS, required=True)
    parser.add_argument("--route", required=True)
    parser.add_argument("--simulation", action="store_true")
    parser.add_argument("--timeout", type=float, default=90.0, help="导航总时限（秒）")
    arguments = parser.parse_args(remove_ros_args()[1:] if args is None else args)
    if not math.isfinite(arguments.timeout) or arguments.timeout <= 0:
        parser.error("导航时限必须是有限正数")
    arena = Arena.load(arguments.arena, arguments.map)
    waypoints = load_route(arguments.route, arena)
    deadline = time.monotonic() + arguments.timeout
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node(
        f"duel_route_{arguments.robot_id}",
        parameter_overrides=[Parameter("use_sim_time", value=arguments.simulation)],
    )
    client = ActionClient(node, NavigateThroughPoses, "/navigate_through_poses")
    last_status = None
    status_at = -math.inf
    handle = None
    result = None

    def observe(message):
        nonlocal last_status, status_at
        status = VehicleStatus.decode(message.data, arguments.robot_id, arena)
        if status is None:
            return
        if (
            last_status is not None
            and status.session_id == last_status.session_id
            and status.seq <= last_status.seq
        ):
            return
        last_status, status_at = status, time.monotonic()

    def can_move():
        return (
            last_status is not None
            and last_status.online
            and last_status.motion_enabled is True
            and time.monotonic() - status_at < PEER_TIMEOUT
        )

    def ready():
        return can_move() and client.server_is_ready() and node.get_clock().now().nanoseconds > 0

    node.create_subscription(String, "/duel/local_status", observe, qos_profile_sensor_data)
    try:
        while time.monotonic() < deadline and not ready():
            rclpy.spin_once(node, timeout_sec=0.05)
        if not ready():
            print("等待 Nav2、双车在线及运动使能超时", file=sys.stderr)
            return 1

        session = last_status.session_id
        goal = NavigateThroughPoses.Goal()
        stamp = node.get_clock().now().to_msg()
        for x, y, yaw in waypoints:
            pose = PoseStamped()
            pose.header.frame_id, pose.header.stamp = "map", stamp
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            pose.pose.orientation.z = math.sin(yaw / 2)
            pose.pose.orientation.w = math.cos(yaw / 2)
            goal.poses.append(pose)
        sent = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(
            node, sent, timeout_sec=max(0.0, min(3.0, deadline - time.monotonic()))
        )
        if not sent.done() or not sent.result().accepted:
            print("Nav2 未确认接受固定航点", file=sys.stderr)
            return 1
        handle = sent.result()
        result = handle.get_result_async()
        print("固定航点导航已启动", flush=True)
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if not can_move() or last_status.session_id != session:
                print("联机状态或运动使能失效，正在取消导航", file=sys.stderr)
                return 1
            if result.done():
                if result.result().status == GoalStatus.STATUS_SUCCEEDED:
                    print("固定航点导航已完成")
                    return 0
                print(f"Nav2 导航结束，状态码 {result.result().status}", file=sys.stderr)
                return 1
        print("固定航点导航超时，正在取消导航", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130
    finally:
        try:
            if handle is not None and (result is None or not result.done()):
                cancel = handle.cancel_goal_async()
                rclpy.spin_until_future_complete(node, cancel, timeout_sec=3.0)
                if result is not None and cancel.done() and cancel.result().goals_canceling:
                    rclpy.spin_until_future_complete(node, result, timeout_sec=3.0)
                    if result.done() and result.result().status == GoalStatus.STATUS_CANCELED:
                        print("Nav2 已取消固定航点任务", file=sys.stderr)
                    if not result.done():
                        print("Nav2 已受理取消，尚未收到最终结果", file=sys.stderr)
        finally:
            client.destroy()
            node.destroy_node()
            rclpy.try_shutdown()
