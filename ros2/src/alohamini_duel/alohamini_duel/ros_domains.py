"""为双车进程分别创建两个 ROS 上下文。"""

import argparse
import signal

from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.utilities import remove_ros_args


def dual_domain_parser(description, defaults):
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--arena", required=True)
    parser.add_argument("--map", required=True)
    parser.add_argument("--domains", nargs=2, type=int, default=defaults)
    return parser


def parse_dual_domain_args(parser, args):
    arguments = parser.parse_args(remove_ros_args()[1:] if args is None else args)
    if len(set(arguments.domains)) != 2 or not all(
        0 <= domain <= 101 for domain in arguments.domains
    ):
        parser.error("需要两个不同的 ROS 域编号，范围为 0 至 101")
    return arguments


def attach_domain(resources, domain, create_node):
    context = Context()
    context.init(args=[], domain_id=domain)
    resources.callback(context.try_shutdown)
    node = create_node(context)
    resources.callback(node.destroy_node)
    executor = SingleThreadedExecutor(context=context)
    resources.callback(executor.shutdown)
    executor.add_node(node)
    return node, executor


def watch_shutdown(resources):
    requested = False

    def request_stop(_signum, _frame):
        nonlocal requested
        requested = True

    # 在循环边界关闭 ROS 上下文，避免信号打断 Humble 的消息转换。
    for signum in (signal.SIGINT, signal.SIGTERM):
        previous = signal.signal(signum, request_stop)
        resources.callback(signal.signal, signum, previous)
    return lambda: requested
