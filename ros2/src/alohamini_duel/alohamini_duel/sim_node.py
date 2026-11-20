"""在两个 ROS 域发布同一个仿真世界。"""

import time
from contextlib import ExitStack

from alohamini_nav_sim.node import STEP_PERIOD, Simulator
from rclpy.executors import ExternalShutdownException

from .arena import Arena
from .ros_domains import attach_domain, parse_dual_domain_args
from .shared_sim_world import SharedDuelWorld


def main(args=None):
    arguments = parse_dual_domain_args(__doc__, args, (45, 46))
    arena = Arena.load(arguments.arena, arguments.map)
    world = SharedDuelWorld(arena)
    with ExitStack() as resources:
        domains = [
            attach_domain(
                resources,
                domain,
                lambda context, vehicle=vehicle: Simulator(context=context, shared_world=vehicle),
            )
            for vehicle, domain in zip(world.vehicles, arguments.domains, strict=True)
        ]
        nodes, executors = zip(*domains, strict=True)
        try:
            while all(node.context.ok() for node in nodes):
                start = time.monotonic()
                for executor in executors:
                    for _ in range(4):
                        executor.spin_once(timeout_sec=0.0)
                commands = (node.requested_velocity() for node in nodes)
                for node, feedback in zip(nodes, world.step(commands, STEP_PERIOD), strict=True):
                    node.publish_frame(feedback)
                time.sleep(max(0.0, STEP_PERIOD - (time.monotonic() - start)))
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
