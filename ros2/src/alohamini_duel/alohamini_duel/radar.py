"""比较雷达回波与静态地图，并发布只读诊断。"""

import json
import math
from dataclasses import dataclass

from .arena import Point, Pose, local_point
from .status import fresh_stamp, yaw_from_quaternion

OCCUPIED = 65


@dataclass(frozen=True)
class StaticGrid:
    origin: Pose
    resolution: float
    width: int
    height: int
    values: tuple[int, ...]

    def cell(self, x: int, y: int) -> int:
        if 0 <= x < self.width and 0 <= y < self.height:
            return self.values[y * self.width + x]
        return -1

    def classify(self, sensor: Point, endpoint: Point, tolerance: float) -> str:
        x, y = (coordinate / self.resolution for coordinate in endpoint)
        col, row = math.floor(x), math.floor(y)
        cell = self.cell(col, row)
        if cell < 0:
            return "unmapped"
        radius = tolerance / self.resolution
        columns = range(max(0, math.floor(x - radius)), min(self.width, math.floor(x + radius) + 1))
        rows = range(max(0, math.floor(y - radius)), min(self.height, math.floor(y + radius) + 1))
        for j in rows:
            dy = max(j - y, y - (j + 1), 0.0)
            for i in columns:
                dx = max(i - x, x - (i + 1), 0.0)
                if self.cell(i, j) >= OCCUPIED and dx * dx + dy * dy <= radius**2:
                    return "background"
        sx, sy = (coordinate / self.resolution for coordinate in sensor)
        i, j = math.floor(sx), math.floor(sy)
        dx, dy = x - sx, y - sy
        step_x, step_y = (1 if dx > 0 else -1), (1 if dy > 0 else -1)
        stride_x = abs(1 / dx) if dx else math.inf
        stride_y = abs(1 / dy) if dy else math.inf
        crossing_x = (i + (step_x > 0) - sx) / dx if dx else math.inf
        crossing_y = (j + (step_y > 0) - sy) / dy if dy else math.inf
        # 按射线穿过栅格边界的顺序前进，避免定距采样漏掉擦过的占用格。
        while (i, j) != (col, row):
            sample = self.cell(i, j)
            if sample < 0:
                return "unmapped"
            if sample >= OCCUPIED:
                return "occluded"
            if i == col or (j != row and crossing_y < crossing_x):
                j += step_y
                crossing_y += stride_y
                continue
            if j != row and crossing_x == crossing_y:
                j += step_y
                crossing_y += stride_y
            i += step_x
            crossing_x += stride_x
        return "unexpected"


def compare_scan(grid, laser_pose, ranges, angle_min, angle_increment, limits, tolerance):
    counts = dict.fromkeys(("background", "unexpected", "occluded", "unmapped", "invalid"), 0)
    points = []
    sensor = local_point(grid.origin, laser_pose[:2])
    for index, distance in enumerate(ranges):
        if not math.isfinite(distance) or not limits[0] <= distance <= limits[1]:
            counts["invalid"] += 1
            continue
        heading = laser_pose[2] + angle_min + index * angle_increment
        local_heading = heading - grid.origin[2]
        endpoint = (
            sensor[0] + distance * math.cos(local_heading),
            sensor[1] + distance * math.sin(local_heading),
        )
        kind = grid.classify(sensor, endpoint, tolerance)
        counts[kind] += 1
        if kind == "unexpected":
            points.append((
                laser_pose[0] + distance * math.cos(heading),
                laser_pose[1] + distance * math.sin(heading),
            ))
    return points, counts


def main(args=None):
    from geometry_msgs.msg import Point32
    from nav_msgs.msg import OccupancyGrid
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan, PointCloud
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformException, TransformListener

    from .ros_domains import spin_until_shutdown

    class RadarDiagnostic(Node):
        def __init__(self):
            super().__init__("alohamini_duel_radar")
            self.declare_parameter("map_tolerance_m", 0.12)
            self.tolerance = self.get_parameter("map_tolerance_m").value
            if not math.isfinite(self.tolerance) or self.tolerance <= 0:
                raise ValueError("地图容差必须是有限正数")
            self.grid = None
            self.pending = None
            self.tf = Buffer(node=self)
            self.listener = TransformListener(self.tf, self)
            self.summary = self.create_publisher(String, "/duel/radar_diagnostics", 1)
            self.cloud = self.create_publisher(PointCloud, "/duel/radar_unexpected", 1)
            self.create_subscription(
                OccupancyGrid, "/map", self.update_map,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            )
            self.create_subscription(
                LaserScan, "/scan_filtered", self.update_scan, qos_profile_sensor_data
            )
            self.create_timer(0.02, self.compare)

        def update_map(self, message):
            info = message.info
            position, orientation = info.origin.position, info.origin.orientation
            origin = (position.x, position.y, yaw_from_quaternion(orientation))
            if (
                message.header.frame_id != "map"
                or not all(math.isfinite(value) for value in (*origin, info.resolution))
                or info.resolution <= 0
                or info.width * info.height == 0
                or len(message.data) != info.width * info.height
            ):
                self.grid = None
                self.get_logger().warning("雷达诊断收到无效地图", throttle_duration_sec=3.0)
                return
            self.grid = StaticGrid(
                origin, info.resolution, info.width, info.height, tuple(message.data)
            )

        def update_scan(self, message):
            self.pending = message

        def compare(self):
            scan = self.pending
            if scan is None or self.grid is None:
                return
            stamp = fresh_stamp(scan.header, "laser_frame", self.get_clock().now().nanoseconds, 0.5)
            if stamp is None or not math.isfinite(scan.angle_increment) or scan.angle_increment <= 0:
                self.pending = None
                return
            try:
                transform = self.tf.lookup_transform(
                    "map", scan.header.frame_id, Time.from_msg(scan.header.stamp)
                )
            except TransformException:
                return
            self.pending = None
            translation, rotation = transform.transform.translation, transform.transform.rotation
            pose = (translation.x, translation.y, yaw_from_quaternion(rotation))
            if not all(math.isfinite(value) for value in (*pose, scan.angle_min)):
                return
            points, counts = compare_scan(
                self.grid, pose, scan.ranges, scan.angle_min, scan.angle_increment,
                (scan.range_min, scan.range_max), self.tolerance,
            )
            cloud = PointCloud()
            cloud.header.frame_id, cloud.header.stamp = "map", scan.header.stamp
            cloud.points = [Point32(x=x, y=y, z=translation.z) for x, y in points]
            self.cloud.publish(cloud)
            self.summary.publish(String(data=json.dumps({"stamp_ns": stamp, **counts})))

    spin_until_shutdown(RadarDiagnostic, args)
