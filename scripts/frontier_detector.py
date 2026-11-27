#!/usr/bin/env python3

from collections import deque
from math import atan2, cos, sin

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class FrontierDetector(Node):
    def __init__(self):
        super().__init__('frontier_detector')

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)

        self.marker_pub = self.create_publisher(
            Marker, '/frontier_markers', 10)

        self.get_logger().info('Frontier detector started')

    def map_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        data = msg.data

        if width < 3 or height < 3:
            return

        # 1. 检测 Frontier：
        # 已知空闲栅格，且至少有一个四邻域栅格未知
        frontiers = set()

        for y in range(1, height - 1):
            for x in range(1, width - 1):
                idx = y * width + x

                if not 0 <= data[idx] <= 20:
                    continue

                neighbors = (
                    idx - 1, idx + 1,
                    idx - width, idx + width
                )

                if any(data[n] == -1 for n in neighbors):
                    frontiers.add(idx)

        # 2. 使用 BFS 将相邻 Frontier 聚成区域
        visited = set()
        clusters = []

        for start in frontiers:
            if start in visited:
                continue

            queue = deque([start])
            visited.add(start)
            cluster = []

            while queue:
                idx = queue.popleft()
                cluster.append(idx)

                x = idx % width
                y = idx // width

                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        if dx == 0 and dy == 0:
                            continue

                        nx, ny = x + dx, y + dy

                        if not (0 <= nx < width and
                                0 <= ny < height):
                            continue

                        nidx = ny * width + nx

                        if nidx in frontiers and nidx not in visited:
                            visited.add(nidx)
                            queue.append(nidx)

            # 忽略太小的噪声边界
            if len(cluster) >= 6:
                clusters.append(cluster)

        # 3. 发布每个 Frontier 区域的代表点
        marker = Marker()
        marker.header.frame_id = msg.header.frame_id or 'map'
        marker.header.stamp = msg.header.stamp
        marker.ns = 'frontier_candidates'
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.06
        marker.scale.y = 0.06
        marker.scale.z = 0.16

        marker.color.r = 0.1
        marker.color.g = 1.0
        marker.color.b = 0.2
        marker.color.a = 1.0

        origin = msg.info.origin.position
        q = msg.info.origin.orientation
        yaw = atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z)
        )
        resolution = msg.info.resolution

        # 显示所有 Frontier 栅格，而不是只显示聚类代表点
        for idx in sorted(frontiers):
            gx = (idx % width + 0.5) * resolution
            gy = (idx // width + 0.5) * resolution

            point = Point()
            point.x = origin.x + gx * cos(yaw) - gy * sin(yaw)
            point.y = origin.y + gx * sin(yaw) + gy * cos(yaw)
            point.z = 0.12

            marker.points.append(point)

        self.marker_pub.publish(marker)

        self.get_logger().info(
            f'Map: {width}x{height}, '
            f'frontier cells: {len(frontiers)}, '
            f'frontier regions: {len(clusters)}'
        )


def main():
    rclpy.init()
    node = FrontierDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
