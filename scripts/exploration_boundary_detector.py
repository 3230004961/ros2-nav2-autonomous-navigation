#!/usr/bin/env python3
import math
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener


class ExplorationBoundaryDetector(Node):
    """Detect the robot-connected enclosed region from a SLAM OccupancyGrid."""

    def __init__(self):
        super().__init__('exploration_boundary_detector')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('occupied_threshold', 50)
        self.declare_parameter('min_enclosed_area', 8.0)
        self.declare_parameter('min_extent', 2.0)
        self.declare_parameter('confirmation_updates', 5)
        self.declare_parameter('stability_duration_sec', 5.0)
        self.declare_parameter('boundary_line_width', 0.025)
        self.declare_parameter('dilation_cells', 1)
        self.declare_parameter('max_map_cells', 1500000)

        self.map_topic = str(self.get_parameter('map_topic').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.occupied_threshold = int(
            self.get_parameter('occupied_threshold').value)
        self.min_enclosed_area = float(
            self.get_parameter('min_enclosed_area').value)
        self.min_extent = float(self.get_parameter('min_extent').value)
        self.confirmation_updates = max(
            2, int(self.get_parameter('confirmation_updates').value))
        self.stability_duration_sec = max(
            0.1, float(self.get_parameter('stability_duration_sec').value))
        self.boundary_line_width = max(
            0.005, float(self.get_parameter('boundary_line_width').value))
        self.dilation_cells = max(
            0, int(self.get_parameter('dilation_cells').value))
        self.max_map_cells = int(
            self.get_parameter('max_map_cells').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        reliable_latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.marker_pub = self.create_publisher(
            MarkerArray, '/exploration_boundary/markers',
            reliable_latched_qos)
        self.status_pub = self.create_publisher(
            DiagnosticArray, '/exploration_boundary/status',
            reliable_latched_qos)
        self.classification_pub = self.create_publisher(
            MarkerArray, '/exploration_boundary/classification_markers',
            reliable_latched_qos)
        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self.on_map,
            reliable_latched_qos)

        self.state = 'BOUNDARY_NOT_FOUND'
        self.previous_signature = None
        self.stable_updates = 0
        self.stable_since = None
        self.stable_duration = 0.0
        self.last_map_received_monotonic = None
        self.latest_frame = 'map'
        self.latest_metrics = {
            'area': 0.0,
            'known_free': 0.0,
            'unknown': 0.0,
            'uncertain': 0.0,
            'coverage': 0.0,
            'bounds': (0.0, 0.0, 0.0, 0.0),
        }
        self.latest_boundary_segments = 0
        self.latest_visualization = None
        self.latest_classification = None
        self.latest_classification_metrics = {
            'cell_area': 0.0,
            'wall_cells': 0,
            'wall_components': 0,
            'obstacle_cells': 0,
            'obstacle_components': 0,
            'known_free_cells': 0,
            'unknown_cells': 0,
            'uncertain_cells': 0,
            'effective_region_cells': 0,
            'safety_margin_cells': 0,
            'excluded_unknown_cells': 0,
        }
        self.status_reason = 'No valid map received yet'

        self.last_reported_state = None
        self.last_not_found_reason = None
        self.last_log_time = 0.0
        self.tf_warning_shown = False

        self.status_timer = self.create_timer(
            1.0, self._on_status_timer)

        self.get_logger().info(
            'Boundary detector started: '
            f'map={self.map_topic}, base_frame={self.base_frame}, '
            f'min_area={self.min_enclosed_area:.1f}m^2, '
            f'confirmation_updates={self.confirmation_updates}, '
            f'stability_duration={self.stability_duration_sec:.1f}s, '
            'map_qos=RELIABLE+TRANSIENT_LOCAL')

    @staticmethod
    def _yaw(q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    @staticmethod
    def _grid_to_world(gx, gy, origin, c, s, res):
        lx, ly = gx * res, gy * res
        return origin.x + c * lx - s * ly, origin.y + s * lx + c * ly

    @staticmethod
    def _world_to_grid(wx, wy, origin, c, s, res):
        dx, dy = wx - origin.x, wy - origin.y
        gx = (c * dx + s * dy) / res
        gy = (-s * dx + c * dy) / res
        return int(math.floor(gx)), int(math.floor(gy))

    def _robot_position(self, frame):
        try:
            tf = self.tf_buffer.lookup_transform(
                frame, self.base_frame, Time())
            self.tf_warning_shown = False
            return tf.transform.translation.x, tf.transform.translation.y
        except Exception as exc:
            if not self.tf_warning_shown:
                self.get_logger().warning(
                    f'Waiting for {frame} -> {self.base_frame} TF: {exc}')
                self.tf_warning_shown = True
            return None

    def _dilate(self, occupied, width, height):
        radius = self.dilation_cells
        if radius == 0:
            return bytearray(occupied)

        blocked = bytearray(width * height)
        offsets = [(dx, dy)
                   for dy in range(-radius, radius + 1)
                   for dx in range(-radius, radius + 1)]
        for i, value in enumerate(occupied):
            if not value:
                continue
            x, y = i % width, i // width
            for dx, dy in offsets:
                nx, ny = x + dx, y + dy
                if 0 <= nx < width and 0 <= ny < height:
                    blocked[ny * width + nx] = 1
        return blocked

    def _outside_flood_fill(self, blocked, width, height):
        # Add a virtual one-cell ring around the map. This avoids assuming
        # that the SLAM map has a known, fixed world-coordinate boundary.
        pw, ph = width + 2, height + 2
        padded_blocked = bytearray(pw * ph)

        for y in range(height):
            src = y * width
            dst = (y + 1) * pw + 1
            padded_blocked[dst:dst + width] = blocked[src:src + width]

        outside = bytearray(pw * ph)
        queue = deque()

        def seed(i):
            if not padded_blocked[i] and not outside[i]:
                outside[i] = 1
                queue.append(i)

        for x in range(pw):
            seed(x)
            seed((ph - 1) * pw + x)
        for y in range(1, ph - 1):
            seed(y * pw)
            seed(y * pw + pw - 1)

        while queue:
            i = queue.popleft()
            x, y = i % pw, i // pw
            for ni in (i - 1 if x > 0 else -1,
                       i + 1 if x + 1 < pw else -1,
                       i - pw if y > 0 else -1,
                       i + pw if y + 1 < ph else -1):
                if ni >= 0 and not padded_blocked[ni] and not outside[ni]:
                    outside[ni] = 1
                    queue.append(ni)

        return padded_blocked, outside, pw

    def _robot_component(self, region, width, height, sx, sy, radius):
        start = None
        best = float('inf')
        for y in range(max(0, sy - radius), min(height, sy + radius + 1)):
            for x in range(max(0, sx - radius), min(width, sx + radius + 1)):
                i = y * width + x
                if region[i]:
                    d2 = (x - sx) ** 2 + (y - sy) ** 2
                    if d2 < best:
                        start, best = i, d2
        if start is None or best > radius * radius:
            return None

        visited = bytearray(width * height)
        visited[start] = 1
        queue = deque([start])
        component = []

        while queue:
            i = queue.popleft()
            component.append(i)
            x, y = i % width, i // width
            neighbors = []
            if x > 0:
                neighbors.append(i - 1)
            if x + 1 < width:
                neighbors.append(i + 1)
            if y > 0:
                neighbors.append(i - width)
            if y + 1 < height:
                neighbors.append(i + width)
            for ni in neighbors:
                if region[ni] and not visited[ni]:
                    visited[ni] = 1
                    queue.append(ni)
        return component

    def _metrics(self, component, data, width, res, origin, c, s):
        known_free = 0
        unknown = 0
        uncertain = 0
        min_x, min_y = width, len(data)
        max_x = max_y = 0

        for i in component:
            value = int(data[i])
            if 0 <= value <= 20:
                known_free += 1
            elif value < 0:
                unknown += 1
            elif 20 < value < 50:
                uncertain += 1

            x, y = i % width, i // width
            min_x, min_y = min(min_x, x), min(min_y, y)
            max_x, max_y = max(max_x, x), max(max_y, y)

        total = known_free + unknown + uncertain
        area = total * res * res
        coverage = 100.0 * known_free / total if total else 0.0

        corners = [
            self._grid_to_world(min_x, min_y, origin, c, s, res),
            self._grid_to_world(max_x + 1, min_y, origin, c, s, res),
            self._grid_to_world(min_x, max_y + 1, origin, c, s, res),
            self._grid_to_world(max_x + 1, max_y + 1, origin, c, s, res),
        ]
        xs, ys = [p[0] for p in corners], [p[1] for p in corners]
        bounds = min(xs), min(ys), max(xs), max(ys)

        return {
            'area': area,
            'known_free': known_free * res * res,
            'unknown': unknown * res * res,
            'uncertain': uncertain * res * res,
            'coverage': coverage,
            'bounds': bounds,
        }

    @staticmethod
    def _stable(old, new):
        if old is None:
            return False
        old_area, old_bounds = old
        new_area, new_bounds = new
        if abs(new_area - old_area) / max(old_area, 1e-6) > 0.15:
            return False

        old_cx = (old_bounds[0] + old_bounds[2]) / 2.0
        old_cy = (old_bounds[1] + old_bounds[3]) / 2.0
        new_cx = (new_bounds[0] + new_bounds[2]) / 2.0
        new_cy = (new_bounds[1] + new_bounds[3]) / 2.0
        if math.hypot(new_cx - old_cx, new_cy - old_cy) > 0.20:
            return False

        old_w, old_h = old_bounds[2] - old_bounds[0], old_bounds[3] - old_bounds[1]
        new_w, new_h = new_bounds[2] - new_bounds[0], new_bounds[3] - new_bounds[1]
        return abs(new_w - old_w) <= 0.25 and abs(new_h - old_h) <= 0.25

    def _exterior_wall_mask(self, padded_blocked, outside, pw):
        """Return blocked-cell components connected to the map exterior.

        Interior furniture/obstacles are deliberately excluded unless they
        are physically connected to the exterior wall component.
        """
        ph = len(padded_blocked) // pw
        exterior_wall = bytearray(len(padded_blocked))
        queue = deque()

        # Seed every blocked cell that directly borders exterior free space.
        for i, is_blocked in enumerate(padded_blocked):
            if not is_blocked:
                continue
            x, y = i % pw, i // pw
            neighbors = []
            if x > 0:
                neighbors.append(i - 1)
            if x + 1 < pw:
                neighbors.append(i + 1)
            if y > 0:
                neighbors.append(i - pw)
            if y + 1 < ph:
                neighbors.append(i + pw)
            if any(outside[ni] for ni in neighbors):
                exterior_wall[i] = 1
                queue.append(i)

        # Expand through the complete 4-connected wall component. This is
        # what lets us draw the room-facing side of multi-cell-thick walls.
        while queue:
            i = queue.popleft()
            x, y = i % pw, i // pw
            neighbors = []
            if x > 0:
                neighbors.append(i - 1)
            if x + 1 < pw:
                neighbors.append(i + 1)
            if y > 0:
                neighbors.append(i - pw)
            if y + 1 < ph:
                neighbors.append(i + pw)
            for ni in neighbors:
                if padded_blocked[ni] and not exterior_wall[ni]:
                    exterior_wall[ni] = 1
                    queue.append(ni)

        return exterior_wall

    def _boundary_segments(self, component, width, height,
                           exterior_wall, pw):
        """Extract grid-edge segments along the room-facing outer wall.

        Coordinates are grid-boundary coordinates, not cell centers. The
        returned segments can be rendered as a thin LINE_LIST in RViz.
        """
        segments = set()
        for i in component:
            x, y = i % width, i // width
            px, py = x + 1, y + 1

            if exterior_wall[py * pw + (px - 1)]:
                segments.add(((x, y), (x, y + 1)))
            if exterior_wall[py * pw + (px + 1)]:
                segments.add(((x + 1, y), (x + 1, y + 1)))
            if exterior_wall[(py - 1) * pw + px]:
                segments.add(((x, y), (x + 1, y)))
            if exterior_wall[(py + 1) * pw + px]:
                segments.add(((x, y + 1), (x + 1, y + 1)))

        return sorted(segments)

    @staticmethod
    def _count_components(mask, width, height):
        """Count 4-connected components in a flat binary mask."""
        visited = bytearray(width * height)
        count = 0

        for start, value in enumerate(mask):
            if not value or visited[start]:
                continue

            count += 1
            visited[start] = 1
            queue = deque([start])

            while queue:
                i = queue.popleft()
                x, y = i % width, i // width
                if x > 0:
                    ni = i - 1
                    if mask[ni] and not visited[ni]:
                        visited[ni] = 1
                        queue.append(ni)
                if x + 1 < width:
                    ni = i + 1
                    if mask[ni] and not visited[ni]:
                        visited[ni] = 1
                        queue.append(ni)
                if y > 0:
                    ni = i - width
                    if mask[ni] and not visited[ni]:
                        visited[ni] = 1
                        queue.append(ni)
                if y + 1 < height:
                    ni = i + width
                    if mask[ni] and not visited[ni]:
                        visited[ni] = 1
                        queue.append(ni)

        return count

    def _classify_cells(self, data, width, height, res, component,
                        occupied, blocked, origin, c, s):
        """Classify raw walls/obstacles separately from inflated safety cells.

        Structural walls are occupied components connected to free space
        outside the enclosure. Raw OccupancyGrid cells are used for this
        semantic classification; the inflated mask is used only for safety.
        """
        raw_padded, raw_outside, raw_pw = self._outside_flood_fill(
            occupied, width, height)
        raw_exterior_wall = self._exterior_wall_mask(
            raw_padded, raw_outside, raw_pw)

        wall_mask = bytearray(width * height)
        obstacle_mask = bytearray(width * height)
        for i, is_occupied in enumerate(occupied):
            if not is_occupied:
                continue
            x, y = i % width, i // width
            if raw_exterior_wall[(y + 1) * raw_pw + (x + 1)]:
                wall_mask[i] = 1
            else:
                obstacle_mask[i] = 1

        component_mask = bytearray(width * height)
        free_indices = []
        unknown_indices = []
        uncertain_indices = []
        for i in component:
            component_mask[i] = 1
            value = int(data[i])
            if 0 <= value <= 20:
                free_indices.append(i)
            elif value < 0:
                unknown_indices.append(i)
            elif 20 < value < 50:
                uncertain_indices.append(i)

        wall_indices = [i for i, value in enumerate(wall_mask) if value]
        obstacle_indices = [
            i for i, value in enumerate(obstacle_mask) if value]
        safety_margin_cells = sum(
            1 for i, value in enumerate(blocked)
            if value and not occupied[i])
        excluded_unknown_cells = sum(
            1 for i, value in enumerate(data)
            if int(value) < 0 and not component_mask[i])

        metrics = {
            'cell_area': res * res,
            'wall_cells': len(wall_indices),
            'wall_components': self._count_components(
                wall_mask, width, height),
            'obstacle_cells': len(obstacle_indices),
            'obstacle_components': self._count_components(
                obstacle_mask, width, height),
            'known_free_cells': len(free_indices),
            'unknown_cells': len(unknown_indices),
            'uncertain_cells': len(uncertain_indices),
            'effective_region_cells': len(component),
            'safety_margin_cells': safety_margin_cells,
            'excluded_unknown_cells': excluded_unknown_cells,
        }

        visual = {
            'wall_indices': wall_indices,
            'obstacle_indices': obstacle_indices,
            'free_indices': free_indices,
            'unknown_indices': unknown_indices,
            'uncertain_indices': uncertain_indices,
            'width': width,
            'height': height,
            'resolution': res,
            'origin': Point(x=origin.x, y=origin.y, z=origin.z),
            'cos_yaw': c,
            'sin_yaw': s,
        }
        return metrics, visual

    def _classification_cube(self, marker_id, namespace, indices, visual,
                             color, alpha, z):
        marker = Marker()
        marker.header.frame_id = self.latest_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        res = visual['resolution']
        marker.scale.x = res * 0.92
        marker.scale.y = res * 0.92
        marker.scale.z = 0.025
        marker.color.r, marker.color.g, marker.color.b = color
        marker.color.a = alpha

        width = visual['width']
        origin = visual['origin']
        c, s = visual['cos_yaw'], visual['sin_yaw']
        for i in indices:
            gx, gy = i % width, i // width
            wx, wy = self._grid_to_world(
                gx + 0.5, gy + 0.5, origin, c, s, res)
            point = Point()
            point.x, point.y, point.z = wx, wy, z
            marker.points.append(point)
        return marker

    def _publish_classification_markers(self):
        if self.latest_classification is None:
            return

        v = self.latest_classification
        arr = MarkerArray()
        arr.markers.append(self._classification_cube(
            0, 'effective_known_free', v['free_indices'], v,
            (0.10, 0.85, 0.25), 0.14, 0.015))
        arr.markers.append(self._classification_cube(
            1, 'effective_unknown', v['unknown_indices'], v,
            (1.00, 0.78, 0.05), 0.90, 0.045))
        arr.markers.append(self._classification_cube(
            2, 'structural_walls', v['wall_indices'], v,
            (0.10, 0.40, 1.00), 0.88, 0.055))
        arr.markers.append(self._classification_cube(
            3, 'interior_obstacles', v['obstacle_indices'], v,
            (1.00, 0.32, 0.04), 0.92, 0.065))
        arr.markers.append(self._classification_cube(
            4, 'uncertain_cells', v['uncertain_indices'], v,
            (0.72, 0.25, 0.90), 0.85, 0.075))
        self.classification_pub.publish(arr)

    def _clear_classification_markers(self):
        arr = MarkerArray()
        namespaces = (
            'effective_known_free', 'effective_unknown',
            'structural_walls', 'interior_obstacles',
            'uncertain_cells')
        for marker_id, namespace in enumerate(namespaces):
            marker = Marker()
            marker.header.frame_id = self.latest_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = namespace
            marker.id = marker_id
            marker.action = Marker.DELETE
            arr.markers.append(marker)
        self.classification_pub.publish(arr)

    def _publish(self, frame, state, metrics, boundary_segments, res,
                 origin, c, s):
        arr = MarkerArray()

        outline = Marker()
        outline.header.frame_id = frame
        outline.header.stamp = self.get_clock().now().to_msg()
        outline.ns, outline.id = 'exploration_boundary', 0
        outline.type, outline.action = Marker.LINE_LIST, Marker.ADD
        outline.pose.orientation.w = 1.0
        outline.scale.x = self.boundary_line_width
        if state == 'BOUNDARY_CONFIRMED':
            outline.color.r, outline.color.g, outline.color.b = 0.1, 0.9, 0.3
        else:
            outline.color.r, outline.color.g, outline.color.b = 1.0, 0.55, 0.05
        outline.color.a = 1.0

        for segment in boundary_segments:
            for gx, gy in segment:
                wx, wy = self._grid_to_world(
                    gx, gy, origin, c, s, res)
                point = Point()
                point.x, point.y, point.z = wx, wy, 0.08
                outline.points.append(point)
        arr.markers.append(outline)

        label = Marker()
        label.header.frame_id = frame
        label.header.stamp = self.get_clock().now().to_msg()
        label.ns, label.id = 'exploration_boundary', 1
        label.type, label.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        label.pose.orientation.w = 1.0
        b = metrics['bounds']
        label.pose.position.x = (b[0] + b[2]) / 2.0
        label.pose.position.y = (b[1] + b[3]) / 2.0
        label.pose.position.z = 0.65
        label.scale.z = 0.25
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0
        label.text = (
            f'{state}\nArea: {metrics["area"]:.1f} m^2\n'
            f'Coverage: {metrics["coverage"]:.1f}%\n'
            f'Stable: {self.stable_duration:.1f}/'
            f'{self.stability_duration_sec:.1f}s '
            f'({self.stable_updates}/{self.confirmation_updates})')
        arr.markers.append(label)
        self.marker_pub.publish(arr)

    def _publish_latest_markers(self):
        if self.latest_visualization is None:
            return
        v = self.latest_visualization
        self._publish(
            self.latest_frame, self.state, self.latest_metrics,
            v['segments'], v['resolution'], v['origin'],
            v['cos_yaw'], v['sin_yaw'])

    def _publish_status(self):
        if self.stable_since is None:
            self.stable_duration = 0.0
        else:
            self.stable_duration = max(
                0.0, time.monotonic() - self.stable_since)

        # Five stable map observations are still required. The additional
        # time condition prevents a burst of rapid map callbacks from
        # confirming the boundary immediately.
        if (self.state == 'BOUNDARY_CANDIDATE' and
                self.stable_updates >= self.confirmation_updates and
                self.stable_duration >= self.stability_duration_sec):
            self.state = 'BOUNDARY_CONFIRMED'
            self.status_reason = ''
            self.get_logger().info(
                'BOUNDARY_CONFIRMED: stable for '
                f'{self.stable_duration:.1f}s across '
                f'{self.stable_updates} map updates')

        self._publish_latest_markers()
        self._publish_classification_markers()

        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.latest_frame

        status = DiagnosticStatus()
        status.level = (DiagnosticStatus.OK
                        if self.state == 'BOUNDARY_CONFIRMED'
                        else DiagnosticStatus.WARN)
        status.name = 'exploration_boundary_detector'
        status.message = self.state
        if self.status_reason:
            status.message += f': {self.status_reason}'
        status.hardware_id = 'slam_toolbox_occupancy_grid'

        map_age = ('unknown' if self.last_map_received_monotonic is None
                   else f'{max(0.0, time.monotonic() - self.last_map_received_monotonic):.2f}')
        values = {
            'state': self.state,
            'coverage': f'{self.latest_metrics["coverage"]:.2f}',
            'area': f'{self.latest_metrics["area"]:.3f}',
            'stable_duration': f'{self.stable_duration:.2f}',
            'known_free': f'{self.latest_metrics["known_free"]:.3f}',
            'unknown': f'{self.latest_metrics["unknown"]:.3f}',
            'uncertain': f'{self.latest_metrics.get("uncertain", 0.0):.3f}',
            'wall_cells': str(self.latest_classification_metrics['wall_cells']),
            'wall_components': str(self.latest_classification_metrics['wall_components']),
            'wall_area_m2': f'{self.latest_classification_metrics["wall_cells"] * self.latest_classification_metrics["cell_area"]:.3f}',
            'obstacle_cells': str(self.latest_classification_metrics['obstacle_cells']),
            'obstacle_area_m2': f'{self.latest_classification_metrics["obstacle_cells"] * self.latest_classification_metrics["cell_area"]:.3f}',
            'obstacle_components': str(self.latest_classification_metrics['obstacle_components']),
            'known_free_cells': str(self.latest_classification_metrics['known_free_cells']),
            'unknown_cells': str(self.latest_classification_metrics['unknown_cells']),
            'uncertain_cells': str(self.latest_classification_metrics['uncertain_cells']),
            'effective_region_cells': str(self.latest_classification_metrics['effective_region_cells']),
            'safety_margin_cells': str(self.latest_classification_metrics['safety_margin_cells']),
            'safety_margin_area_m2': f'{self.latest_classification_metrics["safety_margin_cells"] * self.latest_classification_metrics["cell_area"]:.3f}',
            'excluded_unknown_cells': str(self.latest_classification_metrics['excluded_unknown_cells']),
            'stable_updates': str(self.stable_updates),
            'confirmation_updates': str(self.confirmation_updates),
            'stability_duration_required': f'{self.stability_duration_sec:.2f}',
            'boundary_segments': str(self.latest_boundary_segments),
            'map_age_sec': map_age,
            'reason': self.status_reason or 'none',
        }
        for key, value in values.items():
            item = KeyValue()
            item.key = key
            item.value = value
            status.values.append(item)

        msg.status.append(status)
        self.status_pub.publish(msg)

    def _on_status_timer(self):
        self._publish_status()

    def _clear_markers(self):
        arr = MarkerArray()
        for marker_id in (0, 1):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns, m.id = 'exploration_boundary', marker_id
            m.action = Marker.DELETE
            arr.markers.append(m)
        self.marker_pub.publish(arr)

    def _not_found(self, reason):
        now = time.monotonic()
        if reason != self.last_not_found_reason or now - self.last_log_time >= 5.0:
            self.get_logger().info(f'BOUNDARY_NOT_FOUND: {reason}')
            self.last_not_found_reason = reason
            self.last_log_time = now

        self.state = 'BOUNDARY_NOT_FOUND'
        self.status_reason = reason
        self.last_reported_state = 'BOUNDARY_NOT_FOUND'
        self.previous_signature = None
        self.stable_updates = 0
        self.stable_since = None
        self.stable_duration = 0.0
        self.latest_metrics = {
            'area': 0.0,
            'known_free': 0.0,
            'unknown': 0.0,
            'uncertain': 0.0,
            'coverage': 0.0,
            'bounds': (0.0, 0.0, 0.0, 0.0),
        }
        self.latest_boundary_segments = 0
        self.latest_visualization = None
        self.latest_classification = None
        self.latest_classification_metrics = {
            'cell_area': 0.0,
            'wall_cells': 0,
            'wall_components': 0,
            'obstacle_cells': 0,
            'obstacle_components': 0,
            'known_free_cells': 0,
            'unknown_cells': 0,
            'uncertain_cells': 0,
            'effective_region_cells': 0,
            'safety_margin_cells': 0,
            'excluded_unknown_cells': 0,
        }
        self._clear_markers()
        self._clear_classification_markers()

    def on_map(self, msg):
        self.last_map_received_monotonic = time.monotonic()
        width, height = int(msg.info.width), int(msg.info.height)
        res, data = float(msg.info.resolution), msg.data
        if width <= 0 or height <= 0 or res <= 0.0:
            self._not_found('Invalid OccupancyGrid dimensions or resolution')
            return
        if width * height != len(data):
            self._not_found('OccupancyGrid data length does not match dimensions')
            self.get_logger().warning('Invalid OccupancyGrid dimensions')
            return
        if width * height > self.max_map_cells:
            self._not_found(
                f'Map has {width * height} cells; '
                'increase max_map_cells if needed')
            return

        frame = msg.header.frame_id or 'map'
        self.latest_frame = frame
        robot = self._robot_position(frame)
        if robot is None:
            self._not_found('Robot pose TF is not currently available')
            return

        origin = msg.info.origin.position
        yaw = self._yaw(msg.info.origin.orientation)
        c, s = math.cos(yaw), math.sin(yaw)
        sx, sy = self._world_to_grid(
            robot[0], robot[1], origin, c, s, res)
        if not (0 <= sx < width and 0 <= sy < height):
            self._not_found('Robot pose is outside the current map')
            return

        occupied = bytearray(
            1 if int(v) >= self.occupied_threshold else 0 for v in data)
        blocked = self._dilate(occupied, width, height)
        padded_blocked, outside, pw = self._outside_flood_fill(
            blocked, width, height)

        # The robot-connected exploration region must exclude the dilated
        # wall/safety band. Unknown cells inside the enclosure remain part
        # of the region and count as unexplored area.
        region = bytearray(width * height)
        for y in range(height):
            row, prow = y * width, (y + 1) * pw + 1
            for x in range(width):
                i = row + x
                if not outside[prow + x] and not blocked[i]:
                    region[i] = 1

        radius = max(2, int(math.ceil(0.50 / res)))
        component = self._robot_component(
            region, width, height, sx, sy, radius)
        if not component:
            self._not_found('No enclosed region connected to robot pose')
            return

        metrics = self._metrics(
            component, data, width, res, origin, c, s)
        b = metrics['bounds']
        extent_x, extent_y = b[2] - b[0], b[3] - b[1]
        if metrics['area'] < self.min_enclosed_area or \
                min(extent_x, extent_y) < self.min_extent:
            self._not_found(
                f'Candidate too small: {metrics["area"]:.2f}m^2, '
                f'{extent_x:.2f}x{extent_y:.2f}m')
            return

        signature = metrics['area'], b
        now = time.monotonic()
        if self._stable(self.previous_signature, signature):
            self.stable_updates += 1
        else:
            self.stable_updates = 1
            self.stable_since = now
        if self.stable_since is None:
            self.stable_since = now
        self.stable_duration = max(0.0, now - self.stable_since)
        self.previous_signature = signature

        state = ('BOUNDARY_CONFIRMED'
                 if self.stable_updates >= self.confirmation_updates and
                 self.stable_duration >= self.stability_duration_sec
                 else 'BOUNDARY_CANDIDATE')
        self.state = state
        self.status_reason = ''
        self.last_not_found_reason = None
        self.latest_metrics = metrics

        # Only the blocked wall component that touches the map exterior is
        # eligible. This keeps isolated furniture/obstacles out of the
        # outer boundary while still tracing the room-facing side of thick
        # connected walls.
        exterior_wall = self._exterior_wall_mask(
            padded_blocked, outside, pw)
        boundary_segments = self._boundary_segments(
            component, width, height, exterior_wall, pw)
        self.latest_boundary_segments = len(boundary_segments)

        classification_metrics, classification_visual = self._classify_cells(
            data, width, height, res, component,
            occupied, blocked, origin, c, s)
        self.latest_classification_metrics = classification_metrics
        self.latest_classification = classification_visual

        self.latest_visualization = {
            'segments': boundary_segments,
            'resolution': res,
            'origin': Point(
                x=origin.x, y=origin.y, z=origin.z),
            'cos_yaw': c,
            'sin_yaw': s,
        }
        self._publish_latest_markers()

        now = time.monotonic()
        if state != self.last_reported_state or now - self.last_log_time >= 5.0:
            self.get_logger().info(
                f'{state}: enclosed_area={metrics["area"]:.2f}m^2, '
                f'known_free={metrics["known_free"]:.2f}m^2, '
                f'unknown={metrics["unknown"]:.2f}m^2, '
                f'coverage={metrics["coverage"]:.1f}%, '
                f'extent={extent_x:.2f}x{extent_y:.2f}m, '
                f'stable={self.stable_updates}/'
                f'{self.confirmation_updates}, '
                f'stable_duration={self.stable_duration:.2f}/'
                f'{self.stability_duration_sec:.2f}s, '
                f'boundary_segments={len(boundary_segments)}, '
                f'walls={classification_metrics["wall_cells"]}, '
                f'obstacles={classification_metrics["obstacle_cells"]}, '
                f'obstacle_components={classification_metrics["obstacle_components"]}, '
                f'free={classification_metrics["known_free_cells"]}, '
                f'unknown={classification_metrics["unknown_cells"]}, '
                f'safety_margin={classification_metrics["safety_margin_cells"]}')
            self.last_reported_state = state
            self.last_log_time = now


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationBoundaryDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._clear_markers()
        node._clear_classification_markers()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
