#!/usr/bin/env python3

import math
import signal
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import Buffer, TransformListener
from nav_msgs.msg import OccupancyGrid, Odometry
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.qos import qos_profile_sensor_data
from nav2_msgs.action import NavigateToPose
from visualization_msgs.msg import Marker
from action_msgs.msg import GoalStatus


class CompletionGate:
    """Require distinct detector map observations, not timer heartbeats."""
    def __init__(self, target, hold, max_age):
        self.target, self.hold, self.max_age = target, hold, max_age
        self.reset()

    def reset(self):
        self.first = None
        self.last_obs = None
        self.count = 0
        self.last_update = None
        self.last_status = None
        self.coverage = None
        self.rejection_reason = "waiting for valid boundary observation"

    def observe(self, values, level, now):
        try:
            # ROS diagnostic_msgs/DiagnosticStatus.level is a byte field.
            # Jazzy Python messages can expose it as a single-byte bytes object.
            if isinstance(level, (bytes, bytearray)):
                if len(level) != 1:
                    raise ValueError('invalid diagnostic level length')
                level = level[0]
            if not isinstance(level, int) or level != 0:
                raise ValueError(f'diagnostic level is not OK: {level!r}')
            age = float(values['map_age_sec'])
            updates = int(values['stable_updates'])
            duration = float(values['stable_duration'])
            required = int(values['confirmation_updates'])
            required_sec = float(values['stability_duration_required'])
            free = int(values['known_free_cells'])
            total = int(values['effective_region_cells'])
            unknown = int(values['unknown_cells'])
            uncertain = int(values['uncertain_cells'])
            area = float(values['area'])
            if (values['state'] != 'BOUNDARY_CONFIRMED'
                    or values['reason'] != 'none'
                    or not all(math.isfinite(v) for v in (age, duration, required_sec, area))
                    or not 0 <= age <= self.max_age or area <= 0
                    or int(values['boundary_segments']) <= 0
                    or updates < max(5, required)
                    or duration < max(5.0, required_sec)
                    or total <= 0 or min(free, unknown, uncertain) < 0
                    or free + unknown + uncertain != total):
                raise ValueError('invalid or unconfirmed boundary')
            coverage = 100.0 * free / total
            if coverage < self.target:
                raise ValueError('coverage below target')
        except (KeyError, ValueError, TypeError, OverflowError) as exc:
            self.reset()
            self.rejection_reason = str(exc)
            return False

        # Reset on lost heartbeats, detector restart, or missed update sequence.
        if (self.last_status is not None and now - self.last_status > 5.0
                or self.last_update is not None and
                (updates < self.last_update or updates > self.last_update + 1)):
            self.reset()
        obs = now - age
        if self.last_update != updates:
            if (self.last_obs is not None and
                    (obs <= self.last_obs or obs - self.last_obs > self.max_age)):
                self.reset()
            if self.first is None:
                self.first = obs
            self.last_obs = obs
            self.count += 1
            self.last_update = updates
        self.last_status = now
        self.coverage = coverage
        self.rejection_reason = "none"
        return True

    def fresh(self, now):
        if (self.last_status is None or self.last_obs is None
                or now - self.last_status > 5.0
                or now - self.last_obs > self.max_age):
            self.reset()
            return False
        return True

    def ready(self, now):
        return (self.fresh(now) and self.count >= 3
                and self.last_obs - self.first >= self.hold)


def frontier_score(gain, distance, heading_error, turn_cost):
    # A half turn adds at most turn_cost metres of equivalent path cost.
    angle = abs(math.atan2(math.sin(heading_error), math.cos(heading_error)))
    return gain / (1.0 + distance + turn_cost * angle / math.pi)


class FailureHistory:
    """Spatial cooldown; at most one retry of a failed neighbourhood."""
    def __init__(self, radius=0.80, cooldown=20.0, max_failures=2):
        self.radius, self.cooldown, self.max_failures = radius, cooldown, max_failures
        self.records = []

    def nearby(self, x, y):
        return [r for r in self.records
                if math.hypot(x-r['x'], y-r['y']) < self.radius]

    def eligibility(self, x, y, now, revision):
        records = self.nearby(x, y)
        if any(r['count'] >= self.max_failures or now < r['until']
               or revision <= r['revision'] for r in records):
            return None
        return 1 if records else 0

    def record(self, x, y, now, revision):
        records = self.nearby(x, y)
        if not records:
            r = dict(x=x, y=y, count=0, until=0.0, revision=revision)
            self.records.append(r)
            records = [r]
        # Update all matching neighbourhoods to avoid bypassing overlap.
        for r in records:
            r['count'] += 1
            r['until'] = now + self.cooldown
            r['revision'] = revision
        return max(r['count'] for r in records)


class MissionStateMachine:
    """Supervisory FSM: one phase, derived from the existing action/stop facts.

    The state is reconciled after callbacks, and the same decision controls
    whether a new frontier can be planned. No second action client is used.
    """
    TERMINAL = frozenset(('COMPLETED', 'COMPLETION_TEST_PASSED',
                          'INCOMPLETE', 'STOP_UNVERIFIED', 'USER_STOPPED'))
    PLAN_STATES = frozenset(('PLANNING', 'WAITING_TF', 'WAITING_FRONTIER'))

    def __init__(self):
        self.state = None
        self.transitions = 0
        self.since = None

    @staticmethod
    def classify(n, now):
        if n.finished:
            if n.exit_code == 3:
                return 'STOP_UNVERIFIED'
            if n.exit_code == 130:
                return 'USER_STOPPED'
            if n.exit_code != 0:
                return 'INCOMPLETE'
            return 'COMPLETION_TEST_PASSED' if n.completion_only else 'COMPLETED'
        if n.shutdown_requested:
            return 'USER_STOPPING'
        if n.completion_requested:
            if n.terminal_at is not None:
                return 'VERIFYING_STOP'
            return 'CANCELING_FOR_COMPLETION'
        if n.cancel_reason is not None and n.goal_handle is not None:
            return 'CANCELING_FAILED_GOAL'
        if n.pending_goal_since is not None:
            return 'WAITING_GOAL_ACCEPTANCE'
        if n.goal_handle is not None:
            return 'NAVIGATING'
        if n.completion_only:
            return 'CONFIRMING_COMPLETION' if n.gate.count else 'WAITING_COMPLETION_EVIDENCE'
        if n.selected:
            return 'PREVIEW' if not n.execute else 'WAITING_GOAL_ACCEPTANCE'
        if n.execute and n.gate.last_status is not None and n.gate.last_obs is not None:
            if now - n.gate.last_status <= 5.0 and now - n.gate.last_obs <= n.gate.max_age:
                return 'CONFIRMING_COMPLETION'
        if n.limit_since is not None:
            return 'WAITING_AT_GOAL_LIMIT'
        if n.grid is None:
            return 'WAITING_MAP'
        if now < n.next_plan_time:
            return 'RECOVERING' if n.last_target_failed else 'REPLANNING'
        if n.waiting_tf:
            return 'WAITING_TF'
        if n.no_frontier_since is not None:
            return 'WAITING_FRONTIER'
        return 'PLANNING'

    def can_plan(self, n, now):
        return self.state not in self.TERMINAL and self.classify(n, now) in self.PLAN_STATES

    def reconcile(self, n, now):
        # Terminal outcomes are latched for the life of this mission.
        next_state = self.state if self.state in self.TERMINAL else self.classify(n, now)
        if next_state == self.state:
            return None
        old = self.state or 'INIT'
        self.state = next_state
        self.since = now
        self.transitions += 1
        return old, next_state


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        # 默认只预览目标，不控制机器人
        self.declare_parameter('execute', False)
        self.declare_parameter('max_goals', 6)
        self.execute = self.get_parameter('execute').value
        self.max_goals = max(
            1, int(self.get_parameter('max_goals').value))

        self.declare_parameter('target_coverage', 95.0)
        self.declare_parameter('completion_hold_sec', 10.0)
        self.declare_parameter('max_map_age_sec', 12.0)
        self.declare_parameter('no_frontier_grace_sec', 45.0)
        self.declare_parameter('completion_only', False)
        self.declare_parameter('turn_cost_m', 0.25)
        self.declare_parameter('failure_cooldown_sec', 20.0)
        self.declare_parameter('retry_failed_targets', True)
        self.turn_cost = float(self.get_parameter('turn_cost_m').value)
        cooldown = float(self.get_parameter('failure_cooldown_sec').value)
        retry = bool(self.get_parameter('retry_failed_targets').value)
        if not (math.isfinite(self.turn_cost) and 0 <= self.turn_cost <= 2.0
                and math.isfinite(cooldown) and 0 <= cooldown < 30.0):
            raise ValueError('Invalid optimization/recovery parameters')
        self.failure_history = FailureHistory(cooldown=cooldown,
                                              max_failures=2 if retry else 1)
        self.map_revision = 0
        self.map_stamp = None
        self.pending_goal_since = None
        self.planned_distance = 0.0
        self.retry_dispatches = 0
        self.last_choice_retry = False
        self.last_choice_score = 0.0
        self.last_choice_turn = 0.0
        self.target_coverage = float(self.get_parameter('target_coverage').value)
        self.completion_hold = float(self.get_parameter('completion_hold_sec').value)
        self.max_map_age = float(self.get_parameter('max_map_age_sec').value)
        self.no_frontier_grace = float(self.get_parameter('no_frontier_grace_sec').value)
        self.completion_only = bool(self.get_parameter('completion_only').value)
        if not (0 < self.target_coverage <= 100 and self.completion_hold >= 10
                and self.max_map_age >= 6 and self.no_frontier_grace >= 30):
            raise ValueError('Invalid completion parameters')
        if self.completion_only and self.execute:
            raise ValueError('completion_only requires execute:=false')
        self.gate = CompletionGate(self.target_coverage, self.completion_hold,
                                   self.max_map_age)
        self.completion_requested = False
        self.completion_started = None
        self.terminal_at = None
        self.no_frontier_since = None
        self.limit_since = None
        self.last_status_stamp = None
        self.last_status_log = 0.0
        self.odom_still_since = None
        self.odom_last = None
        self.odom_samples = 0
        self.odom_last_stamp = None
        self.started_at = time.monotonic()
        self.exit_code = 0
        self.mission_fsm = MissionStateMachine()
        self.waiting_tf = False
        self.last_target_failed = False
        self.last_mission_pub = None

        self.grid = None
        self.selected = False
        self.goal_handle = None
        self.finished = False
        self.last_feedback = 0.0
        self.goal_started_at = 0.0
        self.last_progress_at = 0.0
        self.best_remaining = float('inf')
        self.recoveries = 0
        self.cancel_reason = None
        self.cancel_started_at = 0.0
        self.shutdown_requested = False
        self.marker = None

        self.attempts = 0
        self.success_count = 0
        self.visited_targets = []
        self.active_target = None
        self.next_plan_time = 0.0
        self.no_candidate_count = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            OccupancyGrid, '/map', self.on_map,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(
            DiagnosticArray, '/exploration_boundary/status',
            self.on_boundary_status, qos)
        self.create_subscription(
            Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_timer(1.0, self.check_completion)

        self.marker_pub = self.create_publisher(
            Marker, '/exploration_target', qos)
        self.mission_pub = self.create_publisher(
            DiagnosticArray, '/exploration_mission/status', qos)

        self.client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose')

        self.create_timer(2.0, self.tick)

        self.get_logger().info(
            f'Frontier explorer started, execute={self.execute}')

    def update_mission_state(self, force=False):
        now = time.monotonic()
        change = self.mission_fsm.reconcile(self, now)
        if change:
            self.get_logger().info(f'MISSION_STATE: {change[0]} -> {change[1]}')
        if (not force and not change and self.last_mission_pub is not None
                and now - self.last_mission_pub < 1.0):
            return
        self.last_mission_pub = now
        try:
            msg = DiagnosticArray()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            st = DiagnosticStatus()
            state = self.mission_fsm.state
            st.name = 'frontier_explorer'
            st.hardware_id = 'autonomous_exploration'
            st.message = state
            st.level = (DiagnosticStatus.ERROR if state == 'STOP_UNVERIFIED'
                        else DiagnosticStatus.WARN if state in
                        ('INCOMPLETE', 'USER_STOPPED', 'USER_STOPPING')
                        else DiagnosticStatus.OK)
            values = {
                'state': state,
                'state_transitions': self.mission_fsm.transitions,
                'state_duration_sec': f'{now-self.mission_fsm.since:.2f}',
                'mode': 'completion_only' if self.completion_only else
                        ('execute' if self.execute else 'preview'),
                'coverage_valid_for_completion': str(self.gate.last_obs is not None
                    and self.gate.last_status is not None
                    and now-self.gate.last_obs <= self.gate.max_age
                    and now-self.gate.last_status <= 5.0).lower(),
                'qualified_coverage': ('unavailable' if self.gate.coverage is None
                                       else f'{self.gate.coverage:.4f}'),
                'target_coverage': self.target_coverage,
                'completion_observations': self.gate.count,
                'active_owned_goal': str(self.goal_handle is not None).lower(),
                'pending_goal': str(self.pending_goal_since is not None).lower(),
                'cancel_reason': self.cancel_reason or 'none',
                'goals_succeeded': self.success_count,
                'attempts': self.attempts,
                'retry_dispatches': self.retry_dispatches,
                'map_revision': self.map_revision,
                'terminal': str(state in self.mission_fsm.TERMINAL).lower(),
                'exit_code': self.exit_code if self.finished else 'pending',
            }
            st.values = [KeyValue(key=k, value=str(v)) for k, v in values.items()]
            msg.status = [st]
            self.mission_pub.publish(msg)
        except Exception as exc:
            # Telemetry errors must not interrupt navigation or cancellation.
            self.get_logger().warn(f'MISSION_TELEMETRY_ERROR: {exc}')

    def on_map(self, msg):
        self.grid = msg
        stamp = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        if stamp > 0 and stamp != self.map_stamp:
            self.map_stamp = stamp
            self.map_revision += 1

    def on_boundary_status(self, msg):
        if not (self.execute or self.completion_only) or self.finished:
            return
        now = time.monotonic()
        stamp = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        ros_now = self.get_clock().now().nanoseconds
        # A retained message is not automatically a fresh observation.
        if (stamp <= 0 or not -0.5 <= (ros_now - stamp) / 1e9 <= 4.0
                or msg.header.frame_id != 'map'):
            self.gate.reset()
            return
        if self.last_status_stamp is not None:
            if stamp < self.last_status_stamp:
                self.gate.reset()
            elif stamp == self.last_status_stamp:
                return
        self.last_status_stamp = stamp
        statuses = [st for st in msg.status if st.name == 'exploration_boundary_detector']
        if len(statuses) != 1:
            self.gate.reset()
            return
        st = statuses[0]
        values = {kv.key: kv.value for kv in st.values}
        # Do not credit maps processed before this explorer started.
        try:
            if now - float(values['map_age_sec']) < self.started_at:
                self.gate.reset()
                return
        except (KeyError, ValueError, TypeError):
            self.gate.reset()
            return
        self.gate.observe(values, st.level, now)
        if now - self.last_status_log >= 5.0:
            self.last_status_log = now
            self.get_logger().info(
                f'Boundary: {values.get("state", "unknown")}, '
                f'coverage={values.get("coverage", "unknown")}%, '
                f'map_age={values.get("map_age_sec", "unknown")}s, '
                f'completion_observations={self.gate.count}, '
                f'diagnostic_level={st.level!r}, '
                f'rejection_reason={self.gate.rejection_reason}')

    def on_odom(self, msg):
        if self.terminal_at is None or self.finished:
            return
        now = time.monotonic()
        stamp = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        if self.odom_last_stamp is not None and stamp <= self.odom_last_stamp:
            return
        self.odom_last_stamp = stamp
        v, w = msg.twist.twist.linear, msg.twist.twist.angular
        if (stamp <= 0 or not -0.5 <= age <= 2.0
                or not all(math.isfinite(x) for x in (v.x, v.y, v.z, w.x, w.y, w.z))
                or math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z) > 0.02
                or math.sqrt(w.x*w.x + w.y*w.y + w.z*w.z) > 0.05):
            self.odom_still_since = None
            self.odom_samples = 0
            self.odom_last = None
            return
        if self.odom_last is None or now - self.odom_last > 2.0:
            self.odom_still_since = now
            self.odom_samples = 0
        self.odom_last = now
        self.odom_samples += 1

    def finish_incomplete(self, reason):
        self.exit_code = 2
        self.finished = True
        self.get_logger().warn(f'EXPLORATION_INCOMPLETE: {reason}')

    def finish_error(self, reason):
        self.exit_code = 3
        self.finished = True
        self.get_logger().error(f'EXPLORATION_STOP_UNVERIFIED: {reason}')

    def begin_stop_verification(self):
        self.terminal_at = time.monotonic()
        self.odom_last = None
        self.odom_still_since = None
        self.odom_samples = 0
        self.odom_last_stamp = None
        self.selected = False
        self.active_target = None
        if self.marker is not None:
            self.marker.action = Marker.DELETE
            self.marker_pub.publish(self.marker)
            self.marker = None
        self.get_logger().info('No active owned Nav2 goal; verifying odometry stop')

    def check_completion(self):
        if self.finished or self.shutdown_requested or not (self.execute or self.completion_only):
            return
        now = time.monotonic()
        ready = self.gate.ready(now)
        if self.completion_requested:
            if self.terminal_at is None:
                if now - self.completion_started > 15.0:
                    self.finish_error('Nav2 terminal result not received; verify robot stop')
                return
            if (self.odom_still_since is not None and self.odom_last is not None
                    and self.odom_samples >= 3 and now - self.odom_last <= 2.0
                    and self.odom_last - self.odom_still_since >= 1.0):
                if not ready:
                    self.finish_incomplete('robot stopped but completion evidence no longer valid')
                    return
                label = ('COMPLETION_TEST_PASSED' if self.completion_only
                         else 'EXPLORATION_COMPLETED')
                self.get_logger().info(
                    f'{label}: coverage={self.gate.coverage:.4f}%, '
                    f'target={self.target_coverage:.1f}%, '
                    f'hold={self.gate.last_obs-self.gate.first:.2f}s, '
                    f'observations={self.gate.count}, '
                    f'goals={self.success_count}/{self.attempts}, '
                    f'odom_stop_verified=true, completion_only={self.completion_only}')
                self.finished = True
            elif now - self.terminal_at > 12.0:
                self.finish_error('stationary odometry not verified within 12s')
            return
        if ready:
            self.completion_requested = True
            self.completion_started = now
            self.get_logger().info('COMPLETION_CONDITION_MET: stopping navigation')
            if self.goal_handle is not None:
                self.request_cancel('COVERAGE_REACHED')
            elif not self.selected or self.completion_only:
                self.begin_stop_verification()
            # Otherwise goal acceptance is pending; on_accepted handles cancellation.
            return
        if self.completion_only and now - self.started_at > 60.0:
            self.finish_incomplete('stationary completion test timed out')
        elif self.limit_since is not None and now - self.limit_since > self.no_frontier_grace:
            self.finish_incomplete('goal limit reached without verified completion')
        elif self.no_frontier_since is not None and now - self.no_frontier_since > self.no_frontier_grace:
            self.finish_incomplete('no reachable frontier and completion not verified')

    def tick(self):
        if self.finished or self.shutdown_requested or self.completion_only:
            return
        if self.completion_requested:
            return
        self.check_navigation()


        if self.marker is not None:
            self.marker.header.stamp = self.get_clock().now().to_msg()
            self.marker_pub.publish(self.marker)

        if (self.finished or self.selected or self.limit_since is not None or
                self.grid is None or
                time.monotonic() < self.next_plan_time):
            return

        # Hold new dispatch while the coverage confirmation window is accumulating.
        if self.execute and self.gate.fresh(time.monotonic()):
            return

        # All new target planning must be permitted by the mission FSM.
        if not self.mission_fsm.can_plan(self, time.monotonic()):
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', Time())
        except Exception:
            self.waiting_tf = True
            self.get_logger().warn(
                'Waiting for map -> base_footprint TF')
            return

        self.waiting_tf = False
        rx = tf.transform.translation.x
        ry = tf.transform.translation.y

        q = tf.transform.rotation
        robot_yaw = math.atan2(2*(q.w*q.z + q.x*q.y),
                               1-2*(q.y*q.y + q.z*q.z))
        choice = self.find_goal(self.grid, rx, ry, robot_yaw)

        if choice is None:
            self.no_candidate_count += 1
            if self.no_frontier_since is None:
                self.no_frontier_since = time.monotonic()
            self.get_logger().warn(
                f'No reachable candidate, retry {self.no_candidate_count}; '
                f'waiting for boundary/coverage, '
                f'{time.monotonic()-self.no_frontier_since:.1f}/{self.no_frontier_grace:.1f}s')
            if not self.execute and self.no_candidate_count >= 6:
                self.finish_incomplete('no selectable frontier in preview mode')
            return

        x, y, yaw, gain, dist, clusters = choice
        self.selected = True
        self.no_candidate_count = 0
        self.no_frontier_since = None
        self.active_target = (x, y)

        self.get_logger().info(
            f'Frontier regions={clusters}; '
            f'selected target=({x:.2f}, {y:.2f}), '
            f'path distance={dist:.2f}m, '
            f'information gain={gain}, score={self.last_choice_score:.2f}, '
            f'initial_turn={math.degrees(self.last_choice_turn):.1f}deg, '
            f'retry={self.last_choice_retry}')

        self.publish_target(x, y)

        if self.execute:
            if self.send_goal(x, y, yaw):
                self.planned_distance += dist
                self.retry_dispatches += int(self.last_choice_retry)
        else:
            self.get_logger().info(
                'PREVIEW ONLY: robot will not move. '
                'Set execute:=true to navigate.')

    def find_goal(self, msg, rx, ry, robot_yaw=0.0):
        w = msg.info.width
        h = msg.info.height
        n = w * h
        data = msg.data
        res = msg.info.resolution

        if res <= 0 or len(data) != n:
            return None

        origin = msg.info.origin.position
        q = msg.info.origin.orientation
        yaw = math.atan2(
            2 * (q.w*q.z + q.x*q.y),
            1 - 2 * (q.y*q.y + q.z*q.z))

        co, si = math.cos(yaw), math.sin(yaw)

        def to_grid(x, y):
            dx, dy = x - origin.x, y - origin.y
            gx = (co*dx + si*dy) / res
            gy = (-si*dx + co*dy) / res
            return math.floor(gx), math.floor(gy)

        def to_world(i):
            gx = (i % w + 0.5) * res
            gy = (i // w + 0.5) * res
            return (
                origin.x + co*gx - si*gy,
                origin.y + si*gx + co*gy
            )

        def neighbors4(i):
            x, y = i % w, i // w
            if x > 0:
                yield i - 1
            if x + 1 < w:
                yield i + 1
            if y > 0:
                yield i - w
            if y + 1 < h:
                yield i + w

        # A. 障碍物安全膨胀：给机器人预留约20cm空间
        unsafe = bytearray(n)
        radius = max(1, math.ceil(0.20 / res))
        offsets = [
            (dx, dy)
            for dy in range(-radius, radius + 1)
            for dx in range(-radius, radius + 1)
            if dx*dx + dy*dy <= radius*radius
        ]

        for i, value in enumerate(data):
            if value < 65:
                continue

            x, y = i % w, i // w

            for dx, dy in offsets:
                nx, ny = x + dx, y + dy
                if 0 <= nx < w and 0 <= ny < h:
                    unsafe[ny*w + nx] = 1

        free = bytearray(
            1 if 0 <= value <= 20 and not unsafe[i] else 0
            for i, value in enumerate(data)
        )

        # B. 根据 TF 找到机器人所在栅格
        sx, sy = to_grid(rx, ry)
        if not (0 <= sx < w and 0 <= sy < h):
            return None

        start = sy*w + sx

        # 机器人中心若落在离散地图空洞中，寻找附近空闲格
        if not free[start]:
            nearby = [
                i for i in range(n)
                if free[i]
                and abs(i % w - sx) <= 6
                and abs(i // w - sy) <= 6
            ]
            if not nearby:
                return None

            start = min(
                nearby,
                key=lambda i:
                    (i % w - sx)**2 + (i // w - sy)**2
            )

        # C. BFS 搜索机器人实际可达的已知空闲区域
        steps = [-1] * n
        parent = [-1] * n
        steps[start] = 0
        parent[start] = start

        queue = deque([start])

        while queue:
            i = queue.popleft()
            for j in neighbors4(i):
                if free[j] and steps[j] < 0:
                    steps[j] = steps[i] + 1
                    parent[j] = i
                    queue.append(j)

        # D. 找到可达的 Frontier
        frontiers = set()

        for i in range(n):
            if steps[i] < 0:
                continue
            if any(data[j] == -1 for j in neighbors4(i)):
                frontiers.add(i)

        # E. 八邻域聚类
        visited = set()
        clusters = []

        for start_i in sorted(frontiers):
            if start_i in visited:
                continue

            component = []
            queue = deque([start_i])
            visited.add(start_i)

            while queue:
                i = queue.popleft()
                component.append(i)
                x, y = i % w, i // w

                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        nx, ny = x + dx, y + dy
                        if not (0 <= nx < w and 0 <= ny < h):
                            continue

                        j = ny*w + nx
                        if j in frontiers and j not in visited:
                            visited.add(j)
                            queue.append(j)

            if len(component) >= 6:
                clusters.append(component)

        # F. 评分：探索收益与距离成本
        best = None
        best_key = None
        now = time.monotonic()
        self.last_choice_retry = False
        self.last_choice_score = 0.0
        self.last_choice_turn = 0.0

        for component in clusters:
            stride = max(1, len(component) // 80)

            for frontier in component[::stride]:
                # 从 Frontier 沿已知路径向内退三个栅格
                goal_i = frontier

                for _ in range(3):
                    if parent[goal_i] != goal_i:
                        goal_i = parent[goal_i]

                distance = steps[goal_i] * res

                if not 0.7 <= distance <= 12.0:
                    continue

                # 估计 Frontier 附近的未知栅格数量
                fx, fy = frontier % w, frontier // w
                gain = 0

                for dy in range(-7, 8):
                    for dx in range(-7, 8):
                        nx, ny = fx + dx, fy + dy

                        if not (0 <= nx < w and 0 <= ny < h):
                            continue
                        if dx*dx + dy*dy > 49:
                            continue
                        if data[ny*w + nx] == -1:
                            gain += 1

                x, y = to_world(goal_i)
                if any(math.hypot(x-px, y-py) < 0.65
                       for px, py in self.visited_targets):
                    continue
                retry = self.failure_history.eligibility(x, y, now, self.map_revision)
                if retry is None:
                    continue

                # Look along the safe BFS path near its start, not straight through walls.
                look = goal_i
                look_steps = max(1, math.ceil(0.30 / res))
                while steps[look] > look_steps:
                    look = parent[look]
                lx, ly = to_world(look)
                sx_world, sy_world = to_world(start)
                initial_heading = math.atan2(ly-sy_world, lx-sx_world)
                turn = abs(math.atan2(math.sin(initial_heading-robot_yaw),
                                      math.cos(initial_heading-robot_yaw)))
                score = frontier_score(gain, distance, turn, self.turn_cost)
                # Explore a new candidate before retrying a cooled-down failure.
                key = (-retry, score, -distance, -goal_i, -frontier)
                if best_key is None or key > best_key:
                    fx_world, fy_world = to_world(frontier)
                    heading = math.atan2(fy_world-y, fx_world-x)
                    best_key = key
                    self.last_choice_retry = bool(retry)
                    self.last_choice_score = score
                    self.last_choice_turn = turn
                    best = (x, y, heading, gain, distance, len(clusters))

        return best

    def publish_target(self, x, y):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.ns = 'exploration_target'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.15
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.28
        marker.scale.y = 0.28
        marker.scale.z = 0.28
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 1.0
        marker.color.a = 1.0

        self.marker = marker
        self.marker_pub.publish(marker)

    def send_goal(self, x, y, yaw):
        if not self.client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error(
                'Nav2 action server unavailable')
            self.complete_target(False, 'SERVER_UNAVAILABLE')
            return False

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw/2)
        goal.pose.pose.orientation.w = math.cos(yaw/2)

        self.get_logger().info(
            f'Sending exploration goal ({x:.2f}, {y:.2f})')

        self.pending_goal_since = time.monotonic()
        try:
            future = self.client.send_goal_async(goal, feedback_callback=self.on_feedback)
            future.add_done_callback(self.on_accepted)
        except Exception as exc:
            self.finish_error(f'goal dispatch state unknown: {exc}')
            return False
        return True

    def on_accepted(self, future):
        self.pending_goal_since = None
        try:
            handle = future.result()
        except Exception as exc:
            self.finish_error(f'goal acceptance unknown: {exc}')
            return

        if not handle.accepted:
            if self.completion_requested:
                self.begin_stop_verification()
                return
            self.get_logger().error('Exploration goal rejected')
            self.complete_target(False, 'REJECTED')
            return

        self.goal_handle = handle
        self.goal_started_at = time.monotonic()
        self.last_progress_at = self.goal_started_at
        self.best_remaining = float('inf')
        self.recoveries = 0
        self.cancel_reason = None
        self.get_logger().info('Exploration goal accepted')

        if self.shutdown_requested:
            self.request_cancel('USER_STOP')
        elif self.completion_requested:
            self.request_cancel('COVERAGE_REACHED')

        result_future = handle.get_result_async()
        result_future.add_done_callback(self.on_result)

    def on_feedback(self, msg):
        now = time.monotonic()
        f = msg.feedback
        remaining = float(f.distance_remaining)

        self.recoveries = max(
            self.recoveries, int(f.number_of_recoveries))

        # Nav2 刚接收目标时可能暂时报告0米，忽略该值
        if math.isfinite(remaining) and remaining > 0.05:
            if (self.best_remaining == float('inf') or
                    remaining < self.best_remaining - 0.20):
                self.best_remaining = remaining
                self.last_progress_at = now

        if now - self.last_feedback >= 2.0:
            self.last_feedback = now
            self.get_logger().info(
                f'Remaining={remaining:.2f}m, '
                f'recoveries={self.recoveries}')

    def request_cancel(self, reason):
        if self.goal_handle is None or self.cancel_reason is not None:
            return

        self.cancel_reason = reason
        self.cancel_started_at = time.monotonic()

        self.get_logger().warn(
            f'Canceling navigation: {reason}')

        try:
            future = self.goal_handle.cancel_goal_async()
            future.add_done_callback(self.on_cancel_response)
        except Exception as exc:
            self.get_logger().error(f'Cancel request failed: {exc}')

    def on_cancel_response(self, future):
        try:
            response = future.result()
            self.get_logger().info(
                f'Cancel response: return_code={response.return_code}, '
                f'goals_canceling={len(response.goals_canceling)}; awaiting terminal result')
        except Exception as exc:
            self.get_logger().error(f'Cancel response failed: {exc}; awaiting terminal result')

    def check_navigation(self):
        if self.goal_handle is None:
            if (self.pending_goal_since is not None and
                    time.monotonic()-self.pending_goal_since > 12.0):
                self.finish_error('goal acceptance timeout; verify Nav2 before restarting')
            return

        now = time.monotonic()

        if self.cancel_reason is not None:
            if now - self.cancel_started_at > 12.0:
                self.get_logger().error(
                    'Cancellation not confirmed within 12s. '
                    'Stop exploration and verify Nav2 goal state.')
                self.finish_error('cancellation timeout; verify Nav2 goal state')
            return

        if self.recoveries >= 3:
            self.request_cancel('TOO_MANY_RECOVERIES')
        elif now - self.goal_started_at > 90.0:
            self.request_cancel('GOAL_TIMEOUT')
        elif now - self.last_progress_at > 25.0:
            self.request_cancel('NO_PROGRESS')

    def on_result(self, future):
        try:
            result = future.result()
        except Exception as exc:
            self.finish_error(f'Nav2 result unavailable: {exc}')
            return
        if result.status not in (GoalStatus.STATUS_SUCCEEDED,
                                 GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED):
            self.finish_error(f'Unexpected Nav2 result status: {result.status}')
            return
        if self.completion_requested and not self.shutdown_requested:
            self.get_logger().info(f'Nav2 terminal result: status={result.status}; '
                                   'completion stop (not blacklisted)')
            self.attempts += 1
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                self.success_count += 1
            self.goal_handle = None
            self.cancel_reason = None
            self.begin_stop_verification()
            return
        succeeded = (
            result.status == GoalStatus.STATUS_SUCCEEDED
            and self.cancel_reason is None
        )

        if self.shutdown_requested:
            self.get_logger().info('Navigation stopped')
            self.goal_handle = None
            self.exit_code = 130
            self.finished = True
            return

        reason = self.cancel_reason
        self.cancel_reason = None

        self.complete_target(
            succeeded,
            'SUCCEEDED' if succeeded else
            (reason or f'STATUS_{result.status}')
        )

    def complete_target(self, succeeded, status):
        self.last_target_failed = not succeeded
        self.attempts += 1

        if succeeded:
            self.success_count += 1

        self.get_logger().info(
            f'Goal {self.attempts}: {status}, '
            f'success={self.success_count}/{self.attempts}'
        )

        if self.active_target is not None:
            if succeeded:
                self.visited_targets.append(
                    self.active_target
                )
            elif status != 'SERVER_UNAVAILABLE':
                failures = self.failure_history.record(
                    *self.active_target, time.monotonic(), self.map_revision)
                locked = failures >= self.failure_history.max_failures
                self.get_logger().warn(
                    f'FAILED_REGION: target={self.active_target}, failures={failures}, '
                    f'locked={locked}, cooldown={self.failure_history.cooldown:.1f}s, '
                    'retry requires fresh map and no preferred new target')

        self.active_target = None
        self.goal_handle = None
        self.selected = False

        if self.attempts >= self.max_goals:
            self.limit_since = time.monotonic()
            self.get_logger().warn(
                f'Goal safety limit reached ({self.success_count}/{self.attempts}); '
                'waiting for completion evidence without dispatching more goals')
        else:
            # 等待 SLAM 接收新位置的雷达数据
            self.next_plan_time = time.monotonic() + (5.0 if status == 'SERVER_UNAVAILABLE' else 3.0)
            self.get_logger().info(
                'Waiting for map update, then replanning...'
            )


def main():
    stop = {'requested': False}

    def handle_signal(_signum, _frame):
        stop['requested'] = True

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    node = FrontierExplorer()
    stop_deadline = None

    try:
        node.update_mission_state(force=True)
        while rclpy.ok() and not node.finished:
            if stop['requested']:
                if not node.shutdown_requested:
                    node.shutdown_requested = True
                    stop_deadline = time.monotonic() + 12.0
                    node.get_logger().warn(
                        'Shutdown requested; canceling active goal')
                    node.update_mission_state(force=True)

                if node.goal_handle is not None:
                    node.request_cancel('USER_STOP')
                elif not node.selected or not node.execute:
                    node.exit_code = 130
                    node.finished = True
                    break

                if time.monotonic() > stop_deadline:
                    node.finish_error('Shutdown timeout; verify Nav2 is stopped')
                    break

            rclpy.spin_once(node, timeout_sec=0.2)
            node.update_mission_state()

    finally:
        if not node.finished:
            node.exit_code = 3
            node.finished = True
            node.get_logger().error('Mission interrupted without verified terminal outcome')
        if rclpy.ok():
            node.update_mission_state(force=True)
        node.get_logger().info(
            f'MISSION_METRICS: elapsed_wall_sec={time.monotonic()-node.started_at:.2f}, '
            f'goals_succeeded={node.success_count}, attempts={node.attempts}, '
            f'dispatched_bfs_distance_m={node.planned_distance:.2f}, '
            f'retry_dispatches={node.retry_dispatches}, exit_code={node.exit_code}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return node.exit_code


if __name__ == '__main__':
    raise SystemExit(main())
