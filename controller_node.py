# controller_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
import math
import csv
import os
import time

from .path_smoothing import smooth_path
from .trajectory_generator import generate_time_parameterized_trajectory

# DEFAULT WAYPOINTS: edit or load from config/waypoints.yaml if you prefer
WAYPOINTS = [
    (0.0, 0.0),
    (1.0, 0.0),
    (2.0, 1.0),
    (2.0, 2.0),
    (1.0, 2.5),
    (0.0, 2.0)
]


class TrajectoryController(Node):
    def __init__(self):
        super().__init__('trajectory_controller_safe')

        # Publishers and subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 5)
        self.waypoint_marker_pub = self.create_publisher(Marker, '/waypoints_marker', 5)
        self.traj_marker_pub = self.create_publisher(Marker, '/trajectory_marker', 5)
        self.target_marker_pub = self.create_publisher(Marker, '/target_point', 5)

        self.create_subscription(Odometry, '/odom', self.odom_cb, 20)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 20)

        # Build trajectory (do smoothing + time-parameterization here)
        self.get_logger().info('Smoothing waypoints...')
        self.smooth = smooth_path(WAYPOINTS, samples=400)
        self.get_logger().info(f'Smoothed to {len(self.smooth)} pts.')

        self.traj = generate_time_parameterized_trajectory(self.smooth, v_max=0.18, accel=0.6)
        self.get_logger().info(f'Generated trajectory with {len(self.traj)} timed points.')

        # Publish visuals immediately and periodically (makes RViz see them)
        self.publish_visuals()
        self.visual_timer = self.create_timer(1.0, self.publish_visuals)  # 1 Hz refresh

        # controller state
        self.current_pose = None
        self.current_yaw = 0.0
        self.traj_index = 0
        self.lookahead = 0.30   # meters (tunable)
        self.goal_tolerance = 0.12
        self.max_linear = 0.18
        self.min_linear = 0.02

        # Obstacle avoidance
        self.scan_ranges = None
        self.safe_dist = 0.28     # if anything closer than this, strong avoidance/stop
        self.influence_dist = 1.0 # obstacles within this influence avoidance
        self.repulsive_gain = 0.8
        self.goal_gain = 1.2

        # Logging executed path
        self.executed = []
        self.log_path_file = os.path.join(os.getcwd(), f'executed_path_{int(time.time())}.csv')

        # Control timer (fast enough to drive)
        self.control_timer = self.create_timer(0.1, self.control_step)  # 10 Hz

        self.get_logger().info('Trajectory controller initialized.')

    # ----------------- Visuals -----------------
    def publish_visuals(self):
        """Publish path and waypoint visuals (called periodically so RViz always shows them)."""
        now = self.get_clock().now().to_msg()
        frame = 'odom'

        # 1) Publish nav_msgs/Path (smoothed, time-parameterized trajectory)
        path_msg = Path()
        path_msg.header.frame_id = frame
        path_msg.header.stamp = now
        for (x, y, *rest) in [(pt[0], pt[1],) for pt in self.smooth]:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = 0.0
            path_msg.poses.append(ps)
        self.path_pub.publish(path_msg)

        # 2) Publish thick LINE_STRIP for trajectory (very visible)
        traj_marker = Marker()
        traj_marker.header.frame_id = frame
        traj_marker.header.stamp = now
        traj_marker.ns = 'smoothed_trajectory'
        traj_marker.id = 0
        traj_marker.type = Marker.LINE_STRIP
        traj_marker.action = Marker.ADD
        traj_marker.pose.orientation.w = 1.0
        traj_marker.scale.x = 0.12   # thickness of the line
        traj_marker.color.a = 1.0
        traj_marker.color.r = 0.0
        traj_marker.color.g = 0.6
        traj_marker.color.b = 1.0

        traj_marker.points = []
        for (x, y, *_) in self.traj:
            p = Point()
            p.x = x; p.y = y; p.z = 0.02
            traj_marker.points.append(p)
        self.traj_marker_pub.publish(traj_marker)

        # 3) Publish large waypoint spheres (SPHERE_LIST) for original discrete waypoints
        wp_marker = Marker()
        wp_marker.header.frame_id = frame
        wp_marker.header.stamp = now
        wp_marker.ns = 'waypoints'
        wp_marker.id = 1
        wp_marker.type = Marker.SPHERE_LIST
        wp_marker.action = Marker.ADD
        wp_marker.pose.orientation.w = 1.0
        wp_marker.scale.x = 0.28   # big spheres
        wp_marker.scale.y = 0.28
        wp_marker.scale.z = 0.28
        wp_marker.color.a = 1.0
        wp_marker.color.r = 1.0
        wp_marker.color.g = 0.2
        wp_marker.color.b = 0.2

        wp_marker.points = []
        for (x, y) in WAYPOINTS:
            p = Point()
            p.x = x; p.y = y; p.z = 0.05
            wp_marker.points.append(p)
        self.waypoint_marker_pub.publish(wp_marker)

    def publish_target_marker(self, x, y):
        """Publish a single target sphere at the lookahead point (visible)."""
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'target'
        marker.id = 2
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.06
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.20
        marker.scale.y = 0.20
        marker.scale.z = 0.20
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.6
        marker.color.b = 0.0
        self.target_marker_pub.publish(marker)

    # ----------------- Callbacks -----------------
    def odom_cb(self, msg: Odometry):
        self.current_pose = msg.pose.pose.position
        self.current_yaw = self._quat_to_yaw(msg.pose.pose.orientation)
        # Logging executed path
        self.executed.append((self.current_pose.x, self.current_pose.y, self.get_clock().now().nanoseconds/1e9))

    def scan_cb(self, msg: LaserScan):
        self.scan_ranges = msg

    # ----------------- Helpers -----------------
    def _quat_to_yaw(self, q):
        siny = 2.0*(q.w*q.z + q.x*q.y)
        cosy = 1.0 - 2.0*(q.y*q.y + q.z*q.z)
        return math.atan2(siny, cosy)

    def _distance(self, a, b):
        return math.hypot(a[0]-b[0], a[1]-b[1])

    def _wrap(self, a):
        while a > math.pi:
            a -= 2*math.pi
        while a < -math.pi:
            a += 2*math.pi
        return a

    # ----------------- Main control -----------------
    def control_step(self):
        # Called at 10 Hz by control_timer
        if self.current_pose is None or len(self.traj) == 0:
            return

        pos = (self.current_pose.x, self.current_pose.y)
        goal = (self.traj[-1][0], self.traj[-1][1])
        dist_to_goal = self._distance(pos, goal)
        if dist_to_goal < self.goal_tolerance:
            self._publish_stop()
            self.get_logger().info('Goal reached. Stopping.')
            self._save_executed_path()
            return

        # get lookahead index and point
        target_index = self._find_lookahead_index(pos, lookahead=self.lookahead)
        tx, ty, tstamp, desired_v = self.traj[min(target_index, len(self.traj)-1)]

        # publish target marker so you can see where robot is aiming
        self.publish_target_marker(tx, ty)

        # compute heading to target
        angle_to_goal = math.atan2(ty - pos[1], tx - pos[0])

        # compute avoidance (repulsive) heading from LaserScan if available
        repulsive_angle, close_dist = self._compute_repulsive_vector()

        # combine headings: weighted vector sum
        gx_x = math.cos(angle_to_goal) * self.goal_gain
        gx_y = math.sin(angle_to_goal) * self.goal_gain
        if repulsive_angle is not None:
            rx_x = math.cos(repulsive_angle) * self.repulsive_gain
            rx_y = math.sin(repulsive_angle) * self.repulsive_gain
        else:
            rx_x = rx_y = 0.0

        combined_x = gx_x + rx_x
        combined_y = gx_y + rx_y
        combined_angle = math.atan2(combined_y, combined_x)
        combined_error = self._wrap(combined_angle - self.current_yaw)

        # compute linear velocity: reduce when obstacle close or large angular error
        ang_factor = max(0.0, 1.0 - min(1.0, abs(combined_error)/1.2))
        speed = max(self.min_linear, min(self.max_linear, desired_v * ang_factor))
        if close_dist is not None and close_dist < self.safe_dist:
            speed = 0.0
            self.get_logger().warn(f'Obstacle too close ({close_dist:.2f} m). Stopping.')

        # angular PD-like control
        k_ang = 1.8
        angular_cmd = k_ang * combined_error
        max_ang = 1.4
        angular_cmd = max(-max_ang, min(max_ang, angular_cmd))

        # Publish command
        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = angular_cmd
        self.cmd_pub.publish(cmd)

    def _publish_stop(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def _find_lookahead_index(self, pos, lookahead=0.3):
        # naive linear search from current index forward
        for i in range(self.traj_index, len(self.traj)):
            tx, ty, *_ = self.traj[i]
            if math.hypot(tx - pos[0], ty - pos[1]) >= lookahead:
                self.traj_index = i
                return i
        self.traj_index = len(self.traj)-1
        return self.traj_index

    def _compute_repulsive_vector(self):
        if self.scan_ranges is None:
            return None, None

        ranges = self.scan_ranges.ranges
        angle_min = self.scan_ranges.angle_min
        angle_inc = self.scan_ranges.angle_increment

        rx = 0.0; ry = 0.0
        min_dist = None
        for i, r in enumerate(ranges):
            if r == float('inf') or r <= 0.0:
                continue
            if r > self.influence_dist:
                continue
            theta = angle_min + i * angle_inc
            mag = (self.influence_dist - r) / (r + 1e-6)
            rx += (-math.cos(theta)) * mag
            ry += (-math.sin(theta)) * mag
            if min_dist is None or r < min_dist:
                min_dist = r

        norm = math.hypot(rx, ry)
        if norm < 1e-6:
            return None, min_dist

        rep_angle_robot = math.atan2(ry, rx)
        rep_angle_map = self.current_yaw + rep_angle_robot
        return rep_angle_map, min_dist

    def _save_executed_path(self):
        try:
            with open(self.log_path_file, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['x','y','t'])
                for r in self.executed:
                    w.writerow(r)
            self.get_logger().info(f'Saved executed path to {self.log_path_file}')
        except Exception as e:
            self.get_logger().error('Failed to save executed path: ' + str(e))


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt, shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

