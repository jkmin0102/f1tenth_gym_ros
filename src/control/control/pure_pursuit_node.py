#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from ackermann_msgs.msg import AckermannDriveStamped
from tf2_ros import Buffer, TransformListener

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


# ---------- (ported) planner helpers ----------
def nearest_point_on_trajectory(point: np.ndarray, trajectory: np.ndarray):
    diffs = trajectory[1:, :] - trajectory[:-1, :]
    l2s = diffs[:, 0] ** 2 + diffs[:, 1] ** 2

    dots = np.empty((trajectory.shape[0] - 1,))
    for i in range(dots.shape[0]):
        dots[i] = np.dot((point - trajectory[i, :]), diffs[i, :])

    l2s[l2s < 1e-12] = 1e-12
    t = dots / l2s
    t[t < 0.0] = 0.0
    t[t > 1.0] = 1.0

    projections = trajectory[:-1, :] + (t * diffs.T).T

    dists = np.empty((projections.shape[0],))
    for i in range(dists.shape[0]):
        temp = point - projections[i]
        dists[i] = math.sqrt(float(np.sum(temp * temp)))

    min_dist_segment = int(np.argmin(dists))
    return projections[min_dist_segment], dists[min_dist_segment], t[min_dist_segment], min_dist_segment


def first_point_on_trajectory_intersecting_circle(point: np.ndarray, radius: float,
                                                  trajectory: np.ndarray, t: float = 0.0, wrap: bool = False):
    start_i = int(t)
    start_t = t % 1.0
    first_t = None
    first_i = None
    first_p = None

    trajectory = np.ascontiguousarray(trajectory)
    for i in range(start_i, trajectory.shape[0] - 1):
        start = trajectory[i, :]
        end = trajectory[i + 1, :] + 1e-6
        V = np.ascontiguousarray(end - start)

        a = float(np.dot(V, V))
        b = 2.0 * float(np.dot(V, start - point))
        c = float(np.dot(start, start) + np.dot(point, point) - 2.0 * np.dot(start, point) - radius * radius)
        discriminant = b * b - 4 * a * c
        if discriminant < 0:
            continue

        discriminant = math.sqrt(discriminant)
        t1 = (-b - discriminant) / (2.0 * a)
        t2 = (-b + discriminant) / (2.0 * a)

        if i == start_i:
            if 0.0 <= t1 <= 1.0 and t1 >= start_t:
                first_t, first_i, first_p = t1, i, start + t1 * V
                break
            if 0.0 <= t2 <= 1.0 and t2 >= start_t:
                first_t, first_i, first_p = t2, i, start + t2 * V
                break
        else:
            if 0.0 <= t1 <= 1.0:
                first_t, first_i, first_p = t1, i, start + t1 * V
                break
            if 0.0 <= t2 <= 1.0:
                first_t, first_i, first_p = t2, i, start + t2 * V
                break

    if wrap and first_p is None:
        for i in range(-1, start_i):
            start = trajectory[i % trajectory.shape[0], :]
            end = trajectory[(i + 1) % trajectory.shape[0], :] + 1e-6
            V = end - start

            a = float(np.dot(V, V))
            b = 2.0 * float(np.dot(V, start - point))
            c = float(np.dot(start, start) + np.dot(point, point) - 2.0 * np.dot(start, point) - radius * radius)
            discriminant = b * b - 4 * a * c
            if discriminant < 0:
                continue

            discriminant = math.sqrt(discriminant)
            t1 = (-b - discriminant) / (2.0 * a)
            t2 = (-b + discriminant) / (2.0 * a)

            if 0.0 <= t1 <= 1.0:
                first_t, first_i, first_p = t1, i, start + t1 * V
                break
            if 0.0 <= t2 <= 1.0:
                first_t, first_i, first_p = t2, i, start + t2 * V
                break

    return first_p, first_i, first_t


def get_actuation(pose_theta: float, lookahead_point: np.ndarray,
                  position: np.ndarray, lookahead_distance: float, wheelbase: float):
    waypoint_y = float(np.dot(np.array([math.sin(-pose_theta), math.cos(-pose_theta)]),
                              lookahead_point[0:2] - position))
    speed = float(lookahead_point[2])
    if abs(waypoint_y) < 1e-6:
        return speed, 0.0
    radius = 1.0 / (2.0 * waypoint_y / (lookahead_distance ** 2))
    steering_angle = math.atan(wheelbase / radius)
    return speed, steering_angle


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class PurePursuitPlanner:
    def __init__(self, waypoints: np.ndarray, xind: int, yind: int, vind: int, wheelbase: float):
        self.wheelbase = float(wheelbase)
        self.waypoints = waypoints
        self.xind = int(xind)
        self.yind = int(yind)
        self.vind = int(vind)
        self.max_reacquire = 20.0

    def _get_current_waypoint(self, lookahead_distance: float, position: np.ndarray, theta: float):
        wpts_xy = np.vstack((self.waypoints[:, self.xind], self.waypoints[:, self.yind])).T
        nearest_point, nearest_dist, t, i = nearest_point_on_trajectory(position, wpts_xy)

        if nearest_dist < lookahead_distance:
            _, i2, _ = first_point_on_trajectory_intersecting_circle(
                position, lookahead_distance, wpts_xy, i + t, wrap=True
            )
            if i2 is None:
                return None
            current_waypoint = np.empty((3,))
            current_waypoint[0:2] = wpts_xy[i2, :]
            current_waypoint[2] = self.waypoints[i2, self.vind]
            return current_waypoint

        if nearest_dist < self.max_reacquire:
            return np.array([wpts_xy[i, 0], wpts_xy[i, 1], self.waypoints[i, self.vind]], dtype=float)

        return None

    def plan(self, pose_x: float, pose_y: float, pose_theta: float,
             lookahead_distance: float, vgain: float):
        position = np.array([pose_x, pose_y], dtype=float)
        lookahead_point = self._get_current_waypoint(lookahead_distance, position, pose_theta)
        if lookahead_point is None:
            return 0.0, 0.0  # fail-safe: stop

        speed, steering_angle = get_actuation(
            pose_theta, lookahead_point, position, lookahead_distance, self.wheelbase
        )
        speed = float(vgain) * float(speed)
        return speed, float(steering_angle)


class PurePursuitNode(Node):
    def __init__(self):
        super().__init__("pure_pursuit")

        # ---------- params ----------
        self.declare_parameter("waypoints_path", "/racetracks_host/Budapest/Budapest_raceline.csv")
        self.declare_parameter("delimiter", ";")
        self.declare_parameter("skiprows", 3)

        # Budapest_raceline.csv: s;x;y;psi;kappa;vx;ax
        self.declare_parameter("x_index", 1)
        self.declare_parameter("y_index", 2)
        self.declare_parameter("v_index", 5)

        self.declare_parameter("lookahead", 1.2)
        self.declare_parameter("vgain", 1.0)
        self.declare_parameter("wheelbase", 0.33)
        self.declare_parameter("max_steer", 0.30)
        self.declare_parameter("rate_hz", 20.0)

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "ego_racecar/base_link")
        self.declare_parameter("drive_topic", "/drive")

        # Waypoint marker visualization
        self.declare_parameter("waypoints_marker_topic", "/pp_waypoints")
        self.declare_parameter("waypoints_point_size", 0.1)
        self.declare_parameter("waypoints_z", 0.0)

        # ---------- load waypoints ----------
        wpt_path = self.get_parameter("waypoints_path").value
        delim = self.get_parameter("delimiter").value
        skip = int(self.get_parameter("skiprows").value)
        self.xi = int(self.get_parameter("x_index").value)
        self.yi = int(self.get_parameter("y_index").value)
        self.vi = int(self.get_parameter("v_index").value)

        self.lookahead = float(self.get_parameter("lookahead").value)
        self.vgain = float(self.get_parameter("vgain").value)
        self.wheelbase = float(self.get_parameter("wheelbase").value)
        self.max_steer = float(self.get_parameter("max_steer").value)
        self.rate_hz = float(self.get_parameter("rate_hz").value)

        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.drive_topic = self.get_parameter("drive_topic").value

        self.waypoints = np.loadtxt(wpt_path, delimiter=delim, skiprows=skip)
        self.planner = PurePursuitPlanner(self.waypoints, self.xi, self.yi, self.vi, self.wheelbase)

        # ---------- TF ----------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------- pub: drive ----------
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)

        # ---------- pub: waypoint marker (TRANSIENT_LOCAL so RViz can join late) ----------
        marker_qos = QoSProfile(depth=1)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.marker_topic = self.get_parameter("waypoints_marker_topic").value
        self.wp_marker_pub = self.create_publisher(Marker, self.marker_topic, marker_qos)

        # publish marker once (and once more after 1s to be safe)
        self.publish_waypoints_marker()
        self.create_timer(1.0, self._republish_marker_once)
        self._marker_republished = False

        # ---------- timer ----------
        self.timer = self.create_timer(1.0 / self.rate_hz, self.on_timer)

        self.get_logger().info(
            f"PurePursuitNode ready. wpts={self.waypoints.shape[0]}, "
            f"lookahead={self.lookahead}, vgain={self.vgain}, marker={self.marker_topic}"
        )

    def _republish_marker_once(self):
        if self._marker_republished:
            return
        self.publish_waypoints_marker()
        self._marker_republished = True

    def publish_waypoints_marker(self):
        m = Marker()
        m.header.frame_id = self.map_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "pp_waypoints"
        m.id = 0
        m.type = Marker.POINTS
        m.action = Marker.ADD

        size = float(self.get_parameter("waypoints_point_size").value)
        z = float(self.get_parameter("waypoints_z").value)

        m.scale.x = size
        m.scale.y = size

        # yellow points
        m.color.a = 1.0
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 0.0

        # points
        m.points = []
        for i in range(self.waypoints.shape[0]):
            p = Point()
            p.x = float(self.waypoints[i, self.xi])
            p.y = float(self.waypoints[i, self.yi])
            p.z = z
            m.points.append(p)

        self.wp_marker_pub.publish(m)

    def on_timer(self):
        # TF로 현재 pose 읽기
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        except Exception:
            return

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)

        speed, steer = self.planner.plan(x, y, yaw, self.lookahead, self.vgain)
        steer = max(-self.max_steer, min(self.max_steer, steer))

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steer)
        self.drive_pub.publish(msg)


def main():
    rclpy.init()
    node = PurePursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
