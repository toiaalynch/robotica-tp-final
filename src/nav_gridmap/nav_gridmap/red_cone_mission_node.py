#!/usr/bin/env python3
"""
Nodo de mision Parte C: detectar conos rojos y enviar goal navegable.
=====================================================================

La consigna remarca que ver un cono a traves de una abertura NO implica que el
robot deba ir en linea recta. Este nodo separa percepcion de navegacion:

  1) Segmenta conos rojos en la imagen de camara.
  2) Estima una coordenada 3D usando depth o, si no hay depth, una distancia
     aproximada por tamano aparente.
  3) Transforma esa coordenada al frame ``map``.
  4) Publica un ``/goal_pose`` para que el planificador A* decida una ruta
     valida alrededor de paredes y obstaculos.

Con ``auto_goal:=false`` solo publica ``/red_cone/goal_pose`` para inspeccion.
"""

import math

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
import tf2_ros

from .red_cone_vision import RedConeDetector


def as_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def yaw_to_quat_zw(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quat_to_matrix(q):
    """Matriz de rotacion 3x3 desde quaternion geometry_msgs."""
    x, y, z, w = q.x, q.y, q.z, q.w
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ], dtype=np.float64)


def image_to_rgb(msg):
    """Convierte sensor_msgs/Image a RGB numpy sin cv_bridge.

    Soporta los formatos mas comunes de rosbag/camaras: rgb8, bgr8, rgba8,
    bgra8 y mono8. Si aparece otro encoding, falla explicitamente para que el
    grupo pueda adaptar el parametro/camara antes del laboratorio.
    """
    h, w = int(msg.height), int(msg.width)
    enc = msg.encoding.lower()
    data = np.frombuffer(msg.data, dtype=np.uint8)

    if enc == "rgb8":
        rows = data.reshape(h, int(msg.step))
        return rows[:, :w * 3].reshape(h, w, 3).copy()
    if enc == "bgr8":
        rows = data.reshape(h, int(msg.step))
        return rows[:, :w * 3].reshape(h, w, 3)[..., ::-1].copy()
    if enc == "rgba8":
        rows = data.reshape(h, int(msg.step))
        return rows[:, :w * 4].reshape(h, w, 4)[..., :3].copy()
    if enc == "bgra8":
        rows = data.reshape(h, int(msg.step))
        return rows[:, :w * 4].reshape(h, w, 4)[..., 2::-1].copy()
    if enc == "mono8":
        rows = data.reshape(h, int(msg.step))
        mono = rows[:, :w]
        return np.repeat(mono[..., None], 3, axis=2)
    raise ValueError(f"Encoding de imagen no soportado: {msg.encoding}")


def depth_to_meters(msg):
    """Convierte imagen de profundidad a metros."""
    h, w = int(msg.height), int(msg.width)
    enc = msg.encoding.lower()
    if enc in ("16uc1", "mono16"):
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, int(msg.step))
        arr = raw[:, :w * 2].view(np.uint16).reshape(h, w)
        return arr.astype(np.float32) * 0.001
    if enc == "32fc1":
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, int(msg.step))
        return raw[:, :w * 4].view(np.float32).reshape(h, w).copy()
    raise ValueError(f"Encoding de depth no soportado: {msg.encoding}")


class RedConeMissionNode(Node):
    def __init__(self):
        super().__init__("red_cone_mission")

        self.declare_parameter("robot_type", "tb4")
        self.declare_parameter("image_topic", "/tb4_0/color/image")
        self.declare_parameter("depth_topic", "")
        self.declare_parameter("camera_info_topic", "")
        self.declare_parameter("camera_frame", "")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("auto_goal", False)
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("stable_detections", 3)
        self.declare_parameter("cooldown", 5.0)
        self.declare_parameter("goal_offset_m", 0.35)
        self.declare_parameter("fallback_distance_m", 1.2)
        self.declare_parameter("cone_height_m", 0.30)
        self.declare_parameter("min_area", 120)
        self.declare_parameter("red_min", 70)
        self.declare_parameter("saturation_min", 0.35)
        self.declare_parameter("value_min", 0.12)
        self.declare_parameter("tf_timeout", 0.2)

        gp = self.get_parameter
        self.robot_type = gp("robot_type").value
        self.image_topic = gp("image_topic").value
        self.depth_topic = gp("depth_topic").value
        self.camera_info_topic = gp("camera_info_topic").value
        self.camera_frame = gp("camera_frame").value
        self.target_frame = gp("target_frame").value
        self.auto_goal = as_bool(gp("auto_goal").value)
        self.goal_topic = gp("goal_topic").value
        self.stable_needed = int(gp("stable_detections").value)
        self.cooldown = float(gp("cooldown").value)
        self.goal_offset_m = float(gp("goal_offset_m").value)
        self.fallback_distance_m = float(gp("fallback_distance_m").value)
        self.cone_height_m = float(gp("cone_height_m").value)
        self.tf_timeout = float(gp("tf_timeout").value)

        self.detector = RedConeDetector(
            min_area=int(gp("min_area").value),
            red_min=float(gp("red_min").value),
            saturation_min=float(gp("saturation_min").value),
            value_min=float(gp("value_min").value),
        )

        self.camera_info = None
        self.last_depth = None
        self.stable_count = 0
        self.last_goal_time = -1e9

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(Image, self.image_topic, self.image_cb,
                                 qos_profile_sensor_data)
        if self.depth_topic:
            self.create_subscription(Image, self.depth_topic, self.depth_cb,
                                     qos_profile_sensor_data)
        if self.camera_info_topic:
            self.create_subscription(CameraInfo, self.camera_info_topic,
                                     self.camera_info_cb, 10)

        self.pub_goal = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.pub_cone_goal = self.create_publisher(PoseStamped,
                                                   "/red_cone/goal_pose", 10)
        self.pub_status = self.create_publisher(String, "/red_cone/status", 10)

        mode = "AUTO /goal_pose" if self.auto_goal else "solo /red_cone/goal_pose"
        self.get_logger().info(
            f"Parte C red_cone_mission iniciado [{mode}]. image={self.image_topic}")

    def camera_info_cb(self, msg):
        self.camera_info = msg

    def depth_cb(self, msg):
        try:
            self.last_depth = (depth_to_meters(msg), msg.header)
        except ValueError as exc:
            self.get_logger().warn(str(exc))

    def image_cb(self, msg):
        try:
            rgb = image_to_rgb(msg)
        except ValueError as exc:
            self.get_logger().warn(str(exc))
            return

        det = self.detector.detect(rgb)
        if not det.found:
            self.stable_count = 0
            self._status("sin cono rojo")
            return

        self.stable_count += 1
        goal = self._goal_from_detection(det, msg)
        if goal is None:
            self._status(f"cono rojo detectado, sin TF/profundidad estable area={det.area}")
            return

        self.pub_cone_goal.publish(goal)
        self._status(
            f"cono rojo area={det.area} bbox={det.bbox} goal="
            f"({goal.pose.position.x:.2f},{goal.pose.position.y:.2f})")

        now = self._now()
        if (self.auto_goal and self.stable_count >= self.stable_needed
                and (now - self.last_goal_time) >= self.cooldown):
            self.pub_goal.publish(goal)
            self.last_goal_time = now
            self.get_logger().info(
                f"Goal al cono rojo publicado en /goal_pose: "
                f"({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})")

    def _goal_from_detection(self, det, image_msg):
        u, v = det.center
        z = self._estimate_depth(det)
        if z is None or not np.isfinite(z) or z <= 0.05:
            z = self._estimate_depth_from_bbox(det, image_msg)
        if z is None:
            return None

        fx, fy, cx, cy = self._camera_intrinsics(image_msg)
        # Frame optico: x derecha, y abajo, z adelante.
        point_cam = np.array([(u - cx) * z / fx, (v - cy) * z / fy, z],
                             dtype=np.float64)
        frame = self.camera_frame or image_msg.header.frame_id
        if not frame:
            self.get_logger().warn("No hay camera_frame ni frame_id en la imagen.")
            return None

        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame, frame, Time(),
                timeout=Duration(seconds=self.tf_timeout))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            self.get_logger().warn(f"Sin TF {self.target_frame}<-{frame}: {exc}")
            return None

        R = quat_to_matrix(tf.transform.rotation)
        t = np.array([tf.transform.translation.x,
                      tf.transform.translation.y,
                      tf.transform.translation.z], dtype=np.float64)
        point_map = R @ point_cam + t

        # Offset: apuntar unos centimetros antes del cono para no embestirlo.
        robot_xy = self._robot_xy()
        gx, gy = float(point_map[0]), float(point_map[1])
        if robot_xy is not None and self.goal_offset_m > 0:
            rx, ry = robot_xy
            vx, vy = gx - rx, gy - ry
            d = math.hypot(vx, vy)
            if d > self.goal_offset_m:
                gx -= self.goal_offset_m * vx / d
                gy -= self.goal_offset_m * vy / d

        yaw = 0.0
        if robot_xy is not None:
            yaw = math.atan2(float(point_map[1]) - robot_xy[1],
                             float(point_map[0]) - robot_xy[0])

        ps = PoseStamped()
        ps.header.frame_id = self.target_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = gx
        ps.pose.position.y = gy
        ps.pose.position.z = 0.0
        ps.pose.orientation.z, ps.pose.orientation.w = yaw_to_quat_zw(yaw)
        return ps

    def _estimate_depth(self, det):
        if self.last_depth is None:
            return None
        depth, _ = self.last_depth
        x, y, w, h = det.bbox
        pad_x = max(1, int(0.2 * w))
        pad_y = max(1, int(0.2 * h))
        roi = depth[y + pad_y:y + h - pad_y, x + pad_x:x + w - pad_x]
        if roi.size == 0:
            return None
        valid = roi[np.isfinite(roi) & (roi > 0.05)]
        if valid.size < 5:
            return None
        return float(np.median(valid))

    def _estimate_depth_from_bbox(self, det, image_msg):
        _, _, _, h = det.bbox
        fx, fy, _, _ = self._camera_intrinsics(image_msg)
        if h <= 2 or self.cone_height_m <= 0:
            return self.fallback_distance_m
        z = self.cone_height_m * fy / float(h)
        return float(np.clip(z, 0.25, max(self.fallback_distance_m, 0.3)))

    def _camera_intrinsics(self, image_msg):
        if self.camera_info is not None:
            k = self.camera_info.k
            return float(k[0]), float(k[4]), float(k[2]), float(k[5])
        # Fallback razonable si el rosbag no trae CameraInfo.
        w = float(image_msg.width)
        h = float(image_msg.height)
        fx = fy = max(w, h)
        return fx, fy, 0.5 * w, 0.5 * h

    def _robot_xy(self):
        for base in ("base_link", "base_footprint"):
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.target_frame, base, Time(),
                    timeout=Duration(seconds=self.tf_timeout))
                return (float(tf.transform.translation.x),
                        float(tf.transform.translation.y))
            except Exception:
                pass
        return None

    def _status(self, text):
        msg = String()
        msg.data = text
        self.pub_status.publish(msg)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = RedConeMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
