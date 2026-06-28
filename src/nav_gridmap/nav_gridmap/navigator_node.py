#!/usr/bin/env python3
"""
Nodo ROS 2 Navegador — Maquina de estados de la Parte B (Sistema 1).
====================================================================

Orquesta planificacion + seguimiento + evasion con una MAQUINA DE ESTADOS, tal
como pide la consigna. La matematica vive en planner.py, controller.py y
obstacle.py (testeables sin ROS); aca va la plomeria y la logica de estados.

Flujo de estados:

      +-----------+  /goal_pose   +-----------+  ok   +-----------+
      | WAIT_GOAL | ------------> | PLANNING  | ----> | FOLLOWING |
      +-----------+               +-----------+       +-----------+
            ^                          | falla              |  obstaculo / nuevo goal
            |                          v                    v
            |                    (avisa y vuelve)     (marca y RE-PLANIFICA)
            |                                              |
            |                         atasco               v
            |                    +-------------------< RECOVERY
            |   posicion + angulo final alcanzados          |
            +-----------------------< REACHED <-------------+

Requisitos cubiertos:
  - Localizacion: la pose viene del TF map->base (lo provee el nodo MCL).
  - Pose inicial / goal: se fijan desde RViz (2D Pose Estimate / 2D Goal Pose).
  - Planificacion segura (A* con inflado), seguimiento suave (pure pursuit),
    ANGULO final, REPETICIONES (nuevo goal -> re-planifica), y EVASION de
    obstaculos no mapeados (parada de seguridad + re-planificacion).

Entradas:  /goal_pose (PoseStamped), /scan (LaserScan), TF map->base.
Salidas:   /cmd_vel (Twist), /plan (Path), /nav_state (std_msgs/String).
"""

import math
import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration
from rclpy.time import Time

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from std_msgs.msg import String
import tf2_ros

from .static_map import StaticGridMap
from .planner import GridPlanner
from .controller import PurePursuitController
from .obstacle import detect_unmapped


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def yaw_to_quat_zw(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


ROBOT_PROFILES = {
    "tb3": {"scan_topic": "/scan", "cmd_vel_topic": "/cmd_vel",
            "base_frame": "base_footprint"},
    "tb4": {"scan_topic": "/tb4_0/scan", "cmd_vel_topic": "/cmd_vel",
            "base_frame": "base_link"},
}


class NavigatorNode(Node):
    def __init__(self):
        super().__init__("navigator")

        # --- perfil de robot ---
        self.robot_type = self.declare_parameter("robot_type", "tb3").value
        prof = ROBOT_PROFILES.get(self.robot_type, ROBOT_PROFILES["tb3"])

        # --- mapa ---
        self.declare_parameter("map_yaml", "")

        # --- planificador ---
        self.declare_parameter("robot_radius", 0.14)
        self.declare_parameter("inflation_radius", 0.35)
        self.declare_parameter("inflation_cost", 8.0)
        self.declare_parameter("simplify_path", True)

        # --- controlador (pure pursuit) ---
        self.declare_parameter("lookahead", 0.35)
        self.declare_parameter("v_max", 0.22)
        self.declare_parameter("w_max", 2.5)
        self.declare_parameter("goal_tol", 0.10)
        self.declare_parameter("yaw_tol", 0.08)
        self.declare_parameter("slow_radius", 0.45)

        # --- evasion de obstaculos ---
        self.declare_parameter("stop_distance", 0.25)     # frenado de emergencia [m]
        self.declare_parameter("obstacle_check_period", 0.2)  # s
        self.declare_parameter("dynamic_ttl", 6.0)        # vida de un obstaculo dinamico [s]
        self.declare_parameter("dynamic_inflate_cells", 3)  # inflado extra de obstaculos nuevos
        self.declare_parameter("replan_cooldown", 1.0)    # s entre re-planificaciones
        self.declare_parameter("wall_tol", 0.20)
        self.declare_parameter("clear_dynamic_on_new_goal", True)

        # --- recuperacion ante atasco ---
        self.declare_parameter("stuck_timeout", 8.0)      # s sin progreso antes de recuperar
        self.declare_parameter("stuck_min_progress", 0.08)  # mejora minima [m] al goal
        self.declare_parameter("recovery_time", 1.6)      # s girando en el lugar
        self.declare_parameter("recovery_w", 0.7)         # rad/s
        self.declare_parameter("max_recoveries", 3)

        # --- general ---
        self.declare_parameter("control_rate", 20.0)      # Hz
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("scan_topic", prof["scan_topic"])
        self.declare_parameter("cmd_vel_topic", prof["cmd_vel_topic"])
        self.declare_parameter("base_frame", prof["base_frame"])
        self.declare_parameter("max_range", 0.0)
        self.declare_parameter("tf_timeout", 0.2)

        gp = self.get_parameter

        # cargar el mapa
        map_yaml = gp("map_yaml").value
        if not map_yaml or not os.path.exists(map_yaml):
            raise RuntimeError(
                f"Parametro 'map_yaml' invalido: '{map_yaml}'. Usa el launch de "
                f"navegacion, que calcula la ruta del mapa.")
        self.smap = StaticGridMap.from_yaml(map_yaml)

        # planificador y controlador
        self.planner = GridPlanner(
            self.smap,
            robot_radius=float(gp("robot_radius").value),
            inflation_radius=float(gp("inflation_radius").value),
            inflation_cost=float(gp("inflation_cost").value))
        self.controller = PurePursuitController(
            lookahead=float(gp("lookahead").value),
            v_max=float(gp("v_max").value), w_max=float(gp("w_max").value),
            goal_tol=float(gp("goal_tol").value),
            yaw_tol=float(gp("yaw_tol").value),
            slow_radius=float(gp("slow_radius").value))

        # parametros de runtime
        self.map_frame = gp("map_frame").value
        self.base_frame = gp("base_frame").value
        self.stop_distance = float(gp("stop_distance").value)
        self.obstacle_check_period = float(gp("obstacle_check_period").value)
        self.dynamic_ttl = float(gp("dynamic_ttl").value)
        self.dynamic_inflate_cells = int(gp("dynamic_inflate_cells").value)
        self.replan_cooldown = float(gp("replan_cooldown").value)
        self.wall_tol = float(gp("wall_tol").value)
        self.clear_dynamic_on_new_goal = bool(gp("clear_dynamic_on_new_goal").value)
        self.max_range_param = float(gp("max_range").value)
        self.tf_timeout = float(gp("tf_timeout").value)
        self.simplify_path = bool(gp("simplify_path").value)
        self.stuck_timeout = float(gp("stuck_timeout").value)
        self.stuck_min_progress = float(gp("stuck_min_progress").value)
        self.recovery_time = float(gp("recovery_time").value)
        self.recovery_w = float(gp("recovery_w").value)
        self.max_recoveries = int(gp("max_recoveries").value)

        # --- estado ---
        self.state = 'WAIT_GOAL'
        self.goal = None                 # (x, y, yaw)
        self.scan = None                 # (ranges, angles, max_range)
        self.dynamic_cells = {}          # (i,j) -> tiempo (s) de deteccion
        self._last_replan = -1e9
        self._last_obs_check = -1e9
        self._reached_logged = False
        self._best_goal_dist = np.inf
        self._last_progress_time = 0.0
        self._recovery_until = 0.0
        self._recoveries = 0

        # --- TF ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- pub/sub ---
        self.create_subscription(PoseStamped, "/goal_pose", self.goal_cb, 10)
        self.create_subscription(LaserScan, gp("scan_topic").value,
                                 self.scan_cb, qos_profile_sensor_data)
        self.pub_cmd = self.create_publisher(Twist, gp("cmd_vel_topic").value, 10)
        self.pub_plan = self.create_publisher(Path, "/plan", 10)
        self.pub_state = self.create_publisher(String, "/nav_state", 10)

        rate = float(gp("control_rate").value)
        self.dt = 1.0 / rate
        self.create_timer(self.dt, self.control_loop)

        self.get_logger().info(
            f"Navegador iniciado [robot={self.robot_type}]. Esperando goal "
            f"(boton '2D Goal Pose' en RViz)...")

    # ==================================================================
    # Callbacks
    # ==================================================================
    def goal_cb(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        yaw = yaw_from_quat(msg.pose.orientation)
        self.goal = (x, y, yaw)
        self._reached_logged = False
        self._recoveries = 0
        if self.clear_dynamic_on_new_goal:
            self.dynamic_cells.clear()
        self.state = 'PLANNING'           # nuevo goal -> (re)planificar siempre
        self.get_logger().info(
            f"Nuevo goal: ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}deg). Planificando...")

    def scan_cb(self, msg: LaserScan):
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        n = ranges.size
        angles = msg.angle_min + np.arange(n) * msg.angle_increment
        max_range = self.max_range_param if self.max_range_param > 0 else msg.range_max
        valid = np.isfinite(ranges) & (ranges >= msg.range_min)
        self.scan = (ranges[valid], angles[valid], max_range)

    # ==================================================================
    # Pose del robot en el mapa via TF (map -> base).
    # ==================================================================
    def get_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time(),
                timeout=Duration(seconds=self.tf_timeout))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        t = tf.transform.translation
        yaw = yaw_from_quat(tf.transform.rotation)
        return (t.x, t.y, yaw)

    # ==================================================================
    # Bucle de control / maquina de estados (a control_rate Hz).
    # ==================================================================
    def control_loop(self):
        self._publish_state()
        now = self._now()
        pose = self.get_robot_pose()
        if pose is None:
            # sin localizacion todavia: detenerse y esperar
            self._stop()
            return

        if self.state == 'WAIT_GOAL':
            self._stop()
            return

        if self.state == 'PLANNING':
            self._do_planning(pose)
            return

        if self.state == 'FOLLOWING':
            self._do_following(pose, now)
            return

        if self.state == 'RECOVERY':
            self._do_recovery(now)
            return

        if self.state == 'REACHED':
            self._stop()
            if not self._reached_logged:
                self.get_logger().info("Objetivo alcanzado (posicion y angulo).")
                self._reached_logged = True
            self.state = 'WAIT_GOAL'
            return

    # ------------------------------------------------------------------
    def _do_planning(self, pose):
        if self.goal is None:
            self.state = 'WAIT_GOAL'
            return
        self._expire_dynamic()
        cells = list(self.dynamic_cells.keys())
        path = self.planner.plan_with_dynamic(
            (pose[0], pose[1]), (self.goal[0], self.goal[1]), cells,
            inflate_cells=self.dynamic_inflate_cells,
            simplify=self.simplify_path)
        self._last_replan = self._now()
        if path is None or len(path) == 0:
            self.get_logger().warn(
                "No se encontro un camino al objetivo (¿bloqueado o fuera del "
                "mapa?). Esperando un nuevo goal.")
            self._stop()
            self.state = 'WAIT_GOAL'
            return
        self.controller.set_path(path, goal_yaw=self.goal[2])
        self._reset_progress_watch(pose)
        self._publish_plan(path)
        self.state = 'FOLLOWING'
        self.get_logger().info(f"Camino planificado: {len(path)} waypoints. Siguiendo...")

    # ------------------------------------------------------------------
    def _do_following(self, pose, now):
        # --- chequeo de obstaculos (a su propia cadencia) ---
        if (now - self._last_obs_check) >= self.obstacle_check_period and self.scan:
            self._last_obs_check = now
            ranges, angles, max_range = self.scan
            info = detect_unmapped(self.smap, pose, ranges, angles, max_range,
                                   wall_tol=self.wall_tol)
            # parada de emergencia si hay algo muy cerca al frente
            if info['front_dist'] < self.stop_distance:
                self._stop()
                self._register_obstacle(info['cells'], now)
                if (now - self._last_replan) >= self.replan_cooldown:
                    self.get_logger().info(
                        f"Obstaculo a {info['front_dist']:.2f} m al frente. "
                        f"Re-planificando para rodearlo...")
                    self.state = 'PLANNING'
                return
            # obstaculo nuevo cerca (no de emergencia): marcar y, si toca el
            # camino, re-planificar suavemente
            if info['cells'] and info['min_dist'] < (self.stop_distance + 0.6):
                self._register_obstacle(info['cells'], now)
                if (self._path_hits_dynamic()
                        and (now - self._last_replan) >= self.replan_cooldown):
                    self.get_logger().info("Obstaculo sobre la ruta. Re-planificando...")
                    self.state = 'PLANNING'
                    return

        # --- seguimiento normal ---
        v, w, status = self.controller.compute(pose, self.dt)
        self._publish_cmd(v, w)
        if status == 'done':
            self._publish_cmd(0.0, 0.0)
            self.state = 'REACHED'
            return

        if self._is_stuck(pose, now):
            self._stop()
            self._register_scan_as_dynamic(pose, now)
            if self._recoveries >= self.max_recoveries:
                self.get_logger().warn(
                    "El robot no logra progresar; se detiene y espera un nuevo goal.")
                self.state = 'WAIT_GOAL'
                return
            self._recoveries += 1
            self._recovery_until = now + self.recovery_time
            self.get_logger().warn(
                f"Posible atasco detectado. Recuperacion {self._recoveries}/"
                f"{self.max_recoveries}: giro corto y re-planificacion.")
            self.state = 'RECOVERY'

    # ------------------------------------------------------------------
    def _do_recovery(self, now):
        if now < self._recovery_until:
            self._publish_cmd(0.0, self.recovery_w)
            return
        self._stop()
        self.state = 'PLANNING'

    # ------------------------------------------------------------------
    def _reset_progress_watch(self, pose):
        if self.goal is None:
            self._best_goal_dist = np.inf
        else:
            self._best_goal_dist = float(np.hypot(self.goal[0] - pose[0],
                                                  self.goal[1] - pose[1]))
        self._last_progress_time = self._now()

    # ------------------------------------------------------------------
    def _is_stuck(self, pose, now):
        if self.goal is None or self.stuck_timeout <= 0:
            return False
        dist = float(np.hypot(self.goal[0] - pose[0], self.goal[1] - pose[1]))
        if dist < (self._best_goal_dist - self.stuck_min_progress):
            self._best_goal_dist = dist
            self._last_progress_time = now
            return False
        return (now - self._last_progress_time) >= self.stuck_timeout

    # ------------------------------------------------------------------
    def _register_scan_as_dynamic(self, pose, now):
        if not self.scan:
            return
        ranges, angles, max_range = self.scan
        info = detect_unmapped(self.smap, pose, ranges, angles, max_range,
                               wall_tol=self.wall_tol)
        self._register_obstacle(info['cells'], now)

    # ==================================================================
    # Obstaculos dinamicos: registro con expiracion (TTL).
    # ==================================================================
    def _register_obstacle(self, cells, now):
        for c in cells:
            self.dynamic_cells[tuple(c)] = now

    def _expire_dynamic(self):
        now = self._now()
        self.dynamic_cells = {c: t for c, t in self.dynamic_cells.items()
                              if (now - t) < self.dynamic_ttl}

    def _path_hits_dynamic(self):
        """True si algun waypoint del camino actual cae cerca de un obstaculo
        dinamico marcado."""
        if self.controller.path is None or not self.dynamic_cells:
            return False
        res = self.smap.resolution
        for (px, py) in self.controller.path:
            i = int((px - self.smap.origin_x) / res)
            j = int((py - self.smap.origin_y) / res)
            for (ci, cj) in self.dynamic_cells:
                if abs(ci - i) <= 3 and abs(cj - j) <= 3:
                    return True
        return False

    # ==================================================================
    # Helpers de publicacion
    # ==================================================================
    def _publish_cmd(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub_cmd.publish(msg)

    def _stop(self):
        self._publish_cmd(0.0, 0.0)

    def _publish_plan(self, path):
        msg = Path()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for (x, y) in path:
            ps = PoseStamped()
            ps.header.frame_id = self.map_frame
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.pub_plan.publish(msg)

    def _publish_state(self):
        m = String()
        m.data = self.state
        self.pub_state.publish(m)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = NavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
