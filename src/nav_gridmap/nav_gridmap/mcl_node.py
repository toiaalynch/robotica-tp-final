#!/usr/bin/env python3
"""
Nodo ROS 2 de Localizacion por Monte Carlo (MCL) — Parte B, Sistema 1.
======================================================================

Plomeria de ROS (la matematica vive en mcl.py y static_map.py, y se testea
aparte). Este nodo:
  - carga el MAPA ESTATICO de la Parte A (mapa_fastslam_final_v2) y lo publica
    en /map (latched) para que RViz lo muestre,
  - escucha /initialpose (boton "2D Pose Estimate" de RViz) para sembrar la nube,
  - escucha /scan (LIDAR) y /odom (odometria de Gazebo en TB3),
  - corre un paso de MCL cuando el robot se movio lo suficiente (keyframe),
  - publica la pose estimada /amcl_pose, la nube /particlecloud y, lo mas
    importante, el TF map->odom (la correccion que usaran el planner y el control).

Entradas (topicos)
  /scan        sensor_msgs/LaserScan               -> rayos del LIDAR
  /odom        nav_msgs/Odometry                   -> odometria del robot
  /initialpose geometry_msgs/PoseWithCovarianceStamped -> pose inicial (RViz)

Salidas (topicos)
  /map          nav_msgs/OccupancyGrid               -> mapa estatico (latched)
  /amcl_pose    geometry_msgs/PoseWithCovarianceStamped -> pose estimada + covarianza
  /particlecloud geometry_msgs/PoseArray             -> nube de particulas
  TF map -> odom -> correccion de la localizacion sobre la odometria

El nombre de los topicos sigue la convencion de Nav2/AMCL a proposito, para que
RViz y el resto del stack de la Parte B "enchufen" sin configuracion extra.
"""

import math
import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, qos_profile_sensor_data,
                       ReliabilityPolicy, DurabilityPolicy, HistoryPolicy)
from rclpy.time import Time
from rclpy.duration import Duration

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import (PoseWithCovarianceStamped, PoseArray, Pose,
                               Quaternion, TransformStamped)
from tf2_ros import TransformBroadcaster

from .static_map import StaticGridMap
from .mcl import MonteCarloLocalization
from slam_gridmap.motion_model import normalize_angle


def yaw_to_quat(yaw):
    q = Quaternion()
    q.z = float(math.sin(yaw / 2.0))
    q.w = float(math.cos(yaw / 2.0))
    return q


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


# Perfiles de robot, identicos en espiritu a la Parte A: un solo parametro
# (robot_type) ajusta topicos/QoS/fisica del LIDAR para portar tb3 <-> tb4.
ROBOT_PROFILES = {
    "tb3": {
        "scan_topic": "/scan",
        "odom_topic": "/odom",
        "odom_qos": "reliable",
        "lidar_angle_offset": 0.0,
        "discard_zero_intensity": False,
    },
    "tb4": {
        "scan_topic": "/tb4_0/scan",
        "odom_topic": "/odom",
        "odom_qos": "best_effort",
        "lidar_angle_offset": 0.0,
        "discard_zero_intensity": True,
    },
}


class MclNode(Node):
    def __init__(self):
        super().__init__("mcl_localization")

        # --- Perfil de robot (define defaults de topicos/QoS) ---
        self.robot_type = self.declare_parameter("robot_type", "tb3").value
        prof = ROBOT_PROFILES.get(self.robot_type, ROBOT_PROFILES["tb3"])

        # --- Mapa estatico ---
        self.declare_parameter("map_yaml", "")     # ruta al .yaml del mapa (OBLIGATORIO)

        # --- Filtro MCL ---
        self.declare_parameter("num_particles", 500)
        self.declare_parameter("alpha", [0.05, 0.05, 0.05, 0.05])
        self.declare_parameter("neff_ratio", 0.5)
        self.declare_parameter("seed", 1)
        # likelihood field (modelo del sensor)
        self.declare_parameter("sigma_hit", 0.20)
        self.declare_parameter("z_hit", 0.85)
        self.declare_parameter("z_rand", 0.15)
        # Augmented MCL (recuperacion)
        self.declare_parameter("alpha_slow", 0.001)
        self.declare_parameter("alpha_fast", 0.10)
        # siembra inicial
        self.declare_parameter("init_std_xy", 0.30)      # m
        self.declare_parameter("init_std_theta", 0.25)   # rad
        self.declare_parameter("global_init", False)     # arrancar global (sin initialpose)
        # LIDAR / keyframes
        self.declare_parameter("scan_subsample", 6)      # 1 de cada k rayos
        self.declare_parameter("max_range", 0.0)         # 0 -> range_max del scan
        self.declare_parameter("keyframe_dist", 0.10)    # m para disparar update
        self.declare_parameter("keyframe_angle", 0.10)   # rad para disparar update
        self.declare_parameter("sensor_offset_x", 0.0)
        self.declare_parameter("sensor_offset_y", 0.0)
        # frames y topicos (defaults del perfil)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("scan_topic", prof["scan_topic"])
        self.declare_parameter("odom_topic", prof["odom_topic"])
        self.declare_parameter("odom_qos", prof["odom_qos"])
        self.declare_parameter("lidar_angle_offset", prof["lidar_angle_offset"])
        self.declare_parameter("discard_zero_intensity", prof["discard_zero_intensity"])
        self.declare_parameter("transform_tolerance", 0.1)
        self.declare_parameter("publish_map", True)      # republicar el mapa en /map

        gp = self.get_parameter

        # --- cargar el mapa estatico ---
        map_yaml = gp("map_yaml").value
        if not map_yaml or not os.path.exists(map_yaml):
            raise RuntimeError(
                f"Parametro 'map_yaml' invalido o inexistente: '{map_yaml}'. "
                f"Pasa la ruta al .yaml del mapa de la Parte A "
                f"(usa el launch de localizacion, que la calcula sola).")
        self.smap = StaticGridMap.from_yaml(map_yaml)
        self.get_logger().info(
            f"Mapa cargado: {os.path.basename(map_yaml)} "
            f"({self.smap.width}x{self.smap.height} celdas, "
            f"{self.smap.resolution} m/celda).")

        # --- construir el filtro ---
        self.mcl = MonteCarloLocalization(
            self.smap,
            num_particles=int(gp("num_particles").value),
            alpha=tuple(float(a) for a in gp("alpha").value),
            sigma_hit=float(gp("sigma_hit").value),
            z_hit=float(gp("z_hit").value), z_rand=float(gp("z_rand").value),
            neff_ratio=float(gp("neff_ratio").value),
            alpha_slow=float(gp("alpha_slow").value),
            alpha_fast=float(gp("alpha_fast").value),
            seed=int(gp("seed").value))

        # parametros de runtime
        self.scan_subsample = max(1, int(gp("scan_subsample").value))
        self.max_range_param = float(gp("max_range").value)
        self.kf_dist = float(gp("keyframe_dist").value)
        self.kf_angle = float(gp("keyframe_angle").value)
        self.sensor_offset = (float(gp("sensor_offset_x").value),
                              float(gp("sensor_offset_y").value))
        self.lidar_angle_offset = float(gp("lidar_angle_offset").value)
        self.discard_zero_intensity = bool(gp("discard_zero_intensity").value)
        self.init_std_xy = float(gp("init_std_xy").value)
        self.init_std_theta = float(gp("init_std_theta").value)
        self.map_frame = gp("map_frame").value
        self.odom_frame = gp("odom_frame").value
        self.transform_tolerance = float(gp("transform_tolerance").value)

        # estado interno
        self.last_odom = None     # (x,y,th) de odom en el ultimo keyframe
        self.cur_odom = None      # (x,y,th) de odom mas reciente
        self.last_stamp = None    # timestamp del ultimo /scan (para el TF)
        self.map_to_odom = (0.0, 0.0, 0.0)
        self.have_estimate = False

        # arranque global opcional (sin esperar initialpose)
        if bool(gp("global_init").value):
            self.mcl.initialize_global()
            self.have_estimate = True
            self.get_logger().info("Localizacion GLOBAL: nube uniforme sobre el mapa.")
        else:
            self.get_logger().info(
                "Esperando 'initialpose' (boton '2D Pose Estimate' en RViz) "
                "para sembrar la localizacion...")

        # --- QoS ---
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)
        odom_qos = self._make_qos(gp("odom_qos").value, depth=20)

        # --- subscripciones ---
        self.create_subscription(LaserScan, gp("scan_topic").value,
                                 self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, gp("odom_topic").value,
                                 self.odom_cb, odom_qos)
        self.create_subscription(PoseWithCovarianceStamped, "/initialpose",
                                 self.initialpose_cb, 10)

        # --- publicadores ---
        self.pub_map = self.create_publisher(OccupancyGrid, "/map", map_qos)
        self.pub_pose = self.create_publisher(
            PoseWithCovarianceStamped, "/amcl_pose", 10)
        self.pub_cloud = self.create_publisher(PoseArray, "/particlecloud", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # publicar el mapa periodicamente (latched igual, pero por las dudas)
        if bool(gp("publish_map").value):
            self.create_timer(1.0, self.publish_map)
            self.publish_map()

        # re-publicar el TF map->odom de forma continua (20 Hz) con el stamp del
        # ultimo scan + tolerancia: evita el error de RViz "earlier than cache".
        self.create_timer(0.05, self.broadcast_map_to_odom)

    # ------------------------------------------------------------------
    def _make_qos(self, kind, depth=10):
        rel = (ReliabilityPolicy.BEST_EFFORT if str(kind).lower() == "best_effort"
               else ReliabilityPolicy.RELIABLE)
        return QoSProfile(depth=depth, reliability=rel,
                          history=HistoryPolicy.KEEP_LAST)

    # ==================================================================
    # /initialpose: sembrar la nube alrededor de la pose dada por el usuario.
    # ==================================================================
    def initialpose_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose
        x, y = p.position.x, p.position.y
        th = yaw_from_quat(p.orientation)
        self.mcl.initialize_gaussian(x, y, th,
                                     self.init_std_xy, self.init_std_theta)
        self.last_odom = self.cur_odom     # reinicia el delta desde aca
        self.have_estimate = True
        self.get_logger().info(
            f"Pose inicial fijada en ({x:.2f}, {y:.2f}, {math.degrees(th):.0f}deg). "
            f"Localizando...")
        self.publish_estimate()

    # ==================================================================
    # Odometria: guardar la pose mas reciente.
    # ==================================================================
    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose
        self.cur_odom = (p.position.x, p.position.y, yaw_from_quat(p.orientation))

    # ==================================================================
    # LIDAR: aca corre un paso de MCL (si hubo movimiento suficiente).
    # ==================================================================
    def scan_cb(self, msg: LaserScan):
        self.last_stamp = msg.header.stamp
        if not self.mcl.initialized or self.cur_odom is None:
            return

        # --- preparar el escaneo: angulos, filtrado y submuestreo ---
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        n = ranges.size
        angles = (msg.angle_min + np.arange(n) * msg.angle_increment
                  + self.lidar_angle_offset)
        max_range = self.max_range_param if self.max_range_param > 0 else msg.range_max

        valid = np.isfinite(ranges) & (ranges >= msg.range_min)
        if self.discard_zero_intensity and len(msg.intensities) == n:
            valid &= (np.asarray(msg.intensities, dtype=np.float64) > 0.0)
        ranges = ranges[valid]
        angles = angles[valid]
        ranges = ranges[::self.scan_subsample]
        angles = angles[::self.scan_subsample]
        if ranges.size < 10:
            return

        # --- primer scan tras inicializar: corregir sin esperar movimiento ---
        if self.last_odom is None:
            self.last_odom = self.cur_odom
            self.mcl.update(ranges, angles, max_range, self.sensor_offset)
            self.update_map_to_odom()
            self.publish_estimate()
            return

        # --- ¿se movio lo suficiente? (keyframe) ---
        dr1, dt, dr2, moved = self._delta(self.last_odom, self.cur_odom)
        if not moved:
            return

        # --- PASO DE MCL: predecir con el delta, corregir con el scan ---
        self.mcl.predict(dr1, dt, dr2)
        self.mcl.update(ranges, angles, max_range, self.sensor_offset)
        self.last_odom = self.cur_odom

        self.update_map_to_odom()
        self.publish_estimate()

    # ==================================================================
    # Delta de odometria (modelo dr1, dt, dr2) respecto del ultimo keyframe.
    # ==================================================================
    def _delta(self, prev, cur):
        x0, y0, th0 = prev
        x1, y1, th1 = cur
        dx, dy = x1 - x0, y1 - y0
        dt = math.hypot(dx, dy)
        if dt > 1e-6:
            dr1 = normalize_angle(math.atan2(dy, dx) - th0)
            dr2 = normalize_angle(th1 - th0 - dr1)
        else:
            dr1 = 0.0
            dr2 = normalize_angle(th1 - th0)
        moved = (dt > self.kf_dist) or (abs(normalize_angle(th1 - th0)) > self.kf_angle)
        return dr1, dt, dr2, moved

    # ==================================================================
    # TF map->odom: correccion de la localizacion sobre la odometria.
    #   T_map_odom = T_map_base * inv(T_odom_base)
    # ==================================================================
    def update_map_to_odom(self):
        if self.cur_odom is None:
            return
        mx, my, mth = self.mcl.estimate()      # pose en frame map
        ox, oy, oth = self.cur_odom            # pose en frame odom
        dyaw = normalize_angle(mth - oth)
        c, s = math.cos(dyaw), math.sin(dyaw)
        tx = mx - (c * ox - s * oy)
        ty = my - (s * ox + c * oy)
        self.map_to_odom = (tx, ty, dyaw)

    def broadcast_map_to_odom(self):
        if self.last_stamp is None:
            return
        stamp = (Time.from_msg(self.last_stamp)
                 + Duration(seconds=self.transform_tolerance)).to_msg()
        tx, ty, yaw = self.map_to_odom
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        t.transform.translation.x = float(tx)
        t.transform.translation.y = float(ty)
        t.transform.rotation = yaw_to_quat(yaw)
        self.tf_broadcaster.sendTransform(t)

    # ==================================================================
    # Publicar pose estimada (/amcl_pose) y nube (/particlecloud).
    # ==================================================================
    def publish_estimate(self):
        now = self.get_clock().now().to_msg()
        x, y, th = self.mcl.estimate()
        cxx, cyy, cxy = self.mcl.covariance_xy()

        pc = PoseWithCovarianceStamped()
        pc.header.frame_id = self.map_frame
        pc.header.stamp = now
        pc.pose.pose.position.x = float(x)
        pc.pose.pose.position.y = float(y)
        pc.pose.pose.orientation = yaw_to_quat(th)
        # covarianza 6x6 (x,y,z,roll,pitch,yaw): cargamos el bloque xy y el yaw
        cov = [0.0] * 36
        cov[0] = cxx          # x-x
        cov[1] = cxy          # x-y
        cov[6] = cxy          # y-x
        cov[7] = cyy          # y-y
        cov[35] = 0.05        # yaw-yaw (aprox, para visualizacion)
        pc.pose.covariance = cov
        self.pub_pose.publish(pc)

        pa = PoseArray()
        pa.header.frame_id = self.map_frame
        pa.header.stamp = now
        for k in range(self.mcl.N):
            pp = Pose()
            pp.position.x = float(self.mcl.poses[k, 0])
            pp.position.y = float(self.mcl.poses[k, 1])
            pp.orientation = yaw_to_quat(self.mcl.poses[k, 2])
            pa.poses.append(pp)
        self.pub_cloud.publish(pa)

    # ==================================================================
    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = float(self.smap.resolution)
        msg.info.width = int(self.smap.width)
        msg.info.height = int(self.smap.height)
        msg.info.origin.position.x = float(self.smap.origin_x)
        msg.info.origin.position.y = float(self.smap.origin_y)
        msg.info.origin.orientation.w = 1.0
        msg.data = self.smap.to_occupancy_int8()
        self.pub_map.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MclNode()
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
