#!/usr/bin/env python3
"""
Nodo ROS 2 de Grid-Based FastSLAM (Parte A, Opcion 1).
======================================================

Este nodo es solo la "plomeria" de ROS: NO tiene la matematica del filtro
adentro (esa vive en grid_fastslam_core.py y modulos, y se testea aparte).
Aca:
  - escuchamos /scan (LIDAR) y /calc_odom (odometria estimada, con deriva),
  - calculamos el delta de movimiento (dr1, dt, dr2) respecto del paso anterior,
  - corremos un paso de SLAM solo cuando el robot se movio lo suficiente
    (keyframe -> hace el algoritmo viable en tiempo real, como gmapping),
  - publicamos todo para verlo en RViz y guardamos el mapa al cerrar.

Entradas (topicos)
  /scan        sensor_msgs/LaserScan   -> rayos del LIDAR
  /calc_odom   nav_msgs/Odometry       -> odometria estimada (pura, con error)
  /odom        nav_msgs/Odometry       -> ground truth (solo para comparar en RViz)

Salidas (topicos)
  /map             nav_msgs/OccupancyGrid -> mapa de la mejor particula (QoS latched)
  /likelihoodfield nav_msgs/OccupancyGrid -> likelihood field (debug, opcional)
  /belief          geometry_msgs/PoseStamped -> pose corregida por SLAM
  /slam/particles  geometry_msgs/PoseArray   -> nube de particulas
  /slam/path       nav_msgs/Path             -> trayectoria estimada (SLAM)
  /slam/odom_path  nav_msgs/Path             -> trayectoria de /calc_odom
  /slam/gt_path    nav_msgs/Path             -> trayectoria de /odom (ground truth)
  TF  map -> odom  -> correccion de SLAM sobre la odometria
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
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster

from .grid_fastslam_core import GridFastSLAM
from .motion_model import normalize_angle


def yaw_to_quat(yaw):
    q = Quaternion()
    q.z = float(math.sin(yaw / 2.0))
    q.w = float(math.cos(yaw / 2.0))
    return q


def yaw_from_quat(q):
    """Extrae el yaw (rotacion en z) de un quaternion."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


# ======================================================================
# Perfiles de robot. La consigna (Parte 0) pide que el codigo sea portable
# entre el TurtleBot3 (simulado) y el TurtleBot4 (real), que difieren en:
#   - prefijo/namespace de los topicos  (tb4 usa /tb4_0/...)
#   - topico de odometria               (tb3: /calc_odom  | tb4: /odom)
#   - politica de QoS de la odometria    (tb4 necesita BEST_EFFORT)
#   - LIDAR rotado respecto del robot    (offset angular en tb4)
#   - lecturas con intensidad 0 invalidas (descartar en tb4)
# Cambias de robot con un solo parametro: robot_type:=tb3 | tb4
# ======================================================================
ROBOT_PROFILES = {
    "tb3": {
        "scan_topic": "/scan",
        "odom_topic": "/calc_odom",
        "gt_odom_topic": "/odom",        # ground truth disponible en Gazebo
        "odom_qos": "reliable",
        "lidar_angle_offset": 0.0,       # LIDAR alineado con el robot
        "discard_zero_intensity": False,
    },
    "tb4": {
        "scan_topic": "/tb4_0/scan",
        "odom_topic": "/odom",
        "gt_odom_topic": "",             # robot real: no hay ground truth
        "odom_qos": "best_effort",
        "lidar_angle_offset": 0.0,       # AJUSTAR con la rotacion real del LIDAR tb4
        "discard_zero_intensity": True,  # descartar lecturas de intensidad 0
    },
}


class GridFastSlamNode(Node):
    def __init__(self):
        super().__init__("grid_fastslam")

        # ============================================================
        # Parametros (se pueden cambiar sin tocar el codigo:
        #   ros2 run ... --ros-args -p num_particles:=50 )
        # ============================================================
        # --- Perfil de robot: ajusta topicos/QoS/fisica con un solo parametro ---
        # Los defaults de los topicos y QoS de mas abajo SALEN de este perfil.
        self.robot_type = self.declare_parameter("robot_type", "tb3").value
        prof = ROBOT_PROFILES.get(self.robot_type, ROBOT_PROFILES["tb3"])

        # filtro
        self.declare_parameter("num_particles", 30)
        self.declare_parameter("alpha", [0.02, 0.02, 0.02, 0.02])
        self.declare_parameter("neff_ratio", 0.5)
        self.declare_parameter("seed", 1)
        # likelihood field
        self.declare_parameter("sigma_hit", 0.20)
        self.declare_parameter("z_hit", 0.85)
        self.declare_parameter("z_rand", 0.15)
        # mapa
        self.declare_parameter("map_width_m", 16.0)
        self.declare_parameter("map_height_m", 16.0)
        self.declare_parameter("map_resolution", 0.05)
        self.declare_parameter("map_origin_x", -8.0)
        self.declare_parameter("map_origin_y", -8.0)
        self.declare_parameter("p_occ", 0.70)
        self.declare_parameter("p_free", 0.35)
        self.declare_parameter("occ_threshold", 0.65)
        # LIDAR / keyframes
        self.declare_parameter("scan_subsample", 4)      # usar 1 de cada k rayos
        self.declare_parameter("max_range", 0.0)         # 0 -> usar range_max del scan
        self.declare_parameter("keyframe_dist", 0.10)    # m para disparar update
        self.declare_parameter("keyframe_angle", 0.10)   # rad para disparar update
        self.declare_parameter("sensor_offset_x", 0.0)   # LIDAR respecto de base
        self.declare_parameter("sensor_offset_y", 0.0)
        # frames y topicos (los defaults dependen del robot_type via 'prof')
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("scan_topic", prof["scan_topic"])
        self.declare_parameter("odom_topic", prof["odom_topic"])
        self.declare_parameter("gt_odom_topic", prof["gt_odom_topic"])
        self.declare_parameter("odom_qos", prof["odom_qos"])           # reliable | best_effort
        self.declare_parameter("lidar_angle_offset", prof["lidar_angle_offset"])
        self.declare_parameter("discard_zero_intensity", prof["discard_zero_intensity"])
        # salidas opcionales y guardado
        self.declare_parameter("publish_likelihoodfield", True)
        self.declare_parameter("map_save_path", "")      # vacio -> ~/maps/mapa_slam
        self.declare_parameter("save_png", True)         # PNG presentable para el informe

        gp = self.get_parameter
        n = int(gp("num_particles").value)
        alpha = tuple(float(a) for a in gp("alpha").value)
        self.scan_subsample = max(1, int(gp("scan_subsample").value))
        self.max_range_param = float(gp("max_range").value)
        self.kf_dist = float(gp("keyframe_dist").value)
        self.kf_angle = float(gp("keyframe_angle").value)
        self.sensor_offset = (float(gp("sensor_offset_x").value),
                              float(gp("sensor_offset_y").value))
        self.lidar_angle_offset = float(gp("lidar_angle_offset").value)
        self.discard_zero_intensity = bool(gp("discard_zero_intensity").value)
        self.map_frame = gp("map_frame").value
        self.odom_frame = gp("odom_frame").value
        self.publish_lf = bool(gp("publish_likelihoodfield").value)

        map_args = dict(
            width_m=float(gp("map_width_m").value),
            height_m=float(gp("map_height_m").value),
            resolution=float(gp("map_resolution").value),
            origin_x=float(gp("map_origin_x").value),
            origin_y=float(gp("map_origin_y").value),
            p_occ=float(gp("p_occ").value),
            p_free=float(gp("p_free").value),
            occ_threshold=float(gp("occ_threshold").value),
        )

        # ============================================================
        # El filtro
        # ============================================================
        self.slam = GridFastSLAM(
            num_particles=n, alpha=alpha, map_args=map_args,
            sigma_hit=float(gp("sigma_hit").value),
            z_hit=float(gp("z_hit").value), z_rand=float(gp("z_rand").value),
            neff_ratio=float(gp("neff_ratio").value),
            seed=int(gp("seed").value))

        # estado interno para los deltas de odometria
        self.last_odom = None        # (x, y, theta) del ultimo keyframe procesado
        self.cur_odom = None         # (x, y, theta) mas reciente recibido
        self.have_scan_meta = False

        # ============================================================
        # QoS: el LIDAR usa best_effort (datos de sensor); el mapa usa
        # transient_local (latched) para que RViz y el map_saver que se
        # conecten despues igual reciban el ultimo mapa.
        # ============================================================
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)

        # Subscripciones. El /scan siempre va best_effort (datos de sensor);
        # la odometria usa el QoS del perfil (tb4 exige best_effort).
        odom_qos = self._make_qos(gp("odom_qos").value, depth=20)
        self.create_subscription(LaserScan, gp("scan_topic").value,
                                 self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, gp("odom_topic").value,
                                 self.odom_cb, odom_qos)
        gt_topic = gp("gt_odom_topic").value
        if gt_topic:                      # vacio (ej. tb4 real) -> sin ground truth
            self.create_subscription(Odometry, gt_topic, self.gt_odom_cb, odom_qos)

        # Publicadores
        self.pub_map = self.create_publisher(OccupancyGrid, "/map", map_qos)
        self.pub_lf = self.create_publisher(OccupancyGrid, "/likelihoodfield", map_qos)
        self.pub_belief = self.create_publisher(PoseStamped, "/belief", 10)
        self.pub_particles = self.create_publisher(PoseArray, "/slam/particles", 10)
        self.pub_path = self.create_publisher(Path, "/slam/path", 10)
        self.pub_odom_path = self.create_publisher(Path, "/slam/odom_path", 10)
        self.pub_gt_path = self.create_publisher(Path, "/slam/gt_path", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.slam_path = Path(); self.slam_path.header.frame_id = self.map_frame
        self.odom_path = Path(); self.odom_path.header.frame_id = self.map_frame
        self.gt_path = Path(); self.gt_path.header.frame_id = self.map_frame

        # publicar el mapa a ritmo fijo (no en cada scan, para no saturar)
        self.create_timer(1.0, self.publish_map)

        # Correccion map->odom: se calcula en cada keyframe y se RE-PUBLICA
        # continuamente (20 Hz) con el timestamp del ultimo scan + tolerancia.
        # Esto evita el error de RViz "timestamp earlier than transform cache":
        # garantiza que SIEMPRE exista una transformada map->odom fresca y
        # alineada con el reloj de la simulacion.
        self.declare_parameter("transform_tolerance", 0.1)
        self.transform_tolerance = float(self.get_parameter("transform_tolerance").value)
        self.map_to_odom = (0.0, 0.0, 0.0)     # (tx, ty, yaw) de map->odom
        self.last_stamp = None                 # timestamp del ultimo /scan
        self.create_timer(0.05, self.broadcast_map_to_odom)

        # Autoguardado periodico del mapa: NO dependas de cerrar con Ctrl+C.
        # Cada 'autosave_period' segundos guarda el .pgm + .yaml (sin PNG, para
        # que sea liviano). Asi, aunque Gazebo se cuelgue o cierres mal la
        # terminal, siempre queda en disco el ultimo mapa.
        self.declare_parameter("autosave_period", 15.0)
        ap = float(self.get_parameter("autosave_period").value)
        if ap > 0:
            self.create_timer(ap, lambda: self.save_map(with_png=False))

        self.get_logger().info(
            f"Grid-Based FastSLAM iniciado [robot={self.robot_type}]: "
            f"{n} particulas, resolucion {map_args['resolution']} m/celda. "
            f"scan={gp('scan_topic').value}  odom={gp('odom_topic').value} "
            f"({gp('odom_qos').value}).")

    # ------------------------------------------------------------------
    # Construye un QoSProfile reliable o best_effort (segun el robot).
    # ------------------------------------------------------------------
    def _make_qos(self, kind, depth=10):
        rel = (ReliabilityPolicy.BEST_EFFORT if str(kind).lower() == "best_effort"
               else ReliabilityPolicy.RELIABLE)
        return QoSProfile(depth=depth, reliability=rel,
                          history=HistoryPolicy.KEEP_LAST)

    # ==================================================================
    # Callbacks de odometria: guardamos la pose mas reciente.
    # ==================================================================
    def odom_cb(self, msg: Odometry):
        self.cur_odom = self._pose_from_odom(msg)
        # trayectoria de odometria pura (para comparar en RViz)
        self._append_path(self.odom_path, self.pub_odom_path, self.cur_odom)

    def gt_odom_cb(self, msg: Odometry):
        gt = self._pose_from_odom(msg)
        self._append_path(self.gt_path, self.pub_gt_path, gt)

    @staticmethod
    def _pose_from_odom(msg: Odometry):
        p = msg.pose.pose
        return (p.position.x, p.position.y, yaw_from_quat(p.orientation))

    # ==================================================================
    # Callback del LIDAR: aca ocurre un paso de SLAM (si hubo movimiento).
    # ==================================================================
    def scan_cb(self, msg: LaserScan):
        self.last_stamp = msg.header.stamp   # para estampar el TF map->odom
        if self.cur_odom is None:
            return                        # todavia no llego odometria

        # --- preparar el escaneo: angulos, filtrado y submuestreo ---
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        n = ranges.size
        # angulo de cada rayo (+ offset si el LIDAR esta rotado, p.ej. TB4)
        angles = (msg.angle_min + np.arange(n) * msg.angle_increment
                  + self.lidar_angle_offset)
        max_range = self.max_range_param if self.max_range_param > 0 else msg.range_max

        # descartar lecturas invalidas (inf, nan, fuera de rango)
        valid = np.isfinite(ranges) & (ranges >= msg.range_min)
        # TB4: descartar lecturas con intensidad 0 (invalidas)
        if self.discard_zero_intensity and len(msg.intensities) == n:
            valid &= (np.asarray(msg.intensities, dtype=np.float64) > 0.0)
        ranges = ranges[valid]; angles = angles[valid]
        # submuestrear (1 de cada k) -> menos rayos = mucho mas rapido
        ranges = ranges[::self.scan_subsample]
        angles = angles[::self.scan_subsample]
        if ranges.size < 10:
            return

        # --- primer escaneo: fijar referencia y mapear sin mover ---
        if self.last_odom is None:
            self.last_odom = self.cur_odom
            self.slam.update(ranges, angles, max_range, self.sensor_offset)
            return

        # --- ¿se movio lo suficiente como para procesar un keyframe? ---
        dr1, dt, dr2, moved = self._delta(self.last_odom, self.cur_odom)
        if not moved:
            return

        # --- PASO DE SLAM: predecir con el delta, corregir con el scan ---
        self.slam.predict(dr1, dt, dr2)
        self.slam.update(ranges, angles, max_range, self.sensor_offset)
        self.last_odom = self.cur_odom

        # --- publicar resultados ---
        self.publish_belief_and_particles()
        self.update_map_to_odom()

    # ==================================================================
    # Delta de odometria (modelo dr1, dt, dr2), respecto del paso anterior.
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
    # Publicar pose corregida (/belief), nube de particulas y trayectoria.
    # ==================================================================
    def publish_belief_and_particles(self):
        now = self.get_clock().now().to_msg()
        bx, by, bth = self.slam.best_pose()

        # /belief: pose corregida por SLAM
        ps = PoseStamped()
        ps.header.frame_id = self.map_frame
        ps.header.stamp = now
        ps.pose.position.x = float(bx)
        ps.pose.position.y = float(by)
        ps.pose.orientation = yaw_to_quat(bth)
        self.pub_belief.publish(ps)

        # /slam/path: trayectoria estimada
        self.slam_path.header.stamp = now
        self.slam_path.poses.append(ps)
        self.pub_path.publish(self.slam_path)

        # /slam/particles: nube de particulas (PoseArray)
        pa = PoseArray()
        pa.header.frame_id = self.map_frame
        pa.header.stamp = now
        for k in range(self.slam.N):
            pp = Pose()
            pp.position.x = float(self.slam.poses[k, 0])
            pp.position.y = float(self.slam.poses[k, 1])
            pp.orientation = yaw_to_quat(self.slam.poses[k, 2])
            pa.poses.append(pp)
        self.pub_particles.publish(pa)

    # ==================================================================
    # TF map -> odom: la "correccion" que SLAM aplica sobre la odometria.
    # Se calcula en cada keyframe:  T_map_odom = T_map_base * inv(T_odom_base).
    # ==================================================================
    def update_map_to_odom(self):
        if self.cur_odom is None:
            return
        mx, my, mth = self.slam.best_pose()        # pose en frame map
        ox, oy, oth = self.cur_odom                # pose en frame odom
        dyaw = normalize_angle(mth - oth)
        c, s = math.cos(dyaw), math.sin(dyaw)
        tx = mx - (c * ox - s * oy)
        ty = my - (s * ox + c * oy)
        self.map_to_odom = (tx, ty, dyaw)          # se guarda; el timer la publica

    # Re-publica la correccion guardada de forma CONTINUA (timer 20 Hz), con el
    # timestamp del ultimo scan + tolerancia. Asi RViz siempre encuentra la
    # transformada y deja de descartar el LaserScan.
    def broadcast_map_to_odom(self):
        if self.last_stamp is not None:
            stamp = (Time.from_msg(self.last_stamp)
                     + Duration(seconds=self.transform_tolerance)).to_msg()
        else:
            stamp = self.get_clock().now().to_msg()

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
    # Publicar el mapa (y opcionalmente el likelihood field) en /map.
    # ==================================================================
    def publish_map(self):
        if not self.slam.map_initialized:
            return
        grid = self.slam.best_grid()
        self.pub_map.publish(self._grid_to_msg(grid.to_occupancy_int8(), grid))

        if self.publish_lf:
            lf = grid.likelihood_field()
            d_max = 1.0                       # m: distancia para saturar el color
            val = np.clip(100.0 * (1.0 - lf / d_max), 0, 100).astype(np.int8)
            self.pub_lf.publish(self._grid_to_msg(val.flatten().tolist(), grid))

    def _grid_to_msg(self, data_list, grid):
        msg = OccupancyGrid()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = float(grid.resolution)
        msg.info.width = int(grid.width)
        msg.info.height = int(grid.height)
        msg.info.origin.position.x = float(grid.origin_x)
        msg.info.origin.position.y = float(grid.origin_y)
        msg.info.origin.orientation.w = 1.0
        msg.data = data_list
        return msg

    # ------------------------------------------------------------------
    # Helper de trayectorias (Path)
    # ------------------------------------------------------------------
    def _append_path(self, path_msg, pub, pose):
        now = self.get_clock().now().to_msg()
        ps = PoseStamped()
        ps.header.frame_id = self.map_frame
        ps.header.stamp = now
        ps.pose.position.x = float(pose[0])
        ps.pose.position.y = float(pose[1])
        ps.pose.orientation = yaw_to_quat(pose[2])
        path_msg.header.stamp = now
        path_msg.poses.append(ps)
        pub.publish(path_msg)

    # ==================================================================
    # Guardar el mapa final en formato map_server (.pgm + .yaml),
    # directamente reutilizable como insumo de las Partes B y C.
    # ==================================================================
    def save_map(self, with_png=True):
        if not self.slam.map_initialized:
            return                         # todavia no hay nada que guardar
        path = self.get_parameter("map_save_path").value
        if not path:
            path = os.path.join(os.path.expanduser("~"), "maps", "mapa_slam")
        os.makedirs(os.path.dirname(path), exist_ok=True)

        grid = self.slam.best_grid()
        occ = np.array(grid.to_occupancy_int8(), dtype=np.int16).reshape(
            grid.height, grid.width)

        # convertir a imagen PGM (estandar de map_server):
        #   254 = libre, 0 = ocupado, 205 = desconocido. La imagen va con el eje
        #   Y hacia abajo, por eso invertimos las filas (flipud).
        img = np.full((grid.height, grid.width), 205, dtype=np.uint8)
        img[(occ >= 0) & (occ <= 25)] = 254
        img[occ >= 65] = 0
        img = np.flipud(img)

        pgm = path + ".pgm"
        with open(pgm, "wb") as f:
            f.write(b"P5\n%d %d\n255\n" % (grid.width, grid.height))
            f.write(img.tobytes())

        with open(path + ".yaml", "w") as f:
            f.write(f"image: {os.path.basename(pgm)}\n")
            f.write(f"resolution: {grid.resolution}\n")
            f.write(f"origin: [{grid.origin_x}, {grid.origin_y}, 0.0]\n")
            f.write("negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
        self.get_logger().info(f"Mapa guardado en {pgm} (+ .yaml)")

        # PNG presentable para el informe (solo en el guardado final, no en el
        # autoguardado periodico, porque renderizar con matplotlib es costoso)
        if with_png and self.get_parameter("save_png").value:
            try:
                self._save_png(path + ".png", grid)
                self.get_logger().info(f"PNG del mapa guardado en {path}.png")
            except Exception as e:
                self.get_logger().warn(f"No se pudo guardar el PNG: {e}")

    # ------------------------------------------------------------------
    # PNG "lindo" para el informe: mapa (paredes negras / libre blanco /
    # desconocido gris) con las trayectorias real, odometria y SLAM encima.
    # ------------------------------------------------------------------
    def _save_png(self, png_path, grid):
        import matplotlib
        matplotlib.use("Agg")              # sin ventana (corre en cualquier lado)
        import matplotlib.pyplot as plt

        occ = np.array(grid.to_occupancy_int8(), dtype=float).reshape(
            grid.height, grid.width)
        # imagen RGB: libre=blanco, ocupado=negro, desconocido=gris
        known = occ >= 0
        shade = 1.0 - np.clip(occ, 0, 100) / 100.0
        img = np.empty((grid.height, grid.width, 3))
        for c in range(3):
            img[..., c] = np.where(known, shade, 0.75)

        ext = [grid.origin_x, grid.origin_x + grid.width * grid.resolution,
               grid.origin_y, grid.origin_y + grid.height * grid.resolution]

        fig, ax = plt.subplots(figsize=(8, 8))
        ax.imshow(img, origin="lower", extent=ext, interpolation="nearest")

        def xy(path_msg):
            return ([p.pose.position.x for p in path_msg.poses],
                    [p.pose.position.y for p in path_msg.poses])

        if self.gt_path.poses:
            ax.plot(*xy(self.gt_path), "-", color="#1f9d3a", lw=2.0,
                    label="real (/odom)")
        if self.odom_path.poses:
            ax.plot(*xy(self.odom_path), "-", color="#d23b3b", lw=1.6,
                    label="odometria pura (/calc_odom)")
        if self.slam_path.poses:
            ax.plot(*xy(self.slam_path), "--", color="#2747dd", lw=2.0,
                    label="SLAM (/belief)")

        # recortar a la zona mapeada (con margen) para que no sobre fondo gris
        if known.any():
            js, iss = np.where(known)
            x0 = grid.origin_x + iss.min() * grid.resolution - 0.5
            x1 = grid.origin_x + iss.max() * grid.resolution + 0.5
            y0 = grid.origin_y + js.min() * grid.resolution - 0.5
            y1 = grid.origin_y + js.max() * grid.resolution + 0.5
            ax.set_xlim(x0, x1); ax.set_ylim(y0, y1)

        ax.set_title(f"Mapa de ocupacion - Grid-Based FastSLAM "
                     f"({grid.resolution} m/celda)", fontsize=12)
        ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
        ax.set_aspect("equal"); ax.grid(alpha=0.15)
        if self.gt_path.poses or self.slam_path.poses:
            ax.legend(loc="upper right", fontsize=9, framealpha=0.9)
        fig.tight_layout()
        fig.savefig(png_path, dpi=180)
        plt.close(fig)


def main(args=None):
    rclpy.init(args=args)
    node = GridFastSlamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Cerrando: guardando el mapa final...")
    finally:
        node.save_map()                 # guardado automatico del mapa al salir
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
