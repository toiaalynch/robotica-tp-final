#!/usr/bin/env bash

# Prueba offline de Parte C con el rosbag laberinto_conos.
# Levanta SLAM, detector de cono rojo y reproduce el bag desde una ventana
# donde suele aparecer el cono. Los resultados quedan guardados en OUT.

set -eo pipefail

ROOT="${ROOT:-$HOME/tp_final/robotica-tp-final}"
BAG="${BAG:-$HOME/tp_final/rosbags/laberinto_conos}"
OUT="${OUT:-$HOME/tp_final/resultados_rosbag_conos_check}"
RATE="${RATE:-0.5}"
# Arrancar desde 0 evita perder /tf_static. Si se usa --start-offset en este
# rosbag, puede faltar el frame de la camara y la deteccion queda sin goal.
START_OFFSET="${START_OFFSET:-0}"
DURATION="${DURATION:-200}"
TARGET_FRAME="${TARGET_FRAME:-map}"

source /opt/ros/humble/setup.bash
cd "$ROOT"
source install/setup.bash

rm -rf "$OUT"
mkdir -p "$OUT"

cleanup() {
  kill ${STATUS_PID:-} ${GOAL_PID:-} ${CONE_PID:-} ${SLAM_PID:-} 2>/dev/null || true
}
trap cleanup EXIT

ros2 daemon stop >/dev/null 2>&1 || true
sleep 1
ros2 daemon start >/dev/null 2>&1 || true

ros2 run slam_gridmap grid_fastslam --ros-args \
  -p use_sim_time:=true \
  -p robot_type:=tb4 \
  -p map_save_path:="$OUT/mapa_laberinto_conos_check" \
  -p autosave_period:=10.0 \
  > "$OUT/slam.log" 2>&1 &
SLAM_PID=$!

sleep 8

ros2 launch nav_gridmap red_cone_mission.launch.py \
  use_sim_time:=true \
  robot_type:=tb4 \
  image_topic:=/tb4_0/oakd/rgb/preview/image_raw \
  camera_info_topic:=/tb4_0/oakd/rgb/preview/camera_info \
  target_frame:="$TARGET_FRAME" \
  auto_goal:=false \
  tf_timeout:=1.0 \
  tf_cache_time:=120.0 \
  > "$OUT/cone_node.log" 2>&1 &
CONE_PID=$!

sleep 4

ros2 topic echo /red_cone/status > "$OUT/red_cone_status.log" 2>&1 &
STATUS_PID=$!
ros2 topic echo /red_cone/goal_pose > "$OUT/red_cone_goal_pose.log" 2>&1 &
GOAL_PID=$!

timeout "${DURATION}s" ros2 bag play "$BAG" --clock \
  --start-offset "$START_OFFSET" \
  --rate "$RATE" \
  --remap /tb4_0/tf:=/tf /tb4_0/tf_static:=/tf_static \
  > "$OUT/bag_play.log" 2>&1 || true

sleep 3
cleanup

python3 - <<PY
from pathlib import Path
from PIL import Image

base = Path("$OUT/mapa_laberinto_conos_check.pgm")
if base.exists():
    Image.open(base).convert("RGB").save(base.with_suffix(".png"))
    print(f"Mapa PNG: {base.with_suffix('.png')}")
else:
    print(f"No se encontro mapa en {base}")
PY

echo
echo "Resumen de detecciones:"
grep -h "cono rojo" "$OUT/red_cone_status.log" "$OUT/cone_node.log" 2>/dev/null \
  | tail -20 || true

echo
echo "Archivos generados:"
ls -lh "$OUT"
