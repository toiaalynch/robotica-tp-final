#!/usr/bin/env bash
set +e

# Suite practica para validar la Parte B en Gazebo headless.
# Levanta la casa, la localizacion MCL y el navegador; publica varios goals y
# deja un resumen con estado, odometria y eventos del navegador.

cd "$HOME/tp_final/robotica-tp-final" || exit 1
source /opt/ros/humble/setup.bash
source install/setup.bash

export TURTLEBOT3_MODEL=burger
mkdir -p run_logs

SIM_PACKAGE="${SIM_PACKAGE:-slam_gridmap}"
SIM_LAUNCH="${SIM_LAUNCH:-casa_headless.launch.py}"
RUN_STRESS="${RUN_STRESS:-false}"

RUN="nav_goal_suite_$(date +%H%M%S)"
LOG="run_logs/$RUN"

cleanup() {
  timeout 2 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0}, angular: {z: 0.0}}" -r 10 >/dev/null 2>&1 || true
  [ -n "${NAV_PID:-}" ] && kill -INT "$NAV_PID" 2>/dev/null || true
  [ -n "${SIM_PID:-}" ] && kill -INT "$SIM_PID" 2>/dev/null || true
  sleep 3
  pkill -INT -x gzserver 2>/dev/null || true
  pkill -INT -x gzclient 2>/dev/null || true
}

echo "[1/7] Limpiando procesos previos"
pkill -INT -x gzserver 2>/dev/null || true
pkill -INT -x gzclient 2>/dev/null || true
pkill -INT -f "nav_gridmap" 2>/dev/null || true
pkill -INT -f "spawn_entity.py" 2>/dev/null || true
sleep 4

echo "[2/7] Simulacion: ${SIM_PACKAGE} ${SIM_LAUNCH}"
ros2 launch "$SIM_PACKAGE" "$SIM_LAUNCH" > "${LOG}_sim.log" 2>&1 &
SIM_PID=$!
sleep 22

if ! timeout 8 ros2 topic echo /scan --once >/tmp/${RUN}_scan.txt 2>&1; then
  echo "ERROR: no hay /scan"
  cat /tmp/${RUN}_scan.txt
  tail -n 80 "${LOG}_sim.log"
  cleanup
  exit 2
fi

echo "[3/7] Navegacion completa sin RViz"
ros2 launch nav_gridmap navigation.launch.py rviz:=false > "${LOG}_nav.log" 2>&1 &
NAV_PID=$!
sleep 10

echo "[4/7] Pose inicial aproximada"
ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}, covariance: [0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.10]}}" \
  --once >/dev/null 2>&1
sleep 6

send_goal() {
  local name="$1" x="$2" y="$3" z="$4" w="$5" max_s="$6"
  local start_lines
  start_lines=$(wc -l < "${LOG}_nav.log" 2>/dev/null || echo 0)
  echo
  echo "[GOAL $name] objetivo=($x,$y)"
  ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: map}, pose: {position: {x: $x, y: $y, z: 0.0}, orientation: {z: $z, w: $w}}}" \
    --once >/dev/null 2>&1

  local elapsed=0
  local state=""
  while [ "$elapsed" -lt "$max_s" ]; do
    sleep 3
    elapsed=$((elapsed + 3))
    state=$(timeout 4 ros2 topic echo /nav_state --once 2>/dev/null \
      | awk '/data:/ {print $2}' | tr -d "'\"" || true)
    odom=$(timeout 4 ros2 topic echo /odom --once 2>/dev/null \
      | awk '/position:/ {p=1} p&&/x:/ {x=$2} p&&/y:/ {y=$2; print x "," y; exit}' || true)
    echo "  t=${elapsed}s state=${state:-?} odom=${odom:-?}"
    if [ "$state" = "WAIT_GOAL" ]; then
      break
    fi
  done

  tail -n +"$((start_lines + 1))" "${LOG}_nav.log" > "/tmp/${RUN}_${name}.log"
  if grep -q "Objetivo alcanzado" "/tmp/${RUN}_${name}.log"; then
    echo "  RESULTADO: OK, objetivo alcanzado"
  elif grep -q "no logra progresar" "/tmp/${RUN}_${name}.log"; then
    echo "  RESULTADO: BLOQUEADO/ATASCO"
  elif grep -q "No se encontro un camino" "/tmp/${RUN}_${name}.log"; then
    echo "  RESULTADO: SIN CAMINO"
  else
    echo "  RESULTADO: INCIERTO, revisar log"
  fi
  grep -E "Nuevo goal|Camino planificado|Obstaculo|atasco|Objetivo alcanzado|no logra progresar|No se encontro" \
    "/tmp/${RUN}_${name}.log" || true
}

echo "[5/7] Goals de validacion"
# Dos goals simples que deberian llegar.
send_goal "g1_corto" 1.20 0.40 0.2588 0.9659 90
send_goal "g2_derecha" 2.00 0.80 0.0 1.0 120

# Goal inferior representativo: valida cambio de habitacion/pasillo sin forzar
# la zona peor representada por mesa/patas.
send_goal "g3_abajo_derecha" 2.30 -2.30 -0.7071 0.7071 150

if [ "$RUN_STRESS" = "true" ]; then
  # Goal estresante cerca de la zona mesa/sofa. Sirve para demostrar recuperacion
  # o para detectar que el mapa no representa bien esos obstaculos finos.
  send_goal "g4_mesa_stress" 1.50 -1.60 -0.7071 0.7071 150
fi

echo
echo "[6/7] Topicos clave"
ros2 topic list | grep -E '^/(amcl_pose|particlecloud|plan|nav_state|cmd_vel|map)$' || true

echo
echo "[7/7] Limpieza"
cleanup
echo "LOG_PREFIX=$LOG"
