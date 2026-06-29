#!/usr/bin/env bash
set +e

cd "$HOME/tp_final/robotica-tp-final" || exit 1
source /opt/ros/humble/setup.bash
source install/setup.bash

export TURTLEBOT3_MODEL=burger
mkdir -p run_logs

RUN="nav_gridmap_smoke_$(date +%H%M%S)"
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

echo "[1/6] Limpiando procesos previos"
pkill -INT -x gzserver 2>/dev/null || true
pkill -INT -x gzclient 2>/dev/null || true
pkill -INT -f "mcl_localization|navigator|rviz2_navigation" 2>/dev/null || true
pkill -INT -f "spawn_entity.py" 2>/dev/null || true
sleep 4

echo "[2/6] Simulacion headless"
ros2 launch slam_gridmap casa_headless.launch.py > "${LOG}_sim.log" 2>&1 &
SIM_PID=$!
sleep 22
if ! timeout 8 ros2 topic echo /scan --once >/tmp/${RUN}_scan.txt 2>&1; then
  echo "ERROR: no hay /scan"
  cat /tmp/${RUN}_scan.txt
  tail -n 80 "${LOG}_sim.log"
  cleanup
  exit 2
fi
if ! timeout 8 ros2 topic echo /calc_odom --once >/tmp/${RUN}_odom.txt 2>&1; then
  echo "ERROR: no hay /calc_odom"
  cat /tmp/${RUN}_odom.txt
  cleanup
  exit 3
fi

echo "[3/6] Navegacion sin RViz"
ros2 launch nav_gridmap navigation.launch.py rviz:=false > "${LOG}_nav.log" 2>&1 &
NAV_PID=$!
sleep 10

echo "[4/6] Publicando pose inicial"
ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}, covariance: [0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.10]}}" \
  --once >/dev/null 2>&1
sleep 5

echo "[5/6] Publicando goal"
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}" \
  --once >/dev/null 2>&1
sleep 12

echo "[6/6] Verificando salidas"
echo "--- topics ---"
ros2 topic list | grep -E '^/(amcl_pose|particlecloud|plan|nav_state|cmd_vel|map)$' || true
echo "--- nav_state ---"
timeout 5 ros2 topic echo /nav_state --once || true
echo "--- plan ---"
timeout 5 ros2 topic echo /plan --once >/tmp/${RUN}_plan.txt 2>&1
if [ $? -eq 0 ]; then
  grep -E 'frame_id|poses:' /tmp/${RUN}_plan.txt | head -20 || true
  echo "PLAN_OK"
else
  if grep -q "Camino planificado" "${LOG}_nav.log"; then
    echo "PLAN_OK_LOG"
  else
    echo "PLAN_MISSING"
    cat /tmp/${RUN}_plan.txt
  fi
fi
echo "--- cmd_vel ---"
timeout 5 ros2 topic echo /cmd_vel --once || true
echo "--- nav log tail ---"
tail -n 80 "${LOG}_nav.log" || true

cleanup
echo "LOG_PREFIX=$LOG"
