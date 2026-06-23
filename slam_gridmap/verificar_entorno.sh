#!/usr/bin/env bash
# ============================================================================
# Chequeo previo del entorno para correr slam_gridmap (Robostack / ROS 2).
# Corrélo DENTRO de tu entorno conda activado (conda activate <tu_env>).
# Si además ya lanzaste Gazebo, también verifica que los tópicos estén vivos.
#   Uso:  bash verificar_entorno.sh
# ============================================================================
ok=0; warn=0
chk()  { printf "  \033[32m[OK]\033[0m   %s\n" "$1"; }
bad()  { printf "  \033[31m[FALTA]\033[0m %s\n" "$1"; ok=1; }
note() { printf "  \033[33m[OJO]\033[0m  %s\n" "$1"; warn=1; }

echo "=== 1) ROS 2 ==="
if command -v ros2 >/dev/null 2>&1; then chk "ros2 disponible (ROS_DISTRO=${ROS_DISTRO:-?})"
else bad "no se encuentra 'ros2'. Activá tu entorno: conda activate <tu_env>"; fi

echo "=== 2) Variables de entorno ==="
if [ -n "$TURTLEBOT3_MODEL" ]; then chk "TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL"
else bad "TURTLEBOT3_MODEL sin definir -> export TURTLEBOT3_MODEL=burger"; fi

echo "=== 3) Dependencias de Python (en el entorno conda) ==="
for m in numpy scipy matplotlib; do
  if python -c "import $m" 2>/dev/null; then chk "python: $m importa"
  else bad "python no encuentra '$m' -> mamba install $m   (o pip install $m)"; fi
done

echo "=== 4) Paquete compilado ==="
if ros2 pkg prefix slam_gridmap >/dev/null 2>&1; then
  chk "slam_gridmap instalado ($(ros2 pkg prefix slam_gridmap))"
else
  bad "slam_gridmap no aparece. ¿Compilaste y sourceaste el workspace?"
fi

echo "=== 5) Tópicos en vivo (solo si Gazebo ya está corriendo) ==="
TOPICS=$(ros2 topic list 2>/dev/null)
if [ -z "$TOPICS" ]; then
  note "no hay tópicos todavía (lanzá Gazebo en otra terminal y volvé a correr esto)"
else
  for t in /scan /calc_odom /odom; do
    if echo "$TOPICS" | grep -qx "$t"; then chk "tópico $t presente"
    else note "tópico $t NO está (revisá el nombre o el launch de la sim)"; fi
  done
fi

echo "---------------------------------------------"
if [ "$ok" -eq 0 ]; then
  echo -e "\033[32mEntorno listo para lanzar el SLAM.\033[0m"
else
  echo -e "\033[31mFaltan cosas (ver [FALTA] arriba) antes de lanzar.\033[0m"
fi
exit $ok
