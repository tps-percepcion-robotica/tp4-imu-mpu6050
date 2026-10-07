#!/usr/bin/env bash
# Lanza el pipeline de la PC: agente micro-ROS, los dos estimadores y RViz2.
#   ./launch_estimation.sh             con ZUPT (velocidad = 0 al detectar reposo)
#   ./launch_estimation.sh --no-zupt   integracion pura, sin correccion
# Ctrl+C cierra todo.
cd "$(dirname "$0")"
ZUPT=true
[ "$1" = "--no-zupt" ] && ZUPT=false
trap 'kill 0' INT TERM EXIT

ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 &
sleep 2
echo
echo ">>> Reiniciar el ESP32 (boton EN) y dejar el sensor QUIETO unos 10 segundos <<<"
echo

# Metodo 1: filtro complementario sobre datos crudos
python3 imu_state_estimator.py --ros-args -r __node:=complementary_estimator \
    -p child_frame:=imu_link -p zupt:=$ZUPT &
# Metodo 2: orientacion del DMP
python3 imu_state_estimator.py --ros-args -r __node:=dmp_estimator \
    -p use_msg_orientation:=true -p child_frame:=imu_link_dmp -p zupt:=$ZUPT &
# Metodo 3: Madgwick
python3 imu_state_estimator.py --ros-args -r __node:=madgwick_estimator \
    -p filter_type:=madgwick -p child_frame:=imu_link_madg -p zupt:=$ZUPT &

if [ -f imu_tf_view.rviz ]; then rviz2 -d imu_tf_view.rviz & else rviz2 & fi
wait
