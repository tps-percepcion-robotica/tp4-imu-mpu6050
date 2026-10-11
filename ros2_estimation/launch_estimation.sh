#!/usr/bin/env bash
# Lanza el pipeline de la PC: los tres estimadores, la ventana que los compara,
# el comparador contra el encoder y RViz2.
# El agente micro-ROS se lanza aparte, en otra terminal:
#   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
#   ./launch_estimation.sh             con ZUPT (velocidad = 0 al detectar reposo)
#   ./launch_estimation.sh --no-zupt   integracion pura, sin correccion
# Ctrl+C cierra todo.
cd "$(dirname "$0")"
ZUPT=true
[ "$1" = "--no-zupt" ] && ZUPT=false
# Ventanas individuales de cada estimador (true para volver a abrirlas)
VENTANAS=false
trap 'kill 0' INT TERM EXIT

# El ESP32 se conecta solo al agente (no hace falta apretar EN).
# Cada estimador calibra con las primeras muestras: dejar el sensor quieto ~2 s.

# Cada frame se dibuja en un punto fijo distinto en RViz (tf_offset), solo para poder ver
# la actitud de los tres por separado; no afecta ~/odom ni las comparaciones.
# Metodo 1: filtro complementario sobre datos crudos
python3 imu_state_estimator.py --ros-args -r __node:=complementary_estimator \
    -p child_frame:=imu_link -p zupt:=$ZUPT -p plot:=$VENTANAS -p tf_offset:="[-0.3,0.0,0.0]" &
# Metodo 2: orientacion del DMP
python3 imu_state_estimator.py --ros-args -r __node:=dmp_estimator \
    -p use_msg_orientation:=true -p child_frame:=imu_link_dmp -p zupt:=$ZUPT -p plot:=$VENTANAS \
    -p tf_offset:="[0.3,0.0,0.0]" &
# Metodo 3: Madgwick
python3 imu_state_estimator.py --ros-args -r __node:=madgwick_estimator \
    -p filter_type:=madgwick -p child_frame:=imu_link_madg -p zupt:=$ZUPT -p plot:=$VENTANAS \
    -p tf_offset:="[0.0,0.3,0.0]" &

# Una sola ventana comparando los tres metodos (tecla R: recalibrar los tres)
python3 comparar_metodos.py &
# Comparacion contra el encoder AS5600 (referencia del angulo del disco)
python3 comparar_encoder.py &

if [ -f imu_tf_view.rviz ]; then rviz2 -d imu_tf_view.rviz & else rviz2 & fi
wait
