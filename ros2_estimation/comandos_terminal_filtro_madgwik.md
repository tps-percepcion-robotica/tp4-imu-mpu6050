# Terminal: estimador con filtro Madgwick
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=33
cd ~/Escritorio/Percepcion_Robotica/imu_mpu6050/ros2_estimation

python3 imu_state_estimator.py --ros-args -r __node:=madgwick_estimator -p filter_type:=madgwick -p child_frame:=imu_link_madg -p zupt:=true