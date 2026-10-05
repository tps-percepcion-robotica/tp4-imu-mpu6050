cmake_minimum_required(VERSION 3.16)
# Usa el componente micro-ROS del proyecto principal
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../esp32_firmware/components/micro_ros_espidf_component")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(microros_raw_imu_publisher)
