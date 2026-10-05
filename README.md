# IMU MPU-6050 con micro-ROS y ROS2

Estimacion de actitud, velocidad y posicion con un MPU-6050, por dos metodos:
1. Filtro complementario sobre los datos inerciales crudos.
2. Orientacion calculada por el DMP (Digital Motion Processor) del sensor.

## Estructura
    esp32_firmware/     Proyecto ESP-IDF (ESP32). Publica sensor_msgs/Imu en
                        /imu/data_raw a ~100 Hz: cuaternion del DMP + aceleracion
                        y velocidad angular crudas.
    ros2_estimation/    imu_state_estimator.py: nodo que estima actitud, velocidad
                        y posicion, publica TF y odometria, y grafica en vivo.
                        launch_estimation.sh: lanza agente, estimadores y RViz2.
    test_firmware/      Firmwares de prueba (bring-up I2C, publicador solo crudos).

## Cableado
| MPU-6050 | ESP32  |
|----------|--------|
| VCC      | 3V3    |
| GND      | GND    |
| SDA      | GPIO21 |
| SCL      | GPIO22 |

## Firmware (solo cuando cambia el codigo o la red)
    cd esp32_firmware
    idf.py menuconfig        # micro-ROS Settings: WiFi, IP de la PC (hostname -I), puerto 8888
    idf.py build flash monitor

El dominio de ROS2 esta fijado en 33 (ROS_DOMAIN en main/main.cpp).

## Uso
    cd ros2_estimation
    ./launch_estimation.sh              # o ./launch_estimation.sh --no-zupt

Despues reiniciar el ESP32 con el boton EN y dejar el sensor quieto ~10 s.

Nodos y salidas:
| Nodo                      | Metodo                | Frame TF       | Odometria                       |
|---------------------------|-----------------------|----------------|---------------------------------|
| /complementary_estimator  | filtro complementario | imu_link       | /complementary_estimator/odom   |
| /dmp_estimator            | DMP                   | imu_link_dmp   | /dmp_estimator/odom             |

En RViz2: Fixed Frame = odom y display TF. Guardando esa configuracion como
ros2_estimation/imu_tf_view.rviz se carga sola la proxima vez.
En las ventanas de graficos, la tecla R reinicia velocidad y posicion.

## Si no aparece el topico
1. Que el agente este corriendo antes de reiniciar el ESP32.
2. Que la IP de la PC siga siendo la configurada en menuconfig (hostname -I).
3. Que la terminal use ROS_DOMAIN_ID=33.
4. Cableado I2C: probar con test_firmware/mpu6050_i2c_bringup.

## Instalacion de dependencias (despues de clonar)

Los componentes de terceros no estan incluidos en el repositorio. Con ROS2 y
ESP-IDF cargados en la terminal:

    git clone https://github.com/tps-percepcion-robotica/tp4-imu-mpu6050.git
    cd tp4-imu-mpu6050/esp32_firmware
    mkdir -p components

1. micro-ROS para ESP-IDF (usar la rama de la distro de ROS2 instalada):

        git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_espidf_component.git components/micro_ros_espidf_component
        pip3 install catkin_pkg lark-parser colcon-common-extensions

2. Driver del MPU-6050 con soporte de DMP (i2cdevlib):

        git clone --depth 1 https://github.com/jrowberg/i2cdevlib.git /tmp/i2cdevlib
        cp -r /tmp/i2cdevlib/ESP32_ESP-IDF/components/I2Cdev components/
        cp -r /tmp/i2cdevlib/ESP32_ESP-IDF/components/MPU6050 components/

3. CMakeLists de esos dos componentes para ESP-IDF 5.x:

        printf 'file(GLOB srcs "*.cpp")\nidf_component_register(SRCS ${srcs} INCLUDE_DIRS "." REQUIRES driver)\ntarget_compile_options(${COMPONENT_LIB} PRIVATE -w)\n' > components/I2Cdev/CMakeLists.txt
        printf 'file(GLOB srcs "*.cpp")\nidf_component_register(SRCS ${srcs} INCLUDE_DIRS "." REQUIRES I2Cdev driver)\ntarget_compile_options(${COMPONENT_LIB} PRIVATE -w)\n' > components/MPU6050/CMakeLists.txt

4. Configurar, compilar y flashear:

        idf.py set-target esp32
        idf.py menuconfig      # micro-ROS Settings: SSID, contrasena, IP de la PC, puerto 8888
        idf.py build flash monitor

La primera compilacion tarda entre 10 y 30 minutos porque construye la
libreria de micro-ROS.
