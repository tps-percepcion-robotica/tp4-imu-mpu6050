// PASO 3: MPU-6050 con DMP (Digital Motion Processor) + micro-ROS.
// El DMP calcula la orientacion dentro del sensor. Se publica un sensor_msgs/Imu en
// /imu/data_raw con: orientacion (cuaternion del DMP) + aceleracion y giroscopio crudos.
// Asi el mismo topico sirve para los dos metodos:
//   - filtro propio:  usa aceleracion + giroscopio e ignora la orientacion
//   - DMP:            usa la orientacion del mensaje
// Requiere los componentes I2Cdev y MPU6050 de i2cdevlib (ver setup_dmp.sh).
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/i2c.h"

extern "C" {
#include <uros_network_interfaces.h>
}
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/imu.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#endif

#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define PIN_SDA        21
#define PIN_SCL        22
#define ROS_DOMAIN     33

#define RCCHECK(fn) { rcl_ret_t rc_ = fn; if (rc_ != RCL_RET_OK) { \
    printf("Fallo en linea %d: %d. Abortando.\n", __LINE__, (int)rc_); vTaskDelete(NULL); } }

static rcl_publisher_t publisher;
static sensor_msgs__msg__Imu msg;
static char frame_id[] = "imu_link";
static MPU6050 mpu;
static uint8_t fifo_buffer[64];

static void i2c_init(void)
{
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)PIN_SDA;
    conf.scl_io_num = (gpio_num_t)PIN_SCL;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 400000;
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
}

static void micro_ros_task(void *arg)
{
    // ---- Sensor + DMP ----
    i2c_init();
    mpu.initialize();
    uint8_t dmp_status = mpu.dmpInitialize();   // carga el firmware del DMP en el sensor
    if (dmp_status != 0) {
        printf("No se pudo iniciar el DMP (codigo %d). Revisar cableado.\n", dmp_status);
        vTaskDelete(NULL);
    }
    mpu.setDMPEnabled(true);
    const uint16_t packet_size = mpu.dmpGetFIFOPacketSize();

    // Escalas segun los rangos que deja configurados el DMP
    static const float gyro_lsb_per_dps[] = {131.0f, 65.5f, 32.8f, 16.4f};
    const float acc_scale = 9.80665f / (float)(16384 >> mpu.getFullScaleAccelRange());
    const float gyr_scale = 0.017453292f / gyro_lsb_per_dps[mpu.getFullScaleGyroRange() & 0x03];
    printf("DMP OK. Paquete de %d bytes, rango acc %d, rango gyro %d\n",
           packet_size, mpu.getFullScaleAccelRange(), mpu.getFullScaleGyroRange());

    // ---- micro-ROS ----
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP,
                                             CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
#endif

    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "mpu6050_dmp_node", "", &support));

    RCCHECK(rclc_publisher_init_best_effort(
        &publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data_raw"));

    rmw_uros_sync_session(1000);

    memset(&msg, 0, sizeof(msg));
    msg.header.frame_id.data = frame_id;
    msg.header.frame_id.size = strlen(frame_id);
    msg.header.frame_id.capacity = sizeof(frame_id);

    Quaternion q;
    int16_t ax, ay, az, gx, gy, gz;
    mpu.resetFIFO();

    while (1) {
        uint16_t fifo_count = mpu.getFIFOCount();

        if (fifo_count >= 1024) {           // desborde: descartar y empezar de nuevo
            mpu.resetFIFO();
            continue;
        }
        if (fifo_count < packet_size) {     // todavia no hay un paquete completo
            vTaskDelay(1);
            continue;
        }
        // Leer todos los paquetes pendientes y quedarse con el mas reciente
        while (fifo_count >= packet_size) {
            mpu.getFIFOBytes(fifo_buffer, packet_size);
            fifo_count -= packet_size;
        }

        mpu.dmpGetQuaternion(&q, fifo_buffer);
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        int64_t ns = rmw_uros_epoch_nanos();
        msg.header.stamp.sec = (int32_t)(ns / 1000000000LL);
        msg.header.stamp.nanosec = (uint32_t)(ns % 1000000000LL);

        msg.orientation.w = q.w;
        msg.orientation.x = q.x;
        msg.orientation.y = q.y;
        msg.orientation.z = q.z;
        msg.linear_acceleration.x = ax * acc_scale;
        msg.linear_acceleration.y = ay * acc_scale;
        msg.linear_acceleration.z = az * acc_scale;
        msg.angular_velocity.x = gx * gyr_scale;
        msg.angular_velocity.y = gy * gyr_scale;
        msg.angular_velocity.z = gz * gyr_scale;

        rcl_ret_t rc = rcl_publish(&publisher, &msg, NULL);
        (void)rc;
    }
}

extern "C" void app_main(void)
{
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif
    xTaskCreate(micro_ros_task, "uros_task", 16000, NULL, 5, NULL);
}
