// PASO 2: publicar el MPU-6050 como sensor_msgs/Imu con micro-ROS (WiFi/UDP).
// Usar como main/main.c en un proyecto con micro_ros_espidf_component en components/,
// junto a mpu6050.h. Topico: /imu/data_raw a 100 Hz, QoS best effort.
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/imu.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#endif

#include "mpu6050.h"

#define PERIODO_MS 10   // 100 Hz

#define RCCHECK(fn) { rcl_ret_t rc_ = fn; if (rc_ != RCL_RET_OK) { \
    printf("Fallo en linea %d: %d. Abortando.\n", __LINE__, (int)rc_); vTaskDelete(NULL); } }

static rcl_publisher_t publisher;
static sensor_msgs__msg__Imu msg;
static char frame_id[] = "imu_link";

static void micro_ros_task(void *arg)
{
    // ---- Sensor ----
    uint8_t id = 0;
    if (mpu_init(&id) != ESP_OK) {
        printf("No se pudo iniciar el MPU-6050\n");
        vTaskDelete(NULL);
    }
    printf("MPU-6050 OK, WHO_AM_I = 0x%02X\n", id);

    // ---- micro-ROS ----
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_options, 33));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP,
                                             CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
#endif

    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "mpu6050_node", "", &support));

    RCCHECK(rclc_publisher_init_best_effort(
        &publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data_raw"));

    // Sincronizar el reloj con el agente para tener timestamps validos
    rmw_uros_sync_session(1000);

    // ---- Mensaje ----
    memset(&msg, 0, sizeof(msg));
    msg.header.frame_id.data = frame_id;
    msg.header.frame_id.size = strlen(frame_id);
    msg.header.frame_id.capacity = sizeof(frame_id);
    msg.orientation.w = 1.0;
    msg.orientation_covariance[0] = -1.0;   // -1 = orientacion no disponible

    mpu_data_t d;
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        if (mpu_read_data(&d) == ESP_OK) {
            int64_t ns = rmw_uros_epoch_nanos();
            msg.header.stamp.sec = (int32_t)(ns / 1000000000LL);
            msg.header.stamp.nanosec = (uint32_t)(ns % 1000000000LL);

            msg.linear_acceleration.x = d.ax;
            msg.linear_acceleration.y = d.ay;
            msg.linear_acceleration.z = d.az;
            msg.angular_velocity.x = d.gx;
            msg.angular_velocity.y = d.gy;
            msg.angular_velocity.z = d.gz;

            rcl_ret_t rc = rcl_publish(&publisher, &msg, NULL);
            (void)rc;
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIODO_MS));
    }
}

void app_main(void)
{
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    xTaskCreate(micro_ros_task, "uros_task", 16000, NULL,
                5, NULL);
}
