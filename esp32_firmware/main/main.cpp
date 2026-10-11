// PASO 3: MPU-6050 con DMP (Digital Motion Processor) + micro-ROS.
// El DMP calcula la orientacion dentro del sensor. Se publica un sensor_msgs/Imu en
// /imu/data_raw con: orientacion (cuaternion del DMP) + aceleracion y giroscopio crudos.
// Asi el mismo topico sirve para los dos metodos:
//   - filtro propio:  usa aceleracion + giroscopio e ignora la orientacion
//   - DMP:            usa la orientacion del mensaje
// Requiere los componentes I2Cdev y MPU6050 de i2cdevlib (ver setup_dmp.sh).
// Espera al agente micro-ROS y se reconecta solo si se pierde: no hace falta apretar EN.
// Funciona por WiFi o por UART (cable USB) segun lo elegido en menuconfig.
// Ademas lee el encoder magnetico AS5600 (mismo bus I2C) y publica /encoder/angle con el
// MISMO timestamp que la IMU, para usarlo como referencia del angulo del disco.
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "driver/i2c.h"

extern "C" {
#include <uros_network_interfaces.h>
}
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/vector3_stamped.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#endif

#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Transporte por UART (cable USB) en vez de WiFi: se elige en menuconfig
// (micro-ROS Settings -> "Micro XRCE-DDS over UART") junto con app-colcon.meta.
#ifdef CONFIG_MICRO_ROS_ESP_UART_TRANSPORT
#ifndef RMW_UXRCE_TRANSPORT_CUSTOM
#error "La libreria micro-ROS sigue compilada para WiFi: correr 'idf.py clean-microros' y despues 'idf.py build'"
#endif
#include "driver/uart.h"
#include "esp32_serial_transport.h"
// El cable lo usa micro-ROS: cualquier printf se mezclaria con los datos
#define printf(...) ((void)0)
// El cable lleva menos que el WiFi: se publica 1 de cada 2 paquetes del DMP (200 -> 100 Hz)
#define PUBLISH_DIVIDER 2
#else
#define PUBLISH_DIVIDER 1
#endif

#define PIN_SDA        21
#define PIN_SCL        22
#define ROS_DOMAIN     22

// AS5600: comparte el bus con el MPU-6050 (0x68). Se usa el driver I2C viejo (driver/i2c.h)
// porque i2cdevlib usa ese; ESP-IDF 5 no permite mezclarlo con el nuevo (i2c_master.h).
#define AS5600_ADDR           0x36
#define AS5600_REG_STATUS     0x0B
#define AS5600_REG_RAW_ANGLE  0x0C   // 12 bits, 0-4095 = 0-360 grados
#define AS5600_STATUS_MD      0x20   // iman detectado
// Timeout de cada lectura: 2 ticks (20 ms con CONFIG_FREERTOS_HZ=100). Con menos de un
// tick, pdMS_TO_TICKS redondea a 0 y la lectura falla siempre.
#define AS5600_TIMEOUT        2

// Cada cuanto se verifica que el agente siga vivo, y cuantos pings fallidos seguidos
// se toleran antes de considerar la conexion perdida.
#define PING_PERIOD_MS 1000
#define PING_FAILS_MAX 3
#define PING_TIMEOUT_MS 100     // por intento; cada verificacion hace 3 intentos

#define RCCHECK(fn) { rcl_ret_t rc_ = fn; if (rc_ != RCL_RET_OK) { \
    printf("Fallo en linea %d: %d.\n", __LINE__, (int)rc_); return false; } }

static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rcl_publisher_t publisher;
static rcl_publisher_t enc_publisher;
static sensor_msgs__msg__Imu msg;
static geometry_msgs__msg__Vector3Stamped enc_msg;   // vector.x = angulo [rad], vector.y = cuenta cruda
static char frame_id[] = "imu_link";
static char enc_frame_id[] = "encoder";
static bool encoder_ok = false;
static MPU6050 mpu;
static uint8_t fifo_buffer[64];
// Que entidades llegaron a crearse (para destruir solo esas si algo falla a mitad)
static bool support_ok, node_ok, publisher_ok, enc_publisher_ok;

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

// Lee un registro de 12 bits del AS5600 (2 bytes, MSB primero)
static bool as5600_read(uint8_t reg, uint16_t *value)
{
    uint8_t data[2];
    if (i2c_master_write_read_device(I2C_NUM_0, AS5600_ADDR, &reg, 1, data, 2, AS5600_TIMEOUT) != ESP_OK)
        return false;
    *value = ((data[0] << 8) | data[1]) & 0x0FFF;
    return true;
}

static void as5600_init(void)
{
    uint16_t status;
    bool ok = false;
    for (int i = 0; i < 3 && !ok; i++)      // un par de reintentos por si el bus recien arranca
        ok = as5600_read(AS5600_REG_STATUS, &status);
    if (!ok) {
        printf("AS5600 no responde en 0x%02X: se publica solo la IMU.\n", AS5600_ADDR);
        return;
    }
    encoder_ok = true;
    if (status & AS5600_STATUS_MD)
        printf("AS5600 OK, iman detectado.\n");
    else
        printf("AS5600 responde pero NO detecta el iman (STATUS=0x%03X).\n", status);
}

// Espera hasta que el agente micro-ROS responda. Asi no importa si el ESP32 arranca
// antes o despues que el agente: no hace falta apretar EN.
static void wait_for_agent(void)
{
    rcl_init_options_t opts = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&opts, allocator);
#ifdef CONFIG_MICRO_ROS_ESP_UART_TRANSPORT
    while (rmw_uros_ping_agent(200, 1) != RMW_RET_OK) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
#elif defined(CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE)
    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&opts);
    rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options);
    printf("Esperando al agente micro-ROS en %s:%s ...\n", CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT);
    while (rmw_uros_ping_agent_options(200, 1, rmw_options) != RMW_RET_OK) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
#endif
    rcl_init_options_fini(&opts);
    printf("Agente encontrado.\n");
}

static bool create_entities(void)
{
    support_ok = node_ok = publisher_ok = enc_publisher_ok = false;
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN));
#if defined(CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE) && !defined(CONFIG_MICRO_ROS_ESP_UART_TRANSPORT)
    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP,
                                             CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
#endif
    rcl_ret_t rc = rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
    rcl_init_options_fini(&init_options);
    RCCHECK(rc);
    support_ok = true;

    node = rcl_get_zero_initialized_node();
    RCCHECK(rclc_node_init_default(&node, "mpu6050_dmp_node", "", &support));
    node_ok = true;

    publisher = rcl_get_zero_initialized_publisher();
    RCCHECK(rclc_publisher_init_best_effort(
        &publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data_raw"));
    publisher_ok = true;

    if (encoder_ok) {
        enc_publisher = rcl_get_zero_initialized_publisher();
        RCCHECK(rclc_publisher_init_best_effort(
            &enc_publisher, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3Stamped),
            "encoder/angle"));
        enc_publisher_ok = true;
    }

    rmw_uros_sync_session(1000);    // sincroniza el reloj con la PC para los timestamps
    printf("Conectado. Publicando en /imu/data_raw (dominio %d).\n", ROS_DOMAIN);
    return true;
}

static void destroy_entities(void)
{
    if (!support_ok)
        return;
    // El agente puede no responder: no esperar su confirmacion al destruir cada entidad
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
    if (enc_publisher_ok) (void)rcl_publisher_fini(&enc_publisher, &node);
    if (publisher_ok) (void)rcl_publisher_fini(&publisher, &node);
    if (node_ok)      (void)rcl_node_fini(&node);
    (void)rclc_support_fini(&support);
    support_ok = node_ok = publisher_ok = enc_publisher_ok = false;
}

// Lee el paquete mas reciente del DMP y lo publica. Devuelve false si no habia paquete.
static bool publish_imu(uint16_t packet_size, float acc_scale, float gyr_scale)
{
    uint16_t fifo_count = mpu.getFIFOCount();
    if (fifo_count >= 1024) {           // desborde: descartar y empezar de nuevo
        mpu.resetFIFO();
        return false;
    }
    if (fifo_count < packet_size)       // todavia no hay un paquete completo
        return false;
    // Leer todos los paquetes pendientes y quedarse con el mas reciente
    while (fifo_count >= packet_size) {
        mpu.getFIFOBytes(fifo_buffer, packet_size);
        fifo_count -= packet_size;
    }
    static uint32_t n_paquetes = 0;
    if (++n_paquetes % PUBLISH_DIVIDER)
        return true;                    // paquete leido pero no publicado (ver PUBLISH_DIVIDER)

    Quaternion q;
    int16_t ax, ay, az, gx, gy, gz;
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

    (void)rcl_publish(&publisher, &msg, NULL);

    // Encoder leido en el mismo instante y con el mismo timestamp que la IMU
    uint16_t raw;
    if (enc_publisher_ok && as5600_read(AS5600_REG_RAW_ANGLE, &raw)) {
        enc_msg.header.stamp = msg.header.stamp;
        enc_msg.vector.x = raw * (2.0 * M_PI / 4096.0);
        enc_msg.vector.y = raw;
        (void)rcl_publish(&enc_publisher, &enc_msg, NULL);
    }
    return true;
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

    memset(&msg, 0, sizeof(msg));
    msg.header.frame_id.data = frame_id;
    msg.header.frame_id.size = strlen(frame_id);
    msg.header.frame_id.capacity = sizeof(frame_id);
    memset(&enc_msg, 0, sizeof(enc_msg));
    enc_msg.header.frame_id.data = enc_frame_id;
    enc_msg.header.frame_id.size = strlen(enc_frame_id);
    enc_msg.header.frame_id.capacity = sizeof(enc_frame_id);

    as5600_init();

    allocator = rcl_get_default_allocator();

    // ---- micro-ROS: conectar, publicar, y si se pierde el agente volver a esperar ----
    while (1) {
        wait_for_agent();
        if (!create_entities()) {
            destroy_entities();
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        mpu.resetFIFO();                // descartar lo acumulado mientras no habia conexion

        TickType_t last_ping = xTaskGetTickCount();
        int ping_fails = 0;
        while (ping_fails < PING_FAILS_MAX) {
            if (!publish_imu(packet_size, acc_scale, gyr_scale))
                vTaskDelay(1);
            if (xTaskGetTickCount() - last_ping >= pdMS_TO_TICKS(PING_PERIOD_MS)) {
                last_ping = xTaskGetTickCount();
                ping_fails = (rmw_uros_ping_agent(PING_TIMEOUT_MS, 3) == RMW_RET_OK) ? 0 : ping_fails + 1;
            }
        }
        printf("Se perdio la conexion con el agente. Reconectando...\n");
        destroy_entities();
    }
}

extern "C" void app_main(void)
{
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN)
    // Sin ahorro de energia: por defecto la radio duerme entre beacons y las respuestas
    // del agente llegan con 100-300 ms de retraso (fallan los pings y se pierden paquetes)
    esp_wifi_set_ps(WIFI_PS_NONE);
#endif
#ifdef CONFIG_MICRO_ROS_ESP_UART_TRANSPORT
    static size_t uart_port = UART_NUM_0;     // el UART del cable USB
    rmw_uros_set_custom_transport(true, (void *)&uart_port,
                                  esp32_serial_open, esp32_serial_close,
                                  esp32_serial_write, esp32_serial_read);
#endif
    xTaskCreate(micro_ros_task, "uros_task", 16000, NULL, 5, NULL);
}
