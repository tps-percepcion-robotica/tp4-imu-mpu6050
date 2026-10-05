// Driver minimo del MPU-6050 para ESP-IDF (driver I2C clasico, driver/i2c.h).
// Copiar a la carpeta main/ del proyecto junto al main.c.
#pragma once

#include <stdint.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "driver/i2c.h"
#include "esp_err.h"

// ---- Conexion (cambiar si usas otros pines) ----
#define MPU_I2C_PORT   I2C_NUM_0
#define MPU_PIN_SDA    21
#define MPU_PIN_SCL    22
#define MPU_ADDR       0x68      // 0x69 si AD0 esta a 3V3

// ---- Registros ----
#define MPU_REG_SMPLRT_DIV    0x19
#define MPU_REG_CONFIG        0x1A
#define MPU_REG_GYRO_CONFIG   0x1B
#define MPU_REG_ACCEL_CONFIG  0x1C
#define MPU_REG_ACCEL_XOUT_H  0x3B
#define MPU_REG_PWR_MGMT_1    0x6B
#define MPU_REG_WHO_AM_I      0x75

// ---- Escalas para +-2 g y +-250 grados/s ----
#define MPU_ACCEL_LSB_PER_G    16384.0f
#define MPU_GYRO_LSB_PER_DPS   131.0f
#define MPU_G_MS2              9.80665f
#define MPU_DEG2RAD            0.017453292519943f

typedef struct {
    float ax, ay, az;   // m/s^2
    float gx, gy, gz;   // rad/s
    float temp_c;       // grados C
} mpu_data_t;

static inline esp_err_t mpu_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_write_to_device(MPU_I2C_PORT, MPU_ADDR, buf, 2, pdMS_TO_TICKS(100));
}

static inline esp_err_t mpu_read(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(MPU_I2C_PORT, MPU_ADDR, &reg, 1, data, len, pdMS_TO_TICKS(100));
}

// Inicializa el bus I2C y configura el sensor. Devuelve en *who_am_i el ID leido.
static inline esp_err_t mpu_init(uint8_t *who_am_i)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = MPU_PIN_SDA,
        .scl_io_num = MPU_PIN_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };
    esp_err_t err = i2c_param_config(MPU_I2C_PORT, &conf);
    if (err != ESP_OK) return err;
    err = i2c_driver_install(MPU_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) return err;

    err = mpu_read(MPU_REG_WHO_AM_I, who_am_i, 1);
    if (err != ESP_OK) return err;

    // Despertar el sensor y usar el giroscopio X como reloj
    err = mpu_write(MPU_REG_PWR_MGMT_1, 0x01);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(100));

    mpu_write(MPU_REG_CONFIG, 0x03);        // filtro pasa bajos digital ~44 Hz
    mpu_write(MPU_REG_SMPLRT_DIV, 9);       // 1 kHz / (1 + 9) = 100 Hz
    mpu_write(MPU_REG_GYRO_CONFIG, 0x00);   // +-250 grados/s
    return mpu_write(MPU_REG_ACCEL_CONFIG, 0x00);  // +-2 g
}

// Lee acelerometro, temperatura y giroscopio en una sola transaccion.
static inline esp_err_t mpu_read_data(mpu_data_t *d)
{
    uint8_t b[14];
    esp_err_t err = mpu_read(MPU_REG_ACCEL_XOUT_H, b, sizeof(b));
    if (err != ESP_OK) return err;

    int16_t ax = (int16_t)((b[0]  << 8) | b[1]);
    int16_t ay = (int16_t)((b[2]  << 8) | b[3]);
    int16_t az = (int16_t)((b[4]  << 8) | b[5]);
    int16_t t  = (int16_t)((b[6]  << 8) | b[7]);
    int16_t gx = (int16_t)((b[8]  << 8) | b[9]);
    int16_t gy = (int16_t)((b[10] << 8) | b[11]);
    int16_t gz = (int16_t)((b[12] << 8) | b[13]);

    d->ax = ax / MPU_ACCEL_LSB_PER_G * MPU_G_MS2;
    d->ay = ay / MPU_ACCEL_LSB_PER_G * MPU_G_MS2;
    d->az = az / MPU_ACCEL_LSB_PER_G * MPU_G_MS2;
    d->gx = gx / MPU_GYRO_LSB_PER_DPS * MPU_DEG2RAD;
    d->gy = gy / MPU_GYRO_LSB_PER_DPS * MPU_DEG2RAD;
    d->gz = gz / MPU_GYRO_LSB_PER_DPS * MPU_DEG2RAD;
    d->temp_c = t / 340.0f + 36.53f;
    return ESP_OK;
}
