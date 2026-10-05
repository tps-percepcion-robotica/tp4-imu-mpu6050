// PASO 1: probar el MPU-6050 sin ROS.
// Usar como main/main.c en un proyecto ESP-IDF comun, junto a mpu6050.h.
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mpu6050.h"

void app_main(void)
{
    uint8_t id = 0;
    esp_err_t err = mpu_init(&id);
    if (err != ESP_OK) {
        printf("Error al iniciar el MPU-6050: %s. Revisar cableado y direccion I2C.\n",
               esp_err_to_name(err));
        return;
    }
    printf("WHO_AM_I = 0x%02X (se espera 0x68)\n", id);

    mpu_data_t d;
    while (1) {
        if (mpu_read_data(&d) == ESP_OK) {
            printf("acc [m/s2]: %7.2f %7.2f %7.2f | gyro [rad/s]: %7.3f %7.3f %7.3f | T: %.1f C\n",
                   d.ax, d.ay, d.az, d.gx, d.gy, d.gz, d.temp_c);
        } else {
            printf("Fallo de lectura I2C\n");
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
