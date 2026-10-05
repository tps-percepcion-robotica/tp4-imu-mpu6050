# Firmwares de prueba

- `mpu6050_i2c_bringup/`: lee el MPU-6050 por I2C e imprime por serie, sin ROS.
  Sirve para verificar cableado y direccion I2C.
- `microros_raw_imu_publisher/`: publica solo aceleracion y giroscopio crudos
  (sin DMP), con el giroscopio en +-250 grados/s.
