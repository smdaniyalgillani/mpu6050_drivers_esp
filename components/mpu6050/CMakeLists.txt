idf_component_register(
    SRCS "mpu6050_driver.h"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_i2c common
)