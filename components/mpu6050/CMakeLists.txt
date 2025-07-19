idf_component_register(
    SRCS "mpu6050_conf.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_i2c common
)