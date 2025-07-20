idf_component_register(SRCS "main.c"
                    PRIV_REQUIRES esp_driver_i2c mpu6050 comms_i2c
                    INCLUDE_DIRS ".")
