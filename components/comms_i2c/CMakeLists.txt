idf_component_register(
    SRCS "comms_i2c.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_i2c common
)