#include <stdio.h>
#include <driver/i2c_master.h>
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "mpu6050_conf.hpp"

#define TAG "MPU6050"

#define mpu_sck GPIO_NUM_4
#define mpu_sda GPIO_NUM_5
#define mpu_int GPIO_NUM_6

i2c_master_bus_handle_t i2c_bus_handle;
i2c_master_dev_handle_t mpu_dev_handle;

void esp32_mpu_config()
{
    //Setup I2C Bus
    i2c_master_bus_config_t esp_i2c_conf = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = 1,
        .scl_io_num = mpu_sck,
        .sda_io_num = mpu_sda,
        // .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,    
        };

    
    ESP_ERROR_CHECK(i2c_new_master_bus(&esp_i2c_conf, &i2c_bus_handle));
    
    //Configure I2C Device
    i2c_device_config_t mpu6050_config = {
        .device_address = MPU6050_Address,
        .scl_speed_hz = MPU_SPEED,
    };
    

    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &mpu6050_config, &mpu_dev_handle));
}

extern "C"
{
    void app_main(void)
    {
        // esp32_mpu_config();
        // while (1) {
        //     mpu_read_data();  // Read and print sensor data
        //     vTaskDelay(pdMS_TO_TICKS(1000));
        // }

    }
}