#include <stdio.h>
#include <driver/i2c_master.h>
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "mpu6050_driver.h"
#include "comms_i2c.h"

#define TAG "MPU6050"

#define mpu_sck GPIO_NUM_4
#define mpu_sda GPIO_NUM_5
#define mpu_int GPIO_NUM_6

i2c_master_bus_handle_t *i2c_bus_handle;
i2c_master_dev_handle_t *mpu_dev_handle;
mpu6050_dev_t mpu6050;
i2c_bus_controller i2c_control;

void app_main(void)
{

    i2c_bus_init(&i2c_control, i2c_bus_handle, 1, mpu_sck, mpu_sda, true);
    i2c_add_new_device(&i2c_control, mpu_dev_handle, 0x68, 400000);
    mpu_config(&mpu6050, mpu_dev_handle, ACCEL_16G, GYRO_2000DPS);
    mpu_start();
    while (1) {
        mpu_read(&mpu6050, ACCELEROMETER);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
