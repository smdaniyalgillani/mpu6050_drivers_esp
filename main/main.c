#include <stdio.h>
#include <driver/i2c_master.h>
#include <esp_err.h>

#define mpu_sck GPIO_NUM_4
#define mpu_sda GPIO_NUM_5
#define mpu_int GPIO_NUM_6
#define MPU6050_Address                     0x68
#define MPU6050_Gyro_Address                0x1B
#define MPU6050_Accelerometer_Address       0x1B
#define accelx_high                         0x3B
#define accelx_low                          0x3C
#define accely_high                         0x3D
#define accely_low                          0x3E
#define accelz_high                         0x3F
#define accelz_low                          0x40
#define gyrox_high                          0x43
#define gyrox_low                           0x44
#define gyroy_high                          0x45
#define gyroy_low                           0x46
#define gyroz_high                          0x47
#define gyroz_low                           0x48

void i2c_device_config()
{

    i2c_master_bus_config_t i2c_mpu6050_conf = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = 1,
        .scl_io_num = mpu_sck,
        .sda_io_num = mpu_sda,
        // .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,    
        };

    i2c_master_bus_handle_t i2c_bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mpu6050_conf, &i2c_bus_handle));
}

void app_main(void)
{
}
