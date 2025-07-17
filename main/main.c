#include <stdio.h>
#include <driver/i2c_master.h>
#include <esp_err.h>

#define mpu_sck GPIO_NUM_4
#define mpu_sda GPIO_NUM_5
#define mpu_int GPIO_NUM_6
#define MPU6050_Address                     0x68
#define MPU6050_Gyro_Address                0x1B    //Set gyro sensitivity by updating FS_SEL bit
#define MPU6050_Accelerometer_Address       0x1C    //Set gyro sensitivity by updating AFS_SEL bit
#define accelx_high                         0x3B
#define accelx_low                          0x3C
#define accely_high                         0x3D
#define accely_low                          0x3E
#define accelz_high                         0x3F
#define accelz_low                          0x40
#define temp_high                           0x41
#define temp_low                            0x42
#define gyrox_high                          0x43
#define gyrox_low                           0x44
#define gyroy_high                          0x45
#define gyroy_low                           0x46
#define gyroz_high                          0x47
#define gyroz_low                           0x48
#define MPU_SPEED                           400000
#define PWR_MGMT_1                          0x6B

void mpu6050_init(i2c_master_dev_handle_t mpu6050_dev_handle) {
    // Wake up MPU6050 (clear sleep bit)
    uint8_t wakeup_cmd[2] = {PWR_MGMT_1, 0x00};
    ESP_ERROR_CHECK(i2c_master_transmit(mpu6050_dev_handle, wakeup_cmd, 2, -1));
}

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

    i2c_master_bus_handle_t i2c_bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&esp_i2c_conf, &i2c_bus_handle));
    
    //Configure I2C Device
    i2c_device_config_t mpu6050_config = {
        .device_address = MPU6050_Address,
        .scl_speed_hz = MPU_SPEED,
    };
    i2c_master_dev_handle_t mpu6050_dev_handle;

    ESP_ERROR_CHECK(i2c_master_bus_add_device(&i2c_bus_handle, &mpu6050_config, &mpu6050_dev_handle));
    mpu6050_init(mpu6050_dev_handle);
}

void app_main(void)
{
    esp32_mpu_config();

    while(1)
    {

    }


}
