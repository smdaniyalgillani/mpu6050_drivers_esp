#include <stdio.h>
#include <driver/i2c_master.h>
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define TAG "MPU6050"

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


i2c_master_dev_handle_t mpu_dev_handle;
i2c_master_bus_handle_t i2c_bus_handle;

void mpu_init() {
    // Wake up sensor by writing 0x00 to power management register
    uint8_t wake_cmd[2] = {0x6B, 0x00};
    ESP_ERROR_CHECK(i2c_master_transmit(mpu_dev_handle, wake_cmd, sizeof(wake_cmd), -1));

    // Optionally, set gyro config (0x1B) and accel config (0x1C)
    // Example: Set gyro to ±500°/s (0x08), accel to ±4g (0x08)
    uint8_t gyro_cfg[2] = {0x1B, 0x08};
    uint8_t accel_cfg[2] = {0x1C, 0x08};
    ESP_ERROR_CHECK(i2c_master_transmit(mpu_dev_handle, gyro_cfg, sizeof(gyro_cfg), -1));
    ESP_ERROR_CHECK(i2c_master_transmit(mpu_dev_handle, accel_cfg, sizeof(accel_cfg), -1));
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

    
    ESP_ERROR_CHECK(i2c_new_master_bus(&esp_i2c_conf, &i2c_bus_handle));
    
    //Configure I2C Device
    i2c_device_config_t mpu6050_config = {
        .device_address = MPU6050_Address,
        .scl_speed_hz = MPU_SPEED,
    };
    

    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &mpu6050_config, &mpu_dev_handle));
    mpu_init(&mpu_dev_handle);
}

void mpu_read_data() {
    uint8_t start_reg = 0x3B;  // Starting at ACCEL_XOUT_H
    uint8_t sensor_data[14];

    ESP_ERROR_CHECK(i2c_master_transmit_receive(mpu_dev_handle, &start_reg, 1, sensor_data, sizeof(sensor_data), -1));
    int16_t raw_ax = (sensor_data[0] << 8) | sensor_data[1];
    int16_t raw_ay = (sensor_data[2] << 8) | sensor_data[3];
    int16_t raw_az = (sensor_data[4] << 8) | sensor_data[5];

    float ax = raw_ax / 8192.0f;
    float ay = raw_ay / 8192.0f;
    float az = raw_az / 8192.0f;

    int16_t temp_raw = (sensor_data[6] << 8) | sensor_data[7];
    int16_t raw_gx = (sensor_data[8] << 8) | sensor_data[9];
    int16_t raw_gy = (sensor_data[10] << 8) | sensor_data[11];
    int16_t raw_gz = (sensor_data[12] << 8) | sensor_data[13];

    float gx = raw_gx / 65.5f;
    float gy = raw_gy / 65.5f;
    float gz = raw_gz / 65.5f;
    float temperature = temp_raw / 340.0 + 36.53;

    printf("Accel: X=%f Y=%f Z=%f | Gyro: X=%f Y=%f Z=%f | Temp: %.2f C\n", ax, ay, az, gx, gy, gz, temperature);
}


void app_main(void)
{
    esp32_mpu_config();
    while (1) {
        mpu_read_data();  // Read and print sensor data
        vTaskDelay(pdMS_TO_TICKS(200));
    }

}
