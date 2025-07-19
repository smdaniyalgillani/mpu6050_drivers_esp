#include "common_includes.h"
#include "mpu6050_conf.hpp"



float mpu6050::mpu6050_get_accel_scale(accel_range_t accel_range) {
    switch (accel_range) {
        case ACCEL_2G:  return 16384.0f;
        case ACCEL_4G:  return 8192.0f;
        case ACCEL_8G:  return 4096.0f;
        case ACCEL_16G: return 2048.0f;
        default:        return 8192.0f; // fallback
    }
}

float mpu6050::mpu6050_get_gyro_scale(gyro_range_t gyro_range) {
    switch (gyro_range) {
        case GYRO_250DPS:  return 131.0f;
        case GYRO_500DPS:  return 65.5f;
        case GYRO_1000DPS: return 32.8f;
        case GYRO_2000DPS: return 16.4f;
        default:           return 65.5f; // fallback
    }
}


void mpu6050::config(i2c_master_dev_handle_t dev, accel_range_t accel_sel_, gyro_range_t gyro_sel_) {
    mpu_dev_handle = dev;
    gyro_sel = gyro_sel_;
    accel_sel = accel_sel_;

    uint8_t gyro_cfg[2] = {0x1B, gyro_sel};
    uint8_t accel_cfg[2] = {0x1C, accel_sel};
    ESP_ERROR_CHECK(i2c_master_transmit(mpu_dev_handle, gyro_cfg, sizeof(gyro_cfg), -1));
    ESP_ERROR_CHECK(i2c_master_transmit(mpu_dev_handle, accel_cfg, sizeof(accel_cfg), -1));

}

void mpu6050::start()
{
    // Wake up sensor from sleep writing 0x00 to power management register
    uint8_t wake_cmd[2] = {0x6B, 0x00};
    ESP_ERROR_CHECK(i2c_master_transmit(mpu_dev_handle, wake_cmd, sizeof(wake_cmd), -1));

}


void mpu6050::read(sensor_t sensor_type) {
    uint8_t start_reg = 0x3B;  // Starting at ACCEL_XOUT_H
    uint8_t sensor_data[14];

    ESP_ERROR_CHECK(i2c_master_transmit_receive(mpu_dev_handle, &start_reg, 1, sensor_data, sizeof(sensor_data), -1));
    int16_t raw_ax = (sensor_data[0] << 8) | sensor_data[1];
    int16_t raw_ay = (sensor_data[2] << 8) | sensor_data[3];
    int16_t raw_az = (sensor_data[4] << 8) | sensor_data[5];

    int16_t temp_raw = (sensor_data[6] << 8) | sensor_data[7];
    int16_t raw_gx = (sensor_data[8] << 8) | sensor_data[9];
    int16_t raw_gy = (sensor_data[10] << 8) | sensor_data[11];
    int16_t raw_gz = (sensor_data[12] << 8) | sensor_data[13];

    float accel_scale = mpu6050_get_accel_scale(accel_sel);
    float gyro_scale = mpu6050_get_gyro_scale(gyro_sel);

    float ax = raw_ax / accel_scale;
    float ay = raw_ay / accel_scale;
    float az = raw_az / accel_scale;
    float gx = raw_gx / gyro_scale;
    float gy = raw_gy / gyro_scale;
    float gz = raw_gz / gyro_scale;
    float temperature = temp_raw /340.0 + 36.53;
    switch (sensor_type)
    {
    case ACCELEROMETER:
    printf("Accel: X=%f Y=%f Z=%f \n", ax, ay, az);
        break;
    case ACCEL_GYRO:
    printf("Accel: X=%f Y=%f Z=%f | Gyro: X=%f Y=%f Z=%f \n", ax, ay, az, gx, gy, gz);
        break;
    case GYROSCOPE:
    printf("Gyro: X=%f Y=%f Z=%f\n", gx, gy, gz);
        break;
    case ALL:
        printf("Accel: X=%f Y=%f Z=%f | Gyro: X=%f Y=%f Z=%f | Temp: %.2f C\n", ax, ay, az, gx, gy, gz, temperature);
        break;
        
    default:
        printf("Accel: X=%f Y=%f Z=%f | Gyro: X=%f Y=%f Z=%f | Temp: %.2f C\n", ax, ay, az, gx, gy, gz, temperature);
        break;
    }
    
}