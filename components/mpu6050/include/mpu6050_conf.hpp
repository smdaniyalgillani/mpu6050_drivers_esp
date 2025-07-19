#pragma once
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

typedef enum {
    ACCEL_2G  = 0x00,
    ACCEL_4G  = 0x08,
    ACCEL_8G  = 0x10,
    ACCEL_16G = 0x18
} accel_range_t;

typedef enum {
    GYRO_250DPS  = 0x00,
    GYRO_500DPS  = 0x08,
    GYRO_1000DPS = 0x10,
    GYRO_2000DPS = 0x18
} gyro_range_t;

typedef enum {
    ACCELEROMETER  = 0,
    GYROSCOPE  = 1,
    ACCEL_GYRO = 2,
    TEMPERATURE = 3,
    ALL = 4
} sensor_t;

class mpu6050 {
    void config(i2c_master_dev_handle_t dev, accel_range_t accel_sel, gyro_range_t gyro_sel);
    float mpu6050_get_accel_scale(accel_range_t accel_range);
    float mpu6050_get_gyro_scale(gyro_range_t gyro_range);
    
    void start();
    
    void read(sensor_t sensor_type);

    private:
    i2c_master_dev_handle_t mpu_dev_handle;
    gyro_range_t gyro_sel;
    accel_range_t accel_sel;

};

