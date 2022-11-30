#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"

#ifndef MPU9250_H
#define MPU9250_H

/////////////////////////////////////////////////////////////////////
// Author: Jeferson Menegazzo                                        
// Year: 2020                                                        
// License: CC BY-NC-ND 4.0  
/////////////////////////////////////////////////////////////////////
// Adapted for c++/c by Tommaso Bugliesi 2022                 
/////////////////////////////////////////////////////////////////////

// Pico GPIO ports

#define PIN_MISO 4
#define PIN_CS 5
#define PIN_SCK 6
#define PIN_MOSI 7

#define SPI_PORT spi0
#define READ_BIT 0x80

#define PI 3.1415

// Register Map for Gyroscope and Accelerometer - MPU 9250

// Registers
#define PWR_MGMT_1 0x6B

// Configuration
#define CONFIG 0x1A

// Gyroscope Configuration
#define GYRO_CONFIG 0x1B

// Accelerometer Configuration
#define ACCEL_CONFIG 0x1C

// Accelerometer Configuration 2
#define ACCEL_CONFIG_2 0x1D

#define GYRO 0x43
#define WHO_AM_I 0x75

// Accelerometer Measurements - High byte and low byte
#define ACCEL_XOUT_H  0x3B
#define ACCEL_XOUT_L  0x3C
#define ACCEL_YOUT_H  0x3D
#define ACCEL_YOUT_L  0x3E
#define ACCEL_ZOUT_H  0x3F
#define ACCEL_ZOUT_L  0x40

// Accelerometer Self-Test Registers
#define SELF_TEST_X_ACCEL  0x0D
#define SELF_TEST_Y_ACCEL  0x0E
#define SELF_TEST_Z_ACCEL  0x0F

// Accelerometer Offset Registers
#define XA_OFFSET_H  0x77
#define XA_OFFSET_L  0x78
#define YA_OFFSET_H  0x7A
#define YA_OFFSET_L  0x7B
#define ZA_OFFSET_H  0x7D
#define ZA_OFFSET_L  0x7E

// Accel Full Scale Select
#define AFS_2G 0x00  // 2G
#define AFS_4G 0x01  // 4G
#define AFS_8G 0x02  // 8G
#define AFS_16G 0x03  // 16G

// Accelerometer Scale Modifiers
const float ACCEL_SCALE_MODIFIER_2G=2.0/32768.0;
const float ACCEL_SCALE_MODIFIER_4G=4.0/32768.0;
const float ACCEL_SCALE_MODIFIER_8G=8.0/32768.0;
const float ACCEL_SCALE_MODIFIER_16G=16.0/32768.0;

#define ACCEL_SCALE_MODIFIER_2G_DIV 32768.0/2.0 //bits per g, higher the value, lower the resolution
#define ACCEL_SCALE_MODIFIER_4G_DIV 32768.0/4.0
#define ACCEL_SCALE_MODIFIER_8G_DIV 32768.0/8.0
#define ACCEL_SCALE_MODIFIER_16G_DIV 32768.0/16.0

// Gyroscope Measurements - High byte and low byte
#define GYRO_XOUT_H  0x43
#define GYRO_XOUT_L  0x44
#define GYRO_YOUT_H  0x45
#define GYRO_YOUT_L  0x46
#define GYRO_ZOUT_H  0x47
#define GYRO_ZOUT_L  0x48

// Gyroscope Self-Test Registers
#define SELF_TEST_X_GYRO  0x00
#define SELF_TEST_Y_GYRO  0x01
#define SELF_TEST_Z_GYRO  0x02

// Gyro Offset Registers
#define XG_OFFSET_H  0x13
#define XG_OFFSET_L  0x14
#define YG_OFFSET_H  0x15
#define YG_OFFSET_L  0x16
#define ZG_OFFSET_H  0x17
#define ZG_OFFSET_L  0x18

// Gyro Full Scale Select
#define GFS_250 0x00  // 250dps
#define GFS_500 0x01  // 500dps
#define GFS_1000 0x02  // 1000dps
#define GFS_2000 0x03  // 2000dps

// Gyroscope Scale Modifiers
#define GYRO_SCALE_MODIFIER_250DEG  250.0/32768.0
#define GYRO_SCALE_MODIFIER_500DEG  500.0/32768.0
#define GYRO_SCALE_MODIFIER_1000DEG  1000.0/32768.0
#define GYRO_SCALE_MODIFIER_2000DEG  2000.0/32768.0

#define GYRO_SCALE_MODIFIER_250DEG_DIV  32768.0/250.0
#define GYRO_SCALE_MODIFIER_500DEG_DIV  32768.0/500.0
#define GYRO_SCALE_MODIFIER_1000DEG_DIV  32768.0/1000.0
#define GYRO_SCALE_MODIFIER_2000DEG_DIV  32768.0/2000.0

// Magnetometer Measurements - High byte and low byte
#define AK8963_HXL  0x03
#define AK8963_HXH  0x04
#define AK8963_HYL  0x05
#define AK8963_HYH  0x06
#define AK8963_HZL  0x07
#define AK8963_HZH  0x08

// Sensitivity Adjustment values
#define AK8963_ASAX 0x10
#define AK8963_ASAY 0x11
#define AK8963_ASAZ 0x12

// Magneto Scale Select
#define AK8963_BIT_14 0x00  // 14bit output
#define AK8963_BIT_16 0x01  // 16bit output

// Continous data output
#define AK8963_MODE_C8HZ 0x02  // 8Hz
#define AK8963_MODE_C100HZ 0x06  // 100Hz

// Device ID
#define AK8963_WIA  0x00

// Information
#define AK8963_INFO  0x01

// Status 1
#define AK8963_ST1  0x02

// Status 2
#define AK8963_ST2  0x09

// Control 1
#define AK8963_CNTL1  0x0A
// set bit 4 of AK8963_CNTL1 to 1 to change resolution 
// set bit 0:3 of AK8963_CNTL1 to AK8963_MODE_C100HZ to change to continous mode
// Control 2
#define AK8963_CNTL2  0x0B

// Self-Test Control
#define AK8963_ASTC  0x0C

// Test 1, 2
#define AK8963_TS1  0x0D
#define AK8963_TS2  0x0E

// I2C Disable
#define AK8963_I2CDIS  0x0F

// Gravity
#define GRAVITY  9.80665

// Default I2C Address
#define MPU9050_ADDRESS_68  0x68
#define MPU9050_ADDRESS_69  0x69
#define AK8963_ADDRESS  0x0C

class MPU9250
{
    public:
    int16_t acc[3];
    int16_t gyro[3];
    int16_t mag[3];

    float acc_float[3];
    float *acc_float_p = &acc_float[0];
    float gyro_float[3];
    float *gyro_float_p = &gyro_float[0];
    float mag_float[3];
    float *mag_float_p = &mag_float[0];
    float gyro_float_rad[3];
    float deg2pi = PI/180.;
    float pi2deg = 180./PI;

    float acc_scale[1];
    float *acc_scale_p = &acc_scale[0];
    float gyro_scale[1];
    float *gyro_scale_p = &gyro_scale[0];
    float mag_scale[1];
    float *mag_scale_p = &mag_scale[0];

    int16_t acc_bias[3];
    int16_t *acc_bias_p = &acc_bias[0];
    int16_t gyro_bias[3];
    int16_t *gyro_bias_p = &gyro_bias[0];
    int16_t mag_bias[3];
    int16_t *mag_bias_p = &mag_bias[0];

    public:
    MPU9250();
    void cs_select();
    void cs_deselect();
    
    void mpu9250_reset();
    void ak8963_reset(); //NOT USE
    
    void read_registers(uint8_t reg, uint8_t *buf, uint16_t len); 
    
    void mpu9250_afs_set(uint8_t afs);
    void mpu9250_gfs_set(uint8_t gfs);
    
    void mpu9250_read_raw_acc(int16_t acc[3]);
    void mpu9250_read_raw_gyro(int16_t gyro[3]);
    void mpu9250_read_raw_mag(int16_t mag[3]);

    void mpu9250_read_acc(float *acc_float_p, int16_t acc_bias[3], float acc_scale[0]);
    void mpu9250_read_gyro(float *gyro_float_p, int16_t gyrol_bias[3], float gyro_scale[0]);

    void mpu9250_read_acc_scale(float *acc_scale_p);
    void mpu9250_read_gyro_scale(float *gyro_scale_p);

    void calibrate_acc(int16_t *acc_bias_p, float *acc_scale_p, int loop);
    void calibrate_gyro(int16_t *gyro_bias_p, int loop);

    void start_spi();
};

#endif