#include "mpu9250.h"
#include "init.h"

void init(int16_t acc[3], int16_t gyro[3], int16_t mag[3], 
    int16_t* gyro_bias_p, int16_t *acc_bias_p,
    float *gyro_scale_p, float *acc_scale_p){
    
    // PICO
    stdio_init_all(); //Initialize pico 

    // MPU9250
    start_spi();  //Starts spi for communication with mpu

    mpu9250_reset();
    //ak8963_reset();

    mpu9250_afs_set(AFS_2G); // Define accelerometer sensitivity
    mpu9250_read_acc_scale(acc_scale_p);
    calibrate_accel(acc_bias_p, acc_scale_p, 1000);
    
    mpu9250_gfs_set(GFS_250); // Define gyro sensitivity
    mpu9250_read_gyro_scale(gyro_scale_p);
    calibrate_gyro(gyro_bias_p,1000);  //Calibrates the gyro

    mpu9250_read_raw_accel(acc);  //Get acceleration 
    mpu9250_read_raw_gyro(gyro);
    //mpu9250_read_raw_mag(mag);   //Get gyro  

    absolute_time_t timeOfLastCheck;
    timeOfLastCheck = get_absolute_time();
    
}