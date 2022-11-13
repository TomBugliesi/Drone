#include "mpu9250.h"
#include "MahonyAHRS.h"
#include "init.h"

void init(int16_t acc[3], int16_t gyro[3], int16_t mag[3], 
    int16_t gyroCal[3], int16_t *acc_bias_p,
    float *acc_scale_p){
    
    // PICO
    stdio_init_all(); //Initialize pico 

    // MPU9250
    start_spi();  //Starts spi for communication with mpu

    mpu9250_afs_set(AFS_2G); // Define accelerometer sensitivity
    mpu9250_read_acc_scale(acc_scale_p);
    calibrate_accel(acc_bias_p, acc_scale_p, 1000);
    
    mpu9250_gfs_set(GFS_250); // Define gyro sensitivity
    calibrate_gyro(gyroCal,100);  //Calibrates the gyro
    mpu9250_read_raw_gyro_offset(gyro, gyroCal);

    mpu9250_read_raw_accel(acc);  //Get acceleration 
    mpu9250_read_raw_gyro(gyro);
    mpu9250_read_raw_mag(mag);   //Get gyro  

    absolute_time_t timeOfLastCheck;
    timeOfLastCheck = get_absolute_time();
    
}