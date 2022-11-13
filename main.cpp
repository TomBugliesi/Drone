#include <stdio.h>
#include <iostream>

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "mpu9250.h"
#include "init.h"

extern "C" {
#include "MahonyAHRS.h"
}


#define PI 3.1415

int main(){

    // Init pointers and variables
    ///////////////MPU9250/////////////
    int16_t acc[3];
    int16_t gyro[3];
    int16_t mag[3];

    float acc_float[3];
    float gyro_float[3];
    float mag_float[3];

    float gyro_float_rad[3];
    float deg2pi = PI/360.;

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

    float q[4];
    float *q_p = &q[0];
    double angles[3];
    double *angles_p = &angles[0];
 
    int sleep_time = 50; //ms
    float sampleFreq = 1000./sleep_time;
    
    /////////////////////////////////

    int n = 1;

    init(acc, gyro, mag, gyro_bias, acc_bias_p, gyro_scale_p, acc_scale_p); // Init MPU9250

    while (1){
        mpu9250_read_raw_accel(acc);  //Reads the accel and gyro 
        mpu9250_read_raw_gyro(gyro);
        //mpu9250_read_raw_mag(mag);

        mpu9250_read_acc(acc_float, acc_bias, acc_scale);
        mpu9250_read_gyro(gyro_float, gyro_bias, gyro_scale);
        gyro_float_rad[0] = gyro_float[0]*deg2pi;
        gyro_float_rad[1] = gyro_float[1]*deg2pi;
        gyro_float_rad[2] = gyro_float[2]*deg2pi;

        MahonyAHRSupdateIMU(q_p, sampleFreq, gyro_float_rad[0], gyro_float_rad[1], gyro_float_rad[2], acc_float[0], acc_float[1], acc_float[2]);
        //printf("acc. X = %d, Y = %d, Z = %d\n", acc[0], acc[1], acc[2]);
        //printf("gyro. X = %d, Y = %d, Z = %d\n", gyro[0], gyro[1], gyro[2]);
        //printf("mag. X = %d, Y = %d, Z = %d\n", mag[0], mag[1], mag[2]);
        //printf("%f,%f,%f,%d\n", acc_float[0], acc_float[1], acc_float[2], n);
        //printf("%f,%f,%f,%d\n", gyro_float[0], gyro_float[1], gyro_float[2], n);
        //printf("%f,%f,%f,%f\n", q[0], q[1], q[2], q[3]);
        
        angles_p = toEuler(q);
        printf("%lf,%lf,%lf\n", *(angles_p+0), *(angles_p+1), *(angles_p+2));
        //timeOfLastCheck = get_absolute_time();
        //convert_to_full(eulerAngles, acceleration, fullAngles);
        //printf("Scale %f\n", acc_scale);
        //printf("acc_bias. X = %d, Y = %d, Z = %d\n", acc_bias[0], acc_bias[1], acc_bias[2]);
        //printf("Acc. X = %d, Y = %d, Z = %d\n", acc[0], acc[1], acc[2]); 
        //printf("Acc. X = %f, Y = %f, Z = %f\n", acc_float[0], acc_float[1], acc_float[2]); 
        //printf("Euler. Roll = %d, Pitch = %d\n", eulerAngles[0], eulerAngles[1]);
        //printf("Full. Roll = %d, Pitch = %d\n", fullAngles[0], fullAngles[1]);
        //mpu9250_afs_set(AFS_2G);
        sleep_ms(sleep_time);
        n = n+1;
        //int Fs = 1000000 / absolute_time_diff_us(timeOfLastCheck, get_absolute_time());
        //printf("Sampling frequency is: %d\n", Fs);
        //timeOfLastCheck = get_absolute_time();

    }

    return 0;
}
