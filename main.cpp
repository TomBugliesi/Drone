#include <stdio.h>
#include <iostream>

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "mpu9250.h"
#include <RF24.h>

extern "C" {
#include "MahonyAHRS.h"
}

#define PI 3.1415

#define CE_PIN 17
#define CSN_PIN 14

int main(){
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    
    // PICO
    stdio_init_all(); //Initialize pico 

    // Time
    absolute_time_t timeOfLastCheck;
    absolute_time_t timeOfPrint;
    timeOfLastCheck = get_absolute_time();
    timeOfPrint = get_absolute_time();

    // MPU9250
    //MPU9250 mpu9250;
    
    // Mahony filter
    Quaternion *q_p, q;
    q_p = &q;
    EulerAngles *Euler_p, Euler;
    Euler_p = &Euler;
 
    int sleep_time = 1; //ms
    float sampleFreq = 1000./sleep_time;
    /////////////////////////////////
    int payload_size = 11;
    
    RF24 radio(CE_PIN, CSN_PIN, RF24_SPI_SPEED);
    spi.begin(spi0, 6, 7, 4); // spi0 or spi1 bus, SCK, TX, RX
    if (!radio.begin(&spi)) {
        printf("Radio hardware is not responding!\n");
    }

    radio.setChannel(115);
    radio.setPALevel(RF24_PA_LOW, false);
    radio.setDataRate(RF24_250KBPS);
    radio.setPayloadSize(payload_size);
    radio.startListening();
    
    while (1)
    {    
    printf("Start Testing\n");
    
    while (radio.available()==0)
    {
        printf("Data not available\n");
    }
    while (radio.available())
    {
        printf("Data is available\n");
    }
    
    }

    return 0;
    /*
    /////////////////////////////////
    
    int n = 1;
    float fs = 0.;

    while (1){
        mpu9250.mpu9250_read_raw_acc(mpu9250.acc);  //Reads the accel and gyro 
        mpu9250.mpu9250_read_raw_gyro(mpu9250.gyro);
        //mpu9250_read_raw_mag(mag);

        mpu9250.mpu9250_read_acc(mpu9250.acc_float_p, mpu9250.acc_bias, mpu9250.acc_scale);
        mpu9250.mpu9250_read_gyro(mpu9250.gyro_float_p, mpu9250.gyro_bias, mpu9250.gyro_scale);

        
        MahonyAHRSupdateIMU(q_p, 
        sampleFreq, 
        mpu9250.gyro_float[0],
        mpu9250.gyro_float[1],
        mpu9250.gyro_float[2],
        mpu9250.acc_float[0],
        mpu9250.acc_float[1],
        mpu9250.acc_float[2]);
        

        //q_struct.w = q[0]; 
        //q_struct.x = q[1];
        //q_struct.y = q[2];
        //q_struct.z = q[3];
        //EulerAngles Euler = ToEulerAngles(*q_p);
        //printf("acc. X = %d, Y = %d, Z = %d\n", mpu9250.acc[0], mpu9250.acc[1], mpu9250.acc[2]);
        //printf("acc_float. X = %f, Y = %f, Z = %f\n", mpu9250.acc_float[0], mpu9250.acc_float[1], mpu9250.acc_float[2]);
        //printf("gyro. X = %d, Y = %d, Z = %d\n", gyro[0], gyro[1], gyro[2]);
        //printf("mag. X = %d, Y = %d, Z = %d\n", mag[0], mag[1], mag[2]);
        //printf("%f,%f,%f,%d\n", acc_float[0], acc_float[1], acc_float[2], n);
        //printf("%f,%f,%f,%d\n", gyro_float[0], gyro_float[1], gyro_float[2], n);
        //printf("q_p: %f,%f,%f,%f\n", q_p->w, q_p->x, q_p->y, q_p->z);
        //printf("%f,%f,%f,%d\n", Euler.roll*mpu9250.pi2deg, Euler.pitch*mpu9250.pi2deg, Euler.yaw*mpu9250.pi2deg, n);
        
        Euler = ToEulerAngles(q);

        if (absolute_time_diff_us(timeOfPrint,get_absolute_time())>100000){
        printf("%f,%f,%f,%f\n", Euler.pitch, Euler.roll, Euler.yaw, sampleFreq);
        //timeOfLastCheck = get_absolute_time();
        //convert_to_full(eulerAngles, acceleration, fullAngles);
        //printf("Scale %f\n", ACCEL_SCALE_MODIFIER_2G);
        //printf("acc_bias. X = %d, Y = %d, Z = %d\n", mpu9250.acc_bias[0], mpu9250.acc_bias[1], mpu9250.acc_bias[2]);
        //printf("Acc. X = %d, Y = %d, Z = %d\n", acc[0], acc[1], acc[2]); 
        //printf("Acc. X = %f, Y = %f, Z = %f\n", acc_float[0], acc_float[1], acc_float[2]); 
        //printf("Euler. Roll = %d, Pitch = %d\n", eulerAngles[0], eulerAngles[1]);
        //printf("Full. Roll = %d, Pitch = %d\n", fullAngles[0], fullAngles[1]);
        //mpu9250_afs_set(AFS_2G);
        timeOfPrint = get_absolute_time();
        }
        n = n+1;
        sampleFreq = 1000000/(absolute_time_diff_us(timeOfLastCheck,get_absolute_time()));
        timeOfLastCheck = get_absolute_time();
        //int Fs = 1000000 / absolute_time_diff_us(timeOfLastCheck, get_absolute_time());
        //printf("Sampling frequency is: %d\n", Fs);
        //timeOfLastCheck = get_absolute_time();

        ///////////////////
        gpio_put(LED_PIN, 1);
        //sleep_ms(sleep_time);
        gpio_put(LED_PIN, 0);
    }

    return 0;
    */
}
