#include <stdio.h>
#include <string.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "mpu9250.h"

/*
Include information about the device and so on
*/

#define PIN_MISO 4
#define PIN_CS 5
#define PIN_SCK 6
#define PIN_MOSI 7

#define SPI_PORT spi0
#define READ_BIT 0x80

void cs_select(){
    asm volatile("nop \n nop \n nop"); //Delay to set PIN_CS
    gpio_put(PIN_CS, 0); // Active low
    asm volatile("nop \n nop \n nop");
}

void cs_deselect(){
    asm volatile("nop \n nop \n nop"); //Delay to set PIN_CS
    gpio_put(PIN_CS, 1); // Active low
    asm volatile("nop \n nop \n nop");
}

void mpu9250_reset(){
    /*PWR_MGMT_1 0x6B register. Set to 0x01 to reset the hardware with power cycle*/
    uint8_t buf[] = {PWR_MGMT_1, 0x01}; //first is the address, second is the value that we want to transmit
    cs_select();
    spi_write_blocking(SPI_PORT, buf, 2); //int spi_write_blocking	(spi_inst_t *spi,const uint8_t *src,size_t len)	
    cs_deselect();
}

void ak8963_reset(){
    /*Set control mode to continous and change resolution*/
    uint8_t buf[] = {AK8963_CNTL1, AK8963_BIT_16<<4|AK8963_MODE_C100HZ}; //first is the address, second is the value that we want to transmit
    cs_select();
    spi_write_blocking(SPI_PORT, buf, 5); //int spi_write_blocking	(spi_inst_t *spi,const uint8_t *src,size_t len)	
    sleep_us(500);
    cs_deselect();
}//NOT USE

void read_registers(uint8_t reg, uint8_t *buf, uint16_t len){
    /*Read registers of the device. uint8_t is the a cross defintion for unsigned 8 bit variable.
    reg is the mpu9250 starting register. buf is the pointer of the output variable. len is the length
    is the number of byte in buf.
    For this particular device, we send the register we want to read, and we keep reading without re-defining
    the register because it is auto-incrementing: if the ACCEL_XOUT_H register is 59 and we read is content,
    if we read again from the device the new register is 60.
    */
    reg |= READ_BIT; //The first bit of the first byte contains the Read/Write bit and indicates the Read (1) or Write (0) operation.
    cs_select();
    spi_write_blocking(SPI_PORT, &reg, 1); // Define the register to be read and activate reading
    sleep_us(500);
    spi_read_blocking(SPI_PORT, 0, buf, len); //spi_read_blocking(spi_inst_t *spi, uint8_t repeated_tx_data, uint8_t *dst, size_t len)	
    cs_deselect();
    sleep_us(500);
}

void mpu9250_afs_set(uint8_t afs){
    /*afs shold be chosen among AFS_2G AFS_4G etc.*/    
    uint8_t buf[] = {ACCEL_CONFIG, afs << 3 }; //first is the address, second is the value that we want to transmit
    cs_select();
    spi_write_blocking(SPI_PORT, buf, 2); //int spi_write_blocking	(spi_inst_t *spi,const uint8_t *src,size_t len)	
    cs_deselect();
}

void mpu9250_gfs_set(uint8_t gfs){
    /*gfs shold be chosen among GFS_250 GFS_500 etc.*/
    uint8_t buf[] = {GYRO_CONFIG, gfs << 3 }; //first is the address, second is the value that we want to transmit
    cs_select();
    spi_write_blocking(SPI_PORT, buf, 2); //int spi_write_blocking	(spi_inst_t *spi,const uint8_t *src,size_t len)	
    cs_deselect();
}

void mpu9250_read_raw_accel(int16_t accel[3]) { 
    /*Used to get the raw acceleration values from the mpu*/
    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    read_registers(ACCEL_XOUT_H, buffer, 6);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }
}

void mpu9250_read_raw_gyro(int16_t gyro[3]){  
    /*Used to get the raw gyro values from the mpu*/
    uint8_t buffer[6];
    
    read_registers(GYRO_XOUT_H, buffer, 6);

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }
}

void mpu9250_read_raw_mag(int16_t mag[3]) { 
    /*Used to get the raw acceleration values from the mpu*/
    uint8_t buffer[6];
    uint8_t status[1];

    read_registers(AK8963_ST1, status, 1);
    if ((status[0]|0xFE) == 0xFF){
        printf("Data is available\n");
        read_registers(AK8963_HXL, buffer, 6);

        for (int i = 0; i < 3; i++) {
            mag[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
        }
    }
    else{
        printf("Data is not available\n");
    }
    
}

void mpu9250_read_acc(float accel_float[3], int16_t accel_bias[3], float accel_scale[1]){
    int16_t temp[3];
    mpu9250_read_raw_accel(temp);
    
    accel_float[0] = accel_scale[0]*(temp[0]-accel_bias[0]);
    accel_float[1] = accel_scale[0]*(temp[1]-accel_bias[1]);
    accel_float[2] = accel_scale[0]*(temp[2]-accel_bias[2]);
}

void mpu9250_read_gyro(float gyro_float[3], int16_t gyro_bias[3], float gyro_scale[1]){
    int16_t temp[3];
    mpu9250_read_raw_gyro(temp);

    gyro_float[0] = gyro_scale[0]*(temp[0]-gyro_bias[0]);
    gyro_float[1] = gyro_scale[0]*(temp[1]-gyro_bias[1]);
    gyro_float[2] = gyro_scale[0]*(temp[2]-gyro_bias[2]);
}


void mpu9250_read_acc_scale(float *accel_scale_p){
    uint8_t mode[1];
    read_registers(ACCEL_CONFIG, mode, 1);

    if (*mode == AFS_2G){
        *accel_scale_p = ACCEL_SCALE_MODIFIER_2G;
    }
    else if (*mode == AFS_4G){
        *accel_scale_p = ACCEL_SCALE_MODIFIER_4G;
    }
    else if (*mode == AFS_8G){
        *accel_scale_p = ACCEL_SCALE_MODIFIER_8G;
    }
    else if (*mode == AFS_16G){
        *accel_scale_p = ACCEL_SCALE_MODIFIER_16G;
    }
    else {
        *accel_scale_p = ACCEL_SCALE_MODIFIER_2G; // Default value
    }
}

void mpu9250_read_gyro_scale(float *gyro_scale_p){
    uint8_t mode[1];
    read_registers(GYRO_CONFIG, mode, 1);

    if (*mode == GFS_250){
        *gyro_scale_p = GYRO_SCALE_MODIFIER_250DEG;
    }
    else if (*mode == GFS_500){
        *gyro_scale_p = GYRO_SCALE_MODIFIER_500DEG;
    }
    else if (*mode == GFS_1000){
        *gyro_scale_p = GYRO_SCALE_MODIFIER_1000DEG;
    }
    else if (*mode == GFS_2000){
        *gyro_scale_p = GYRO_SCALE_MODIFIER_2000DEG;
    }
    else {
        *gyro_scale_p = GYRO_SCALE_MODIFIER_250DEG; // Default value
    }
}

void calibrate_accel(int16_t *acc_bias_p, float *acc_scale_p, int loop){
    printf("Accelerometer calibration started");
     
    int16_t temp_bias;
    if (*acc_scale_p == ACCEL_SCALE_MODIFIER_2G){
        temp_bias = ACCEL_SCALE_MODIFIER_2G_DIV;
    }
    else if (*acc_scale_p == ACCEL_SCALE_MODIFIER_4G){
        temp_bias = ACCEL_SCALE_MODIFIER_4G_DIV;
    }
    else if (*acc_scale_p == ACCEL_SCALE_MODIFIER_8G){
        temp_bias = ACCEL_SCALE_MODIFIER_8G_DIV;
    }
    else if (*acc_scale_p == ACCEL_SCALE_MODIFIER_16G){
        temp_bias = ACCEL_SCALE_MODIFIER_16G_DIV;
    }
    else {
        temp_bias = ACCEL_SCALE_MODIFIER_2G_DIV; // Default value
    }

    int16_t temp[3];

    mpu9250_read_raw_accel(temp);

    *(acc_bias_p+0) = temp[0];
    *(acc_bias_p+1) = temp[1];
    *(acc_bias_p+2) = temp[2]+temp_bias;

    for (int i = 0; i < loop; i++)
    {
        mpu9250_read_raw_accel(temp);
        *(acc_bias_p+0) = (*(acc_bias_p+0)+temp[0])/2;
        *(acc_bias_p+1) = (*(acc_bias_p+1)+temp[1])/2;
        *(acc_bias_p+2) = (*(acc_bias_p+2)+temp[2]+temp_bias)/2;
        printf("Step %d \n", i);
    }
    printf("Accelerometer calibration completed");
}

void calibrate_gyro(int16_t *gyro_bias_p, int loop){ 
    printf("Gyroscope calibration started");

    int16_t temp[3];

    mpu9250_read_raw_gyro(temp);

    *(gyro_bias_p +0) = temp[0];
    *(gyro_bias_p +1) = temp[1];
    *(gyro_bias_p +2) = temp[2];

    for (int i = 0; i < loop; i++)
    {
        mpu9250_read_raw_gyro(temp);
        *(gyro_bias_p +0) = (*(gyro_bias_p +0)+temp[0])/2;
        *(gyro_bias_p +1) = (*(gyro_bias_p +1)+temp[1])/2;
        *(gyro_bias_p +2) = (*(gyro_bias_p +2)+temp[2])/2;
    }
    printf("Gyroscope calibration completed");
}

void mpu9250_read_raw_gyro_offset(int16_t gyro[3], int16_t gyroCal[3]){  
    /*Used to get the raw gyro values with offset from the mpu*/
    uint8_t buffer[6];
    
    read_registers(0x43, buffer, 6);

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
        gyro[i] -= gyroCal[i];
    }
}


void calculate_angles_from_accel(int16_t eulerAngles[2], int16_t accel[3]){
    /*Uses just the direction gravity is pulling to calculate angles.*/
    float accTotalVector = sqrt((accel[0] * accel[0]) + (accel[1] * accel[1]) + (accel[2] * accel[2]));

    float anglePitchAcc = asin(accel[1] / accTotalVector) * 57.296; //What is this constant?
    float angleRollAcc = asin(accel[0] / accTotalVector) * -57.296;

    eulerAngles[0] = anglePitchAcc;
    eulerAngles[1] = angleRollAcc;
}

void calculate_angles(int16_t eulerAngles[2], int16_t accel[3], int16_t gyro[3], uint64_t usSinceLastReading){
    /*Calculates angles based on the accelerometer and gyroscope. Requires usSinceLastReading to use the gyro.*/
    long hertz = 1000000/usSinceLastReading;
    
    if (hertz < 150)
    {
        calculate_angles_from_accel(eulerAngles, accel);
        return;
    }

    long temp = 1.l/(hertz * 65.5l);  //What is this constant?

    eulerAngles[0] += gyro[0] * temp;
    eulerAngles[1] += gyro[1] * temp;

    eulerAngles[0] += eulerAngles[1] * sin(gyro[2] * temp * 0.1f);
    eulerAngles[1] -= eulerAngles[0] * sin(gyro[2] * temp * 0.1f);

    int16_t accelEuler[2];
    calculate_angles_from_accel(accelEuler, accel);

    eulerAngles[0] = eulerAngles[0] * 0.9996 + accelEuler[0] * 0.0004;
    eulerAngles[1] = eulerAngles[1] * 0.9996 + accelEuler[1] * 0.0004;
}

void convert_to_full(int16_t eulerAngles[2], int16_t accel[3], int16_t fullAngles[2]){
    /*Converts from -90/90 to 360 using the direction gravity is pulling*/
    if (accel[1] > 0 && accel[2] > 0) fullAngles[0] = eulerAngles[0];
    if (accel[1] > 0 && accel[2] < 0) fullAngles[0] = 180 - eulerAngles[0];
    if (accel[1] < 0 && accel[2] < 0) fullAngles[0] = 180 - eulerAngles[0];
    if (accel[1] < 0 && accel[2] > 0) fullAngles[0] = 360 + eulerAngles[0];

    if (accel[0] < 0 && accel[2] > 0) fullAngles[1] = eulerAngles[1];
    if (accel[0] < 0 && accel[2] < 0) fullAngles[1] = 180 - eulerAngles[1];
    if (accel[0] > 0 && accel[2] < 0) fullAngles[1] = 180 - eulerAngles[1];
    if (accel[0] > 0 && accel[2] > 0) fullAngles[1] = 360 + eulerAngles[1];
}

void start_spi(){
    /*Starts the mpu and resets it*/
    spi_init(SPI_PORT, 1000 * 1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);

    mpu9250_reset();

        // See if SPI is working - interrograte the device for its I2C ID number, should be 0x71
    uint8_t id;
    read_registers(0x75, &id, 1);
    printf("I2C address is 0x%x\n", id);
}