#include <stdio.h>
#include <string.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "mpu9250.h"

MPU9250::MPU9250(){
    MPU9250::start_spi();
    MPU9250::mpu9250_reset();

    MPU9250::mpu9250_afs_set(AFS_2G);
    MPU9250::mpu9250_gfs_set(GFS_250);

    MPU9250::mpu9250_read_acc_scale(acc_scale_p);
    MPU9250::calibrate_acc(acc_bias_p, acc_scale_p, 1000);
    MPU9250::mpu9250_read_gyro_scale(gyro_scale_p);
    MPU9250::calibrate_gyro(gyro_bias_p,1000);  //Calibrates the gyro
}

void MPU9250::cs_select(){
    asm volatile("nop \n nop \n nop"); //Delay to set PIN_CS
    gpio_put(PIN_CS, 0); // Active low
    asm volatile("nop \n nop \n nop");
}

void MPU9250::cs_deselect(){
    asm volatile("nop \n nop \n nop"); //Delay to set PIN_CS
    gpio_put(PIN_CS, 1); // Active low
    asm volatile("nop \n nop \n nop");
}

void MPU9250::mpu9250_reset(){
    /*PWR_MGMT_1 0x6B register. Set to 0x01 to reset the hardware with power cycle*/
    uint8_t buf[] = {PWR_MGMT_1, 0x01}; //first is the address, second is the value that we want to transmit
    this->cs_select();
    spi_write_blocking(SPI_PORT, buf, 2); //int spi_write_blocking	(spi_inst_t *spi,const uint8_t *src,size_t len)	
    this->cs_deselect();
}

void MPU9250::ak8963_reset(){
    /*Set control mode to continous and change resolution*/
    uint8_t buf[] = {AK8963_CNTL1, AK8963_BIT_16<<4|AK8963_MODE_C100HZ}; //first is the address, second is the value that we want to transmit
    this->cs_select();
    spi_write_blocking(SPI_PORT, buf, 5); //int spi_write_blocking	(spi_inst_t *spi,const uint8_t *src,size_t len)	
    sleep_us(500);
    this->cs_deselect();
}//NOT USE

void MPU9250::read_registers(uint8_t reg, uint8_t *buf, uint16_t len){
    /*Read registers of the device. uint8_t is the a cross defintion for unsigned 8 bit variable.
    reg is the mpu9250 starting register. buf is the pointer of the output variable. len is the length
    is the number of byte in buf.
    For this particular device, we send the register we want to read, and we keep reading without re-defining
    the register because it is auto-incrementing: if the ACCEL_XOUT_H register is 59 and we read is content,
    if we read again from the device the new register is 60.
    */
    reg |= READ_BIT; //The first bit of the first byte contains the Read/Write bit and indicates the Read (1) or Write (0) operation.
    this->cs_select();
    spi_write_blocking(SPI_PORT, &reg, 1); // Define the register to be read and activate reading
    sleep_us(500);
    spi_read_blocking(SPI_PORT, 0, buf, len); //spi_read_blocking(spi_inst_t *spi, uint8_t repeated_tx_data, uint8_t *dst, size_t len)	
    this->cs_deselect();
    sleep_us(500);
}

void MPU9250::mpu9250_afs_set(uint8_t afs){
    /*afs shold be chosen among AFS_2G AFS_4G etc.*/    
    uint8_t buf[] = {ACCEL_CONFIG, afs << 3 }; //first is the address, second is the value that we want to transmit
    this->cs_select();
    spi_write_blocking(SPI_PORT, buf, 2); //int spi_write_blocking	(spi_inst_t *spi,const uint8_t *src,size_t len)	
    this->cs_deselect();
}

void MPU9250::mpu9250_gfs_set(uint8_t gfs){
    /*gfs shold be chosen among GFS_250 GFS_500 etc.*/
    uint8_t buf[] = {GYRO_CONFIG, gfs << 3 }; //first is the address, second is the value that we want to transmit
    this->cs_select();
    spi_write_blocking(SPI_PORT, buf, 2); //int spi_write_blocking	(spi_inst_t *spi,const uint8_t *src,size_t len)	
    this->cs_deselect();
}

void MPU9250::mpu9250_read_raw_acc(int16_t acc[3]) { 
    /*Used to get the raw acceleration values from the mpu*/
    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    this->read_registers(ACCEL_XOUT_H, buffer, 6);

    for (int i = 0; i < 3; i++) {
        acc[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }
}

void MPU9250::mpu9250_read_raw_gyro(int16_t gyro[3]){  
    /*Used to get the raw gyro values from the mpu*/
    uint8_t buffer[6];
    
    this->read_registers(GYRO_XOUT_H, buffer, 6);

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }
}

void MPU9250::mpu9250_read_raw_mag(int16_t mag[3]) { 
    /*Used to get the raw acceleration values from the mpu*/
    uint8_t buffer[6];
    uint8_t status[1];

    this->read_registers(AK8963_ST1, status, 1);
    if ((status[0]|0xFE) == 0xFF){
        printf("Data is available\n");
        this->read_registers(AK8963_HXL, buffer, 6);

        for (int i = 0; i < 3; i++) {
            mag[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
        }
    }
    else{
        printf("Data is not available\n");
    }
    
}

void MPU9250::mpu9250_read_acc(float *acc_float_p, int16_t acc_bias[3], float acc_scale[1]){
    int16_t temp[3];
    this->mpu9250_read_raw_acc(temp);
    
    *(acc_float_p + 0) = acc_scale[0]*(temp[0]-acc_bias[0]);
    *(acc_float_p + 1) = acc_scale[0]*(temp[1]-acc_bias[1]);
    *(acc_float_p + 2) = acc_scale[0]*(temp[2]-acc_bias[2]);
}

void MPU9250::mpu9250_read_gyro(float *gyro_float_p, int16_t gyro_bias[3], float gyro_scale[1]){
    int16_t temp[3];
    this->mpu9250_read_raw_gyro(temp);

    *(gyro_float_p + 0) = this->deg2pi*gyro_scale[0]*(temp[0]-gyro_bias[0]);
    *(gyro_float_p + 1) = this->deg2pi*gyro_scale[0]*(temp[1]-gyro_bias[1]);
    *(gyro_float_p + 2) = this->deg2pi*gyro_scale[0]*(temp[2]-gyro_bias[2]);
}


void MPU9250::mpu9250_read_acc_scale(float *acc_scale_p){
    uint8_t mode[1];
    this->read_registers(ACCEL_CONFIG, mode, 1);
    
    if (*mode == AFS_2G){
        *acc_scale_p = ACCEL_SCALE_MODIFIER_2G;
    }
    else if (*mode == AFS_4G){
        *acc_scale_p = ACCEL_SCALE_MODIFIER_4G;
    }
    else if (*mode == AFS_8G){
        *acc_scale_p = ACCEL_SCALE_MODIFIER_8G;
    }
    else if (*mode == AFS_16G){
        *acc_scale_p = ACCEL_SCALE_MODIFIER_16G;
    }
    else {
        *acc_scale_p = ACCEL_SCALE_MODIFIER_2G; // Default value
    }
}

void MPU9250::mpu9250_read_gyro_scale(float *gyro_scale_p){
    uint8_t mode[1];
    this->read_registers(GYRO_CONFIG, mode, 1);

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

void MPU9250::calibrate_acc(int16_t *acc_bias_p, float *acc_scale_p, int loop){
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

    this->mpu9250_read_raw_acc(temp);

    *(acc_bias_p+0) = temp[0];
    *(acc_bias_p+1) = temp[1];
    *(acc_bias_p+2) = temp[2]+temp_bias;

    for (int i = 0; i < loop; i++)
    {
        this->mpu9250_read_raw_acc(temp);
        *(acc_bias_p+0) = (*(acc_bias_p+0)+temp[0])/2;
        *(acc_bias_p+1) = (*(acc_bias_p+1)+temp[1])/2;
        *(acc_bias_p+2) = (*(acc_bias_p+2)+temp[2]+temp_bias)/2;
        printf("Step %d \n", i);
    }
    printf("Accelerometer calibration completed");
}

void MPU9250::calibrate_gyro(int16_t *gyro_bias_p, int loop){ 
    printf("Gyroscope calibration started");

    int16_t temp[3];

    this->mpu9250_read_raw_gyro(temp);

    *(gyro_bias_p +0) = temp[0];
    *(gyro_bias_p +1) = temp[1];
    *(gyro_bias_p +2) = temp[2];

    for (int i = 0; i < loop; i++)
    {
        this->mpu9250_read_raw_gyro(temp);
        *(gyro_bias_p +0) = (*(gyro_bias_p +0)+temp[0])/2;
        *(gyro_bias_p +1) = (*(gyro_bias_p +1)+temp[1])/2;
        *(gyro_bias_p +2) = (*(gyro_bias_p +2)+temp[2])/2;
    }
    printf("Gyroscope calibration completed");
}

void MPU9250::start_spi(){
    /*Starts the mpu and resets it*/
    spi_init(SPI_PORT, 1000 * 1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);

    this->mpu9250_reset();

        // See if SPI is working - interrograte the device for its I2C ID number, should be 0x71
    uint8_t id;
    this->read_registers(0x75, &id, 1);
    printf("I2C address is 0x%x\n", id);
}