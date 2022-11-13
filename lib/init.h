#ifndef INIT_H
#define INIT_H

void init(int16_t acc[3], int16_t gyro[3], int16_t mag[3],
 int16_t gyroCal[3], int16_t *acc_bias_p,
 float *acc_scale_p);

#endif