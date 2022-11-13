#ifndef INIT_H
#define INIT_H

void init(int16_t acc[3], int16_t gyro[3], int16_t mag[3],
 int16_t *gyro_bias_p, int16_t *acc_bias_p,
 float *gyro_scale_p, float *acc_scale_p);

#endif