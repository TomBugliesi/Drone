#ifndef MAHONYAHRS_H
#define MAHONYAHRS_H
//=====================================================================================================
// MahonyAHRS.h
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
//----------------------------------------------------------------------------------------------------
// Variable declaration

extern volatile float twoKp;			// 2 * proportional gain (Kp)
extern volatile float twoKi;			// 2 * integral gain (Ki)
extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame

#define PI 3.1415
//---------------------------------------------------------------------------------------------------
// Function declarations

typedef struct Quaternion
{
    double w, x, y, z;
}Quaternion;

typedef struct EulerAngles {
    double roll, pitch, yaw;
}EulerAngles;

void MahonyAHRSupdate(Quaternion *q_p, float sampleFreq, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MahonyAHRSupdateIMU(Quaternion *q_p, float sampleFreq, float gx, float gy, float gz, float ax, float ay, float az);
Quaternion ToQuaternion(double roll, double pitch, double yaw);
EulerAngles ToEulerAngles(Quaternion q);

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
