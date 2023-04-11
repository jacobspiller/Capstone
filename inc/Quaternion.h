/*
 * Quaternion.h
 *
 *  Created on: Apr 10, 2023
 *      Author: Mona Dlikan
 */

#ifndef INC_QUATERNION_H_
#define INC_QUATERNION_H_

#include "Vector.h"

typedef struct Quaternion {
    float a;
    float b;
    float c;
    float d;
    // q = a + bi + cj + dk
} Quaternion;

typedef struct euler_angles {
    float roll;
    float pitch;
    float yaw;
} euler_angles;

Quaternion quaternion_initialize(float a, float b, float c, float d);
Quaternion quaternion_product(Quaternion q1, Quaternion q2);
Quaternion quaternion_conjugate(Quaternion q);
Quaternion quaternion_normalize(Quaternion q);
Quaternion quaternion_between_vectors(vector_ijk v1, vector_ijk v2);
vector_ijk quaternion_rotate_vector(vector_ijk v, Quaternion q);
//euler_angles quaternion_to_euler_angles(Quaternion q);

void calculateEulerAngles(float accel_x, float accel_y, float accel_z, float gyro_x, float gyro_y, float gyro_z, float* roll, float* pitch, float* yaw);


#endif /* INC_QUATERNION_H_ */
