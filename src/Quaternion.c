/*
 * Quaternion.c
 *
 *  Created on: Apr 10, 2023
 *      Author: Mona Dlikan
 */

#include "Quaternion.h"

#include <math.h>
#define PI 3.14159265359
#define SAMPLING_FREQ 100 // Sampling frequency in Hz

Quaternion quaternion_initialize(float a, float b, float c, float d){
    Quaternion q;
    q.a = a;
    q.b = b;
    q.c = c;
    q.d = d;
    return q;
}

Quaternion quaternion_product(Quaternion q1, Quaternion q2){
    //q = q1*q2
    Quaternion q;
    q.a = (q1.a*q2.a) - (q1.b*q2.b) - (q1.c*q2.c) - (q1.d*q2.d);
    q.b = (q1.a*q2.b) + (q1.b*q2.a) + (q1.c*q2.d) - (q1.d*q2.c);
    q.c = (q1.a*q2.c) - (q1.b*q2.d) + (q1.c*q2.a) + (q1.d*q2.b);
    q.d = (q1.a*q2.d) + (q1.b*q2.c) - (q1.c*q2.b) + (q1.d*q2.a);
    return q;
}

Quaternion quaternion_conjugate(Quaternion q1){
    Quaternion q2;
    q2.a = q1.a;
    q2.b = -q1.b;
    q2.c = -q1.c;
    q2.d = -q1.d;
    return q2;
}

Quaternion quaternion_normalize(Quaternion q1){
    Quaternion q2;
    float one_by_sqrt;
    one_by_sqrt = InvSqrt(q1.a*q1.a + q1.b*q1.b + q1.c*q1.c + q1.d*q1.d);
    q2.a = q1.a*one_by_sqrt;
    q2.b = q1.b*one_by_sqrt;
    q2.c = q1.c*one_by_sqrt;
    q2.d = q1.d*one_by_sqrt;
    return q2;
}

Quaternion quaternion_between_vectors(vector_ijk v1, vector_ijk v2){
    // rotates from v1 to v2
    vector_ijk v1_norm = vector_3d_normalize(v1);
    vector_ijk v2_norm = vector_3d_normalize(v2);
    vector_ijk half_way_vector = vector_3d_normalize(vector_3d_sum(v1_norm,v2_norm));
    float angle = vector_3d_dot_product(v1_norm, half_way_vector);
    vector_ijk axis = vector_3d_cross_product(v1_norm, half_way_vector);
    Quaternion result = quaternion_initialize(angle, axis.a, axis.b,axis.c);
    return result;
}

vector_ijk quaternion_rotate_vector(vector_ijk v, Quaternion q){
    Quaternion quaternion_vector = quaternion_initialize(0.0, v.a, v.b, v.c);
    Quaternion q_inverse = quaternion_conjugate(q);
    Quaternion quaternion_rotated_vector = quaternion_product(quaternion_product(q, quaternion_vector),q_inverse);
    vector_ijk rotated = vector_3d_initialize(quaternion_rotated_vector.b,quaternion_rotated_vector.c,quaternion_rotated_vector.d);
    return rotated;
}

//euler_angles quaternion_to_euler_angles(Quaternion q){
//	euler_angles result;
//    double q0 = q.a;
//    double q1 = q.b;
//    double q2 = q.c;
//    double q3 = q.d;
//    result.roll = atan2(2*(q0*q1 + q2*q3),1 - 2*(q1*q1 + q2*q2))*180/3.14;
//    result.pitch = asin(2*(q0*q2 - q3*q1))*180/3.14;
//    if (q.d==0)
//        result.yaw = 0.0;
//    else
//        result.yaw = atan2(2*(q0*q3 + q1*q2),1 - 2*(q2*q2 + q3*q3))*180/3.14;
//    return result;
//}

//// Quaternion structure
//typedef struct Quaternion{
//    float w;
//    float x;
//    float y;
//    float z;
//} Quaternion;

// Calculate Euler angles from accelerometer and gyroscope data using quaternions
void calculateEulerAngles(float accel_x, float accel_y, float accel_z, float gyro_x, float gyro_y, float gyro_z, float* roll, float* pitch, float* yaw)
{
    Quaternion q, dq;
    float dt = 1.0 / SAMPLING_FREQ; // Time interval between sensor readings

    // Normalize accelerometer data
    float accel_norm = sqrtf(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);
    if (accel_norm == 0) {
        // Accelerometer data is invalid
        return;
    }
    accel_x /= accel_norm;
    accel_y /= accel_norm;
    accel_z /= accel_norm;

    // Calculate quaternion from accelerometer data
    q.a = sqrtf((accel_norm + accel_z) * 0.5);
    q.b = (accel_y >= 0) ? -sqrtf((accel_norm - accel_z) * 0.5) : sqrtf((accel_norm - accel_z) * 0.5);
    q.c = (accel_x >= 0) ? -sqrtf((accel_norm - accel_z) * 0.5) : sqrtf((accel_norm - accel_z) * 0.5);
    q.d = 0;

    // Integrate quaternion using gyroscope data
    dq.a = 1;
    dq.b = gyro_x * dt / 2;
    dq.c = gyro_y * dt / 2;
    dq.d = gyro_z * dt / 2;

    Quaternion q_new;
    q_new.a = q.a * dq.a - q.b * dq.b - q.c * dq.c - q.d * dq.d;
    q_new.b = q.a * dq.b + q.b * dq.a + q.c * dq.d - q.d * dq.c;
    q_new.c = q.a * dq.c - q.b * dq.d + q.c * dq.a + q.d * dq.b;
    q_new.d = q.a * dq.d + q.b * dq.c - q.c * dq.b + q.d * dq.a;

    // Normalize quaternion
    float q_norm = sqrtf(q_new.a * q_new.a + q_new.b * q_new.b + q_new.c * q_new.c + q_new.d * q_new.d);
    q_new.a /= q_norm;
    q_new.b /= q_norm;
    q_new.c /= q_norm;
    q_new.d /= q_norm;

    // Convert quaternion to Euler angles
    (*roll) = atan2f(2 * (q_new.a * q_new.b + q_new.c * q_new.d), 1 - 2 * (q_new.b * q_new.b + q_new.c * q_new.c)) * 180 / PI;
    (*pitch) = asinf(2 * (q_new.a * q_new.c - q_new.d * q_new.b)) * 180 / PI;
    (*yaw) = atan2f(2 * (q_new.a * q_new.d + q_new.b * q_new.c), 1 - 2 * (q_new.c * q_new.c + q_new.d * q_new.d)) * 180 / PI;
}
