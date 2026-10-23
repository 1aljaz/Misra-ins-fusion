#include "../../include/ahrs/vector_math.h"
#include <math.h>
#define pi 3.14159265f
float32_t Vector3_Length(const Vector3_t* vector){
    return sqrtf(vector->x*vector->x + vector->y*vector->y + vector->z*vector->z);
}

void Vector3_Normalize(Vector3_t* vector){
    float32_t vector_length = Vector3_Length(vector);
    if (vector_length > 0.0f){
        vector->x /= vector_length;
        vector->y /= vector_length;
        vector->z /= vector_length;
    }
}

void Quaternion_ToEuler(const Quaternion_t* quaternion, EulerAngle_t* euler){
    Quaternion_t q = *quaternion;
    // 1. Roll
    float32_t sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    float32_t cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    euler->roll = atan2f(sinr_cosp, cosr_cosp);
    // 2. Pitch
    float32_t sinp = 2.0f * (q.w * q.y - q.z * q.x);
    if (sinp >= 1.0f){
        euler->pitch = pi / 2.0f;
    } else if (sinp <= -1.0f){
        euler->pitch = -pi / 2.0f;
    } else{
        euler->pitch = asinf(sinp);
    }
    // 3. Yaw
    float32_t siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    float32_t cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    euler->yaw = atan2f(siny_cosp, cosy_cosp);
}