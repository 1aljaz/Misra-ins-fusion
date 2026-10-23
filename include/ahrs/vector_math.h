#ifndef VECTOR_MATH_H
#define VECTOR_MATH_H
    #include "../core/pose_types.h"
    float32_t Vector3_Length (const Vector3_t* vector);
    void Vector3_Normalize (Vector3_t* vector);
    void Quaternion_ToEuler(const Quaternion_t* quaternion, EulerAngle_t* euler);
#endif