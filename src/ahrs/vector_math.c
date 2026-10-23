#include "../../include/ahrs/vector_math.h"
#include <math.h>

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