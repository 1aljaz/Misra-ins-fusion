#include "../include/ahrs/madgwick.h"
#include "../include/ahrs/vector_math.h"
#include <stdio.h>

int main(){
    IMU_Data_t sensor_data;
    Quaternion_t q = {1.0f, 0.0f, 0.0f, 0.0f};
    EulerAngle_t euler;

   for (int i = 0; i <= 100; i++) {
        sensor_data.acceleration = (Vector3_t){0.0f, 0.0f, 1.0f};
        
        sensor_data.magnetometer = (Vector3_t){1.0f, 0.0f, 0.0f};
        
        sensor_data.gyro = (Vector3_t){1.0f, 0.0f, 0.0f}; 

        MadgwickUpdate(&sensor_data, &q);

        Quaternion_ToEuler(&q, &euler);
        float32_t rad2deg = 180.0f / 3.14159265f;

        printf("%f,%f,%f\n", euler.roll * rad2deg, euler.pitch * rad2deg, euler.yaw * rad2deg);
    }
}