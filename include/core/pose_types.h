#ifndef POSE_TYPES_H
#define POSE_TYPES_H
    typedef float float32_t;

    typedef struct{
        float32_t x, y, z;
    } Vector3_t;

    typedef struct{
        Vector3_t acceleration, magnetometer, gyro;
    } IMU_Data_t;

    typedef struct{
        float32_t w, x, y, z;
    } Quaternion_t;

    typedef struct {
        float32_t yaw, roll, pitch;
    } EulerAngle_t;
    
#endif