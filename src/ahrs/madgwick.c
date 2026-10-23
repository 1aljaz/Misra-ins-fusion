#include "../../include/ahrs/madgwick.h"
#include "../../include/ahrs/vector_math.h"
#define BETA 0.1f

void MadgwickUpdate(const IMU_Data_t* imu_data, Quaternion_t* quaternion){
    Vector3_t accel = imu_data->acceleration;
    Vector3_Normalize(&accel);
    Vector3_t magnet = imu_data->magnetometer;
    Vector3_Normalize(&magnet);
    Vector3_t gyro = imu_data->gyro;

    Quaternion_t q = *quaternion;
    float32_t dt = 0.01f;

    float32_t qDot_w = 0.5f * (-q.x * gyro.x - q.y * gyro.y - q.z * gyro.z);
    float32_t qDot_x = 0.5f * ( q.w * gyro.x + q.y * gyro.z - q.z * gyro.y);
    float32_t qDot_y = 0.5f * ( q.w * gyro.y - q.x * gyro.z + q.z * gyro.x);
    float32_t qDot_z = 0.5f * ( q.w * gyro.z + q.x * gyro.y - q.y * gyro.x);

    float32_t _2qw = 2.0f * q.w;
    float32_t _2qx = 2.0f * q.x;
    float32_t _2qy = 2.0f * q.y;
    float32_t _2qz = 2.0f * q.z;

    float32_t vx = _2qx * q.z  - _2qw * q.y;
    float32_t vy = _2qw * q.x  + _2qy * q.z;
    float32_t vz = 1.0f - _2qx * q.x - _2qy * q.y; 

    float32_t ex = vx - accel.x;
    float32_t ey = vy - accel.y;
    float32_t ez = vz - accel.z;

    float32_t step_w = -_2qy * ex + _2qx * ey;
    float32_t step_x =  _2qz * ex + _2qw * ey - 4.0f * q.x * ez;
    float32_t step_y = -_2qw * ex + _2qz * ey - 4.0f * q.y * ez;
    float32_t step_z =  _2qx * ex + _2qy * ey;

    float32_t step_norm = sqrtf(step_w * step_w + step_x * step_x + step_y * step_y + step_z * step_z);

    if (step_norm > 0.0f){
        step_w /= step_norm;
        step_x /= step_norm;
        step_y /= step_norm;
        step_z /= step_norm;
    }

    qDot_w = qDot_w - BETA * step_w;
    qDot_x = qDot_x - BETA * step_x;
    qDot_y = qDot_y - BETA * step_y;
    qDot_z = qDot_z - BETA * step_z;

    q.w += qDot_w * dt;
    q.x += qDot_x * dt;
    q.y += qDot_y * dt;
    q.z += qDot_z * dt;

    float32_t q_norm = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);

    if (q_norm > 0.0f){
        q.w /= q_norm;
        q.x /= q_norm;
        q.y /= q_norm;
        q.z /= q_norm;
    }

    *quaternion = q;
}