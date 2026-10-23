#ifndef MADGWICK_H
#define MADGWICK_H
    #include "../core/pose_types.h"
    void MadgwickUpdate(const IMU_Data_t* imu_data, Quaternion_t* quaternion);

#endif