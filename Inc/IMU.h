#pragma once
typedef struct{
    float acceleration_mg [3];
    float angular_rate_dps[3];
    float linearVelocity;
    float attitudeAngles  [3];
}IMU_Data_t;

void initIMU(void);
void caliIMU(void);
void getIMU(IMU_Data_t*);