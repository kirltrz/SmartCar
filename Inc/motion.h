#pragma once
#include "IMU.h"

void initMotor(void);
void setAndUpdateMotion(float targetLinearVelocity, float targetAngular, IMU_Data_t *currentIMUData);
void setMotor(float motor1_speed, float motor2_speed);