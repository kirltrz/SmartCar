#include "stm32f1xx_hal.h"
#include "tim.h"
#include "motion.h"
#include <math.h>
#include <stdint.h>
#include "main.h"

void setAndUpdateMotion(float targetLinearVelocity, float targetAngular, IMU_Data_t *currentIMUData) {
    // 线速度：纯开环（避免 IMU 漂移干扰）
    const float LINEAR_GAIN = 100.0f;  // 调整此值匹配实际速度（如 0.3 m/s → ~30% 功率）

    // 角速度：完整 PID 闭环（陀螺仪可靠）
    const float angularKp = 100.0f;
    const float angularKi = 0.0f;
    const float angularKd = 10.0f;
    const float MAX_ANGULAR_INTEGRAL = 50.0f;

    static float angularIntegral = 0.0f;
    static float lastAngularError = 0.0f;
    static uint32_t lastTime = 0;

    // 时间间隔
    uint32_t currentTime = HAL_GetTick();
    float dt = (lastTime == 0) ? 0.01f : (currentTime - lastTime) / 1000.0f;
    if (dt > 0.1f) dt = 0.01f;
    lastTime = currentTime;


    // 角度误差（负号用于匹配电机方向，实测调整）
    float angularError = -(targetAngular - currentIMUData->attitudeAngles[2]);

    // 角度 PID
    angularIntegral += angularError * dt;
    if (angularIntegral > MAX_ANGULAR_INTEGRAL) angularIntegral = MAX_ANGULAR_INTEGRAL;
    if (angularIntegral < -MAX_ANGULAR_INTEGRAL) angularIntegral = -MAX_ANGULAR_INTEGRAL;

    float angularDerivative = (angularError - lastAngularError) / (dt < 0.001f ? 0.001f : dt);
    float angularPower = angularKp * angularError
                       + angularKi * angularIntegral
                       + angularKd * angularDerivative;
    lastAngularError = angularError;

    // 线速度开环
    float linearPower = targetLinearVelocity * LINEAR_GAIN;

    // 差速合成（轮距 12 cm）
    const float wheelBase = 0.12f;
    float motor1 = linearPower - angularPower * (wheelBase / 2.0f);
    float motor2 = linearPower + angularPower * (wheelBase / 2.0f);

    // 限幅 [-100, 100]
    motor1 = fmaxf(-100.0f, fminf(100.0f, motor1));
    motor2 = fmaxf(-100.0f, fminf(100.0f, motor2));

    setMotor(motor1, motor2);
}
void initMotor(){
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  // 电机1 IN1（PA8）
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);  // 电机1 IN2（PA9）
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);  // 电机2 IN1（PA10）
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);  // 电机2 IN2（PA11）
}
// 设置单个电机速度
void setSingleMotor(TIM_HandleTypeDef* htim, uint32_t pwm_channel, uint32_t decay_channel, float speed)
{
    const uint16_t DECAY_FAST = htim->Init.Period; // 快衰减（固定高电平）
    const float MAX_SPEED = 100.0f;  // 最大速度百分比
    
    // 速度限幅（-100~100）
    float speed_clamped = (speed > MAX_SPEED) ? MAX_SPEED : 
                         (speed < -MAX_SPEED) ? -MAX_SPEED : speed;
    
    // 计算PWM脉冲值
    float speed_abs = (speed_clamped >= 0) ? speed_clamped : -speed_clamped;
    uint16_t pulse = (uint16_t)(speed_abs * DECAY_FAST / MAX_SPEED);
    
    if (pulse == 0) { 
        // 速度=0 → 停止（两通道低电平）
        __HAL_TIM_SET_COMPARE(htim, pwm_channel, 0);
        __HAL_TIM_SET_COMPARE(htim, decay_channel, 0);
    } 
    else if (speed_clamped > 0) { 
        // 正值 → 逆时针（CCW）：PWM通道输出PWM，衰减通道快衰减（高电平）
        __HAL_TIM_SET_COMPARE(htim, pwm_channel, 0);
        __HAL_TIM_SET_COMPARE(htim, decay_channel, pulse);
    } 
    else { 
        // 负值 → 顺时针（CW）：衰减通道输出PWM，PWM通道快衰减（高电平）
        __HAL_TIM_SET_COMPARE(htim, decay_channel, 0);
        __HAL_TIM_SET_COMPARE(htim, pwm_channel, pulse);
    }
}

// 设置双电机速度
void setMotor(float motor1_speed, float motor2_speed)
{
    if(offGround){
        motor1_speed = 0.0f;
        motor2_speed = 0.0f;
    }
    // 左电机：通道2为PWM，通道1为衰减（逆时针时）
    setSingleMotor(&htim1, TIM_CHANNEL_2, TIM_CHANNEL_1, motor1_speed);
    
    // 右电机：通道4为PWM，通道3为衰减（逆时针时）
    setSingleMotor(&htim1, TIM_CHANNEL_4, TIM_CHANNEL_3, motor2_speed);
}