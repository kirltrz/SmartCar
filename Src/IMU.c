#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "..\Drivers\BSP\Components\lsm6dsr\lsm6dsr_reg.h"
#include "stm32F1xx_hal.h"
#include "main.h"
#include "IMU.h"
#include "custom_bus.h"
#include "UART.h"
#include "stm32f1xx_hal.h"

#define CS_Pin IMU_CS_Pin
#define CS_GPIO_Port IMU_CS_GPIO_Port
#define BOOT_TIME 10 // ms
#define PI 3.1415926
extern SPI_HandleTypeDef hspi1;

extern UART_HandleTypeDef huart1;
static stmdev_ctx_t dev_ctx;

/* Private variables ---------------------------------------------------------*/
static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
static float acceleration_mg[3];
static float angular_rate_dps[3];
static uint8_t whoamI, rst;

static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void tx_com(uint8_t *tx_buffer, uint16_t len);
static void platform_delay(uint32_t ms);
static void platform_init(void);

static float fused_pitch = 0.0f;  // 俯仰角（绕Y轴，°）
static float fused_roll = 0.0f;   // 横滚角（绕X轴，°）
static float fused_yaw = 0.0f;    // 偏航角（绕Z轴，°）
static float accumulated_linear_vel = 0.0f;
static uint32_t last_sample_time = 0;  // 上一次采样时间（用于计算dt）

void float_to_str(float f, uint8_t decimals, char *buf) {
  if (buf == NULL || decimals > 3) return;

  char *ptr = buf;
  int32_t integer_part;
  int32_t decimal_part;

  // 处理负号
  if (f < 0.0f) {
    *ptr++ = '-';
    f = -f;
  }

  // 四舍五入修正
  float round_factor = 1.0f;
  for (uint8_t i = 0; i < decimals; i++) {
    round_factor *= 10.0f;
  }
  f += 0.5f / round_factor;

  // 拆分整数和小数部分
  integer_part = (int32_t)f;
  decimal_part = (int32_t)((f - integer_part) * round_factor);

  // 整数部分转字符串
  if (integer_part == 0) {
    *ptr++ = '0';
  } else {
    char temp[10];
    uint8_t len = 0;
    while (integer_part > 0) {
      temp[len++] = (integer_part % 10) + '0';
      integer_part /= 10;
    }
    // 逆序转回
    for (int8_t i = len - 1; i >= 0; i--) {
      *ptr++ = temp[i];
    }
  }

  // 小数部分转字符串
  if (decimals > 0) {
    *ptr++ = '.';
    uint8_t decimal_len = 0;
    char temp[4];
    while (decimal_part > 0 || decimal_len < decimals) {
      temp[decimal_len++] = (decimal_part % 10) + '0';
      decimal_part /= 10;
    }
    // 逆序转回
    for (int8_t i = decimal_len - 1; i >= 0; i--) {
      *ptr++ = temp[i];
    }
  }

  *ptr = '\0';  // 字符串结束符
}

void initIMU(void){
    platform_init();
}
static float gyro_z_bias = 0.0f;  // 陀螺仪Z轴零偏值
static float acc_x_bias = 0.0f;
static float acc_y_bias = 0.0f;

// 新增：Pitch/Roll初始偏移量（校准后存储静态偏移）
static float pitch_offset = 0.0f;
static float roll_offset = 0.0f;
// 校准标志位（0=未校准，1=已校准Yaw+Pitch+Roll）
static uint8_t is_all_calibrated = 0;

// 原有全局变量保持不变，新增以下3个
static float vel_x = 0.0f;    // X轴速度分量（m/s）- 替代原有的“合速度累加”
static float vel_y = 0.0f;    // Y轴速度分量（m/s）- 支持反向减速
const float vel_deadzone = 0.01f;  // 速度死区（<0.05m/s视为静止）

// 整合式零偏校准：同时校准Yaw陀螺仪零偏 + Pitch/Roll初始偏移 + XY加速度零偏
void caliIMU(void) {
    uint8_t calib_cnt = 0;
    float gyro_z_sum = 0.0f;       // Yaw陀螺仪零偏求和
    float pitch_acc_sum = 0.0f;    // Pitch参考值求和
    float roll_acc_sum = 0.0f;     // Roll参考值求和
    // 新增：XY轴加速度零偏求和变量
    float acc_x_sum = 0.0f;        // X轴加速度原始值累加
    float acc_y_sum = 0.0f;        // Y轴加速度原始值累加
    
    uint8_t xl_reg, gy_reg;

    uint32_t caliStartTime = HAL_GetTick();
    uart_printf("<1秒后开始校准IMU，请保持小车水平静止！大约需要5秒>");
    HAL_Delay(1000);  // 提示用户保持水平静置（关键：确保XY轴无加速度）

    // 采集200组数据（约2秒，确保数据稳定）
    while (calib_cnt < 200) {
        // 同时读取加速度计和陀螺仪数据（仅有效数据参与校准）
        lsm6dsr_xl_flag_data_ready_get(&dev_ctx, &xl_reg);
        lsm6dsr_gy_flag_data_ready_get(&dev_ctx, &gy_reg);

        if (xl_reg && gy_reg) {
            // 1. 读取加速度计数据：同时用于Pitch/Roll偏移校准 + XY加速度零偏校准
            memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
            lsm6dsr_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
            float acc_x_raw = lsm6dsr_from_fs2g_to_mg(data_raw_acceleration[0]);  // X轴原始值（mg）
            float acc_y_raw = lsm6dsr_from_fs2g_to_mg(data_raw_acceleration[1]);  // Y轴原始值（mg）
            acceleration_mg[2] = lsm6dsr_from_fs2g_to_mg(data_raw_acceleration[2]);  // Z轴（无需零偏，承受重力）

            // 新增：累加XY轴加速度原始值（用于计算零偏）
            acc_x_sum += acc_x_raw;
            acc_y_sum += acc_y_raw;

            // 原有逻辑：计算Pitch/Roll参考值（用于初始偏移校准）
            float sqrt_yz = sqrtf(acc_y_raw * acc_y_raw + acceleration_mg[2] * acceleration_mg[2]);
            float sqrt_xz = sqrtf(acc_x_raw * acc_x_raw + acceleration_mg[2] * acceleration_mg[2]);
            sqrt_yz = (sqrt_yz < 0.001f) ? 0.001f : sqrt_yz;
            sqrt_xz = (sqrt_xz < 0.001f) ? 0.001f : sqrt_xz;

            float pitch_acc = atan2f(-acc_y_raw, sqrt_yz) * 180.0f / PI;
            float roll_acc = atan2f(acc_x_raw, sqrt_xz) * 180.0f / PI;

            // 原有累加：Pitch/Roll参考值
            pitch_acc_sum += pitch_acc;
            roll_acc_sum += roll_acc;

            // 2. 读取陀螺仪数据，计算Yaw零偏（原有逻辑保持不变）
            memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
            lsm6dsr_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate);
            float gyro_z = lsm6dsr_from_fs2000dps_to_mdps(data_raw_angular_rate[2]) / 1000.0f;
            gyro_z_sum += gyro_z;

            calib_cnt++;
        }
        HAL_Delay(10);
    }

    // 计算平均值：得到所有校准参数（新增XY加速度零偏）
    gyro_z_bias = gyro_z_sum / 200.0f;          // Yaw陀螺仪零偏
    pitch_offset = pitch_acc_sum / 200.0f;      // Pitch初始偏移量
    roll_offset = roll_acc_sum / 200.0f;        // Roll初始偏移量
    acc_x_bias = acc_x_sum / 200.0f;            // 新增：X轴加速度零偏（mg）
    acc_y_bias = acc_y_sum / 200.0f;            // 新增：Y轴加速度零偏（mg）
    is_all_calibrated = 1;

    // 新增：XY加速度零偏的字符串转换（用于串口输出）
    char offset[5][10];  // 原有3个→扩展为5个（新增accX/accY偏置）
    float_to_str(gyro_z_bias, 3, offset[0]);
    float_to_str(pitch_offset, 3, offset[1]);
    float_to_str(roll_offset, 3, offset[2]);
    float_to_str(acc_x_bias, 3, offset[3]);     // X轴加速度零偏
    float_to_str(acc_y_bias, 3, offset[4]);     // Y轴加速度零偏

    // 串口输出：补充XY加速度零偏信息（保持格式一致）
    uart_printf("<校准完成 用时：%d 毫秒>\n", HAL_GetTick()-caliStartTime);
    uart_printf("Gyro Z bias=%s dps | Pitch offset=%s° | Roll offset=%s° | Acc X bias=%s mg | Acc Y bias=%s mg>",
                offset[0], offset[1], offset[2], offset[3], offset[4]);
    HAL_Delay(2000);
}
void getIMU(IMU_Data_t *IMU_Data)
{
    uint8_t xl_reg = 0;
    uint8_t gy_reg = 0;

    /* 检查加速度计数据就绪 */
    lsm6dsr_xl_flag_data_ready_get(&dev_ctx, &xl_reg);
    if (xl_reg)
    {
        /* 读取加速度计数据 + 应用XY轴零偏校准（原有逻辑不变） */
        memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
        lsm6dsr_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
        acceleration_mg[0] = lsm6dsr_from_fs2g_to_mg(data_raw_acceleration[0]) - acc_x_bias;
        acceleration_mg[1] = lsm6dsr_from_fs2g_to_mg(data_raw_acceleration[1]) - acc_y_bias;
        acceleration_mg[2] = lsm6dsr_from_fs2g_to_mg(data_raw_acceleration[2]);

        /* 读取陀螺仪数据并补偿Yaw零偏（原有逻辑不变） */
        lsm6dsr_gy_flag_data_ready_get(&dev_ctx, &gy_reg);
        if (gy_reg)
        {
            memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
            lsm6dsr_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate);
            angular_rate_dps[0] = lsm6dsr_from_fs2000dps_to_mdps(data_raw_angular_rate[0]) / 1000.0f;
            angular_rate_dps[1] = lsm6dsr_from_fs2000dps_to_mdps(data_raw_angular_rate[1]) / 1000.0f;
            angular_rate_dps[2] = (lsm6dsr_from_fs2000dps_to_mdps(data_raw_angular_rate[2]) / 1000.0f) - gyro_z_bias;
        }

        /* 计算dt（原有逻辑不变）*/
        uint32_t current_time = HAL_GetTick();
        float dt = (current_time - last_sample_time) / 1000.0f;
        last_sample_time = current_time;
        if (dt > 0.1f) dt = 0.1f;

        // #################### 线速度计算（核心修复）####################
        // 1. 单位转换：mg → m/s²（保留XY轴方向，不取绝对值）
        float ax = (acceleration_mg[0] / 1000.0f )* 9.80665f;  // X轴加速度（带方向：正/负）
        float ay = (acceleration_mg[1] / 1000.0f )* 9.80665f;  // Y轴加速度（带方向：正/负）

        
        // 2. 噪声抑制：加速度死区（过滤微小偏差，可根据实际调整）
        const float acc_deadzone = 0.015f;  // 适当增大死区（原0.1f），减少漂移
        if (fabsf(ax) < acc_deadzone) ax = 0.0f;
        if (fabsf(ay) < acc_deadzone) ay = 0.0f;

        // 3. 速度矢量积分（支持减速/反向，替代原合加速度绝对值积分）
        vel_x += -ax * dt;  // X轴速度：加速度带方向，可加可减
        vel_y += -ay * dt;  // Y轴速度：同上

        // 4. 静止判断：速度小于死区时，强制归零（关键！解决漂移）
        float total_vel = sqrtf(vel_x*vel_x + vel_y*vel_y);
        if (total_vel < vel_deadzone)
        {
            vel_x = 0.0f;
            vel_y = 0.0f;
            total_vel = 0.0f;
        }

        // 5. （可选）速度限制：避免异常值（根据小车最大速度调整，如2m/s）
        const float max_vel = 2.0f;
        if (total_vel > max_vel)
        {
            vel_x = (vel_x / total_vel) * max_vel;
            vel_y = (vel_y / total_vel) * max_vel;
            total_vel = max_vel;
        }
        // ################################################################

        /* 加速度计计算Pitch/Roll参考值（原有逻辑不变）*/
        float sqrt_yz = sqrtf(acceleration_mg[1] * acceleration_mg[1] + acceleration_mg[2] * acceleration_mg[2]);
        float sqrt_xz = sqrtf(acceleration_mg[0] * acceleration_mg[0] + acceleration_mg[2] * acceleration_mg[2]);
        sqrt_yz = (sqrt_yz < 0.001f) ? 0.001f : sqrt_yz;
        sqrt_xz = (sqrt_xz < 0.001f) ? 0.001f : sqrt_xz;

        float pitch_acc = atan2f(-acceleration_mg[1], sqrt_yz) * 180.0f / PI - pitch_offset;
        float roll_acc = atan2f(acceleration_mg[0], sqrt_xz) * 180.0f / PI - roll_offset;

        /* 互补滤波融合（原有逻辑不变） */
        fused_pitch = 0.85f * (fused_pitch + angular_rate_dps[1] * dt) + 0.15f * pitch_acc;
        fused_roll = 0.85f * (fused_roll + angular_rate_dps[0] * dt) + 0.15f * roll_acc;
        fused_yaw += angular_rate_dps[2] * dt;

        // 数据赋值（更新为修复后的线速度）
        for (int i = 0; i < 3; i++) {
          IMU_Data->acceleration_mg[i]  = acceleration_mg[i];
          IMU_Data->angular_rate_dps[i] = angular_rate_dps[i];
        }

        IMU_Data->linearVelocity    = vel_y;
        IMU_Data->attitudeAngles[0] = fused_pitch;
        IMU_Data->attitudeAngles[1] = fused_roll;
        IMU_Data->attitudeAngles[2] = fused_yaw;

    }
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len)
{
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(handle, &reg, 1, 1000);
    HAL_SPI_Transmit(handle, (uint8_t *)bufp, len, 1000);
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

    return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
    reg |= 0x80;
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(handle, &reg, 1, 1000);
    HAL_SPI_Receive(handle, bufp, len, 1000);
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
    return 0;
}

/*
 * @brief  Send buffer to console (platform dependent)
 *
 * @param  tx_buffer     buffer to transmit
 * @param  len           number of byte to send
 *
 */
static void tx_com(uint8_t *tx_buffer, uint16_t len)
{
    HAL_UART_Transmit(&huart1, tx_buffer, len, 1000);
}

/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms)
{
    HAL_Delay(ms);
}

/*
 * @brief  platform specific initialization (platform dependent)
 */
void platform_init(void)
{
    /* Initialize mems driver interface */
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    dev_ctx.handle = &hspi2;

    /* Wait sensor boot time */
    platform_delay(BOOT_TIME);
    /* Check device ID */
    while (1)
    {
        // 考虑如何喂狗
        lsm6dsr_device_id_get(&dev_ctx, &whoamI);
        if (whoamI == LSM6DSR_ID)
        {
            uart_printf(
                    "<Read id :0x%2X>",
                    whoamI);
            //tx_com(tx_buffer, strlen((char const *)tx_buffer));

            break;
        }
        platform_delay(BOOT_TIME);
    }
    /* Restore default configuration */
    lsm6dsr_reset_set(&dev_ctx, PROPERTY_ENABLE);
    do
    {
        lsm6dsr_reset_get(&dev_ctx, &rst);
    } while (rst);

    /* Disable I3C interface */
    lsm6dsr_i3c_disable_set(&dev_ctx, LSM6DSR_I3C_DISABLE);
    /* Enable Block Data Update */
    lsm6dsr_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
    /* Set Output Data Rate */
    lsm6dsr_xl_data_rate_set(&dev_ctx, LSM6DSR_XL_ODR_1666Hz);//过高会使yaw角漂移加重
    lsm6dsr_gy_data_rate_set(&dev_ctx, LSM6DSR_GY_ODR_1666Hz);
    /* Set full scale */
    lsm6dsr_xl_full_scale_set(&dev_ctx, LSM6DSR_2g);
    lsm6dsr_gy_full_scale_set(&dev_ctx, LSM6DSR_2000dps);
    /* Configure filtering chain(No aux interface)
     * Accelerometer - LPF1 + LPF2 path
     */
    lsm6dsr_xl_hp_path_on_out_set(&dev_ctx, LSM6DSR_LP_ODR_DIV_100);
    lsm6dsr_xl_filter_lp2_set(&dev_ctx, PROPERTY_ENABLE);
}