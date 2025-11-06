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

/* Main Example --------------------------------------------------------------*/
//在主函数里面调用这个接口就行
static float fused_pitch = 0.0f;  // 俯仰角（绕Y轴，°）
static float fused_roll = 0.0f;   // 横滚角（绕X轴，°）
static float fused_yaw = 0.0f;    // 偏航角（绕Z轴，°）
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

void IMU_Init(void){
    platform_init();
}
static float gyro_z_bias = 0.0f;  // 陀螺仪Z轴零偏值
// 新增：Pitch/Roll初始偏移量（校准后存储静态偏移）
static float pitch_offset = 0.0f;
static float roll_offset = 0.0f;
// 校准标志位（0=未校准，1=已校准Yaw+Pitch+Roll）
static uint8_t is_all_calibrated = 0;

// 整合式零偏校准：同时校准Yaw陀螺仪零偏 + Pitch/Roll初始偏移
void lsm6dsr_all_calibrate(void) {
    uint8_t calib_cnt = 0;
    float gyro_z_sum = 0.0f;       // Yaw陀螺仪零偏求和
    float pitch_acc_sum = 0.0f;    // Pitch参考值求和
    float roll_acc_sum = 0.0f;     // Roll参考值求和
    uint8_t xl_reg, gy_reg;

    uint32_t caliStartTime = HAL_GetTick();
    uart_printf("<1秒后开始校准IMU，请保持小车静止！大约需要5秒>");
    HAL_Delay(1000);  // 提示用户保持小车水平静置，延迟1秒

    // 采集200组数据（约2秒，确保数据稳定）
    while (calib_cnt < 200) {
        // 同时读取加速度计和陀螺仪数据
        lsm6dsr_xl_flag_data_ready_get(&dev_ctx, &xl_reg);
        lsm6dsr_gy_flag_data_ready_get(&dev_ctx, &gy_reg);

        if (xl_reg && gy_reg) {
            // 1. 读取加速度计数据，计算Pitch/Roll参考值
            memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
            lsm6dsr_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
            acceleration_mg[0] = lsm6dsr_from_fs2g_to_mg(data_raw_acceleration[0]);
            acceleration_mg[1] = lsm6dsr_from_fs2g_to_mg(data_raw_acceleration[1]);
            acceleration_mg[2] = lsm6dsr_from_fs2g_to_mg(data_raw_acceleration[2]);
 
            // 计算Pitch/Roll参考值（和主函数逻辑一致，确保校准偏移匹配）
            float sqrt_yz = sqrtf(acceleration_mg[1] * acceleration_mg[1] + acceleration_mg[2] * acceleration_mg[2]);
            float sqrt_xz = sqrtf(acceleration_mg[0] * acceleration_mg[0] + acceleration_mg[2] * acceleration_mg[2]);
            sqrt_yz = (sqrt_yz < 0.001f) ? 0.001f : sqrt_yz;
            sqrt_xz = (sqrt_xz < 0.001f) ? 0.001f : sqrt_xz;

            float pitch_acc = atan2f(-acceleration_mg[1], sqrt_yz) * 180.0f / PI;  // 你的最终映射逻辑
            float roll_acc = atan2f(acceleration_mg[0], sqrt_xz) * 180.0f / PI;   // 保持和主函数一致

            // 累加求和
            pitch_acc_sum += pitch_acc;
            roll_acc_sum += roll_acc;

            // 2. 读取陀螺仪数据，计算Yaw零偏
            memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
            lsm6dsr_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate);
            float gyro_z = lsm6dsr_from_fs2000dps_to_mdps(data_raw_angular_rate[2]) / 1000.0f;
            gyro_z_sum += gyro_z;

            calib_cnt++;
        }
        HAL_Delay(10);  // 
    }

    // 计算平均值：得到校准参数
    gyro_z_bias = gyro_z_sum / 200.0f;          // Yaw陀螺仪零偏
    pitch_offset = pitch_acc_sum / 200.0f;      // Pitch初始偏移量
    roll_offset = roll_acc_sum / 200.0f;        // Roll初始偏移量
    is_all_calibrated = 1;

    char offset[3][10];
    float_to_str(gyro_z_bias,3,offset[0]);
    float_to_str(pitch_offset,3,offset[1]);
    float_to_str(roll_offset,3,offset[2]);

    uart_printf("<校准完成 用了 %d 毫秒\n", HAL_GetTick()-caliStartTime);
    uart_printf("Gyro Z bias=%s dps | Pitch offset=%s° | Roll offset=%s°>",
                offset[0], offset[1], offset[2]);
    HAL_Delay(2000);
}
void lsm6dsr_read_angle_data_polling(void)
{
    uint8_t xl_reg = 0;
    uint8_t gy_reg = 0;

    /* 1. 未校准则执行整合式校准（一次校准3个轴） */
    if (!is_all_calibrated) {
        lsm6dsr_all_calibrate();
    }

    /* 2. 检查加速度计数据就绪 */
    lsm6dsr_xl_flag_data_ready_get(&dev_ctx, &xl_reg);
    if (xl_reg)
    {
        /* 3. 读取加速度计数据（不变） */
        memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
        lsm6dsr_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
        acceleration_mg[0] = lsm6dsr_from_fs2g_to_mg(data_raw_acceleration[0]);
        acceleration_mg[1] = lsm6dsr_from_fs2g_to_mg(data_raw_acceleration[1]);
        acceleration_mg[2] = lsm6dsr_from_fs2g_to_mg(data_raw_acceleration[2]);

        /* 4. 读取陀螺仪数据并补偿Yaw零偏（不变） */
        lsm6dsr_gy_flag_data_ready_get(&dev_ctx, &gy_reg);
        if (gy_reg)
        {
            memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
            lsm6dsr_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate);
            angular_rate_dps[0] = lsm6dsr_from_fs2000dps_to_mdps(data_raw_angular_rate[0]) / 1000.0f;
            angular_rate_dps[1] = lsm6dsr_from_fs2000dps_to_mdps(data_raw_angular_rate[1]) / 1000.0f;
            angular_rate_dps[2] = (lsm6dsr_from_fs2000dps_to_mdps(data_raw_angular_rate[2]) / 1000.0f) - gyro_z_bias;
        }

        /* 5. 计算dt（不变） */
        uint32_t current_time = HAL_GetTick();
        float dt = (current_time - last_sample_time) / 1000.0f;
        last_sample_time = current_time;
        if (dt > 0.1f) dt = 0.1f;

        /* 6. 加速度计计算Pitch/Roll参考值 + 减去校准偏移量（核心新增） */
        float sqrt_yz = sqrtf(acceleration_mg[1] * acceleration_mg[1] + acceleration_mg[2] * acceleration_mg[2]);
        float sqrt_xz = sqrtf(acceleration_mg[0] * acceleration_mg[0] + acceleration_mg[2] * acceleration_mg[2]);
        sqrt_yz = (sqrt_yz < 0.001f) ? 0.001f : sqrt_yz;
        sqrt_xz = (sqrt_xz < 0.001f) ? 0.001f : sqrt_xz;

        // 计算原始参考值 → 减去校准偏移量（关键：消除安装倾斜）
        float pitch_acc = atan2f(-acceleration_mg[1], sqrt_yz) * 180.0f / PI - pitch_offset;
        float roll_acc = atan2f(acceleration_mg[0], sqrt_xz) * 180.0f / PI - roll_offset;

        /* 7. 互补滤波融合（不变） */
        fused_pitch = 0.85f * (fused_pitch + angular_rate_dps[1] * dt) + 0.15f * pitch_acc;
        fused_roll = 0.85f * (fused_roll + angular_rate_dps[0] * dt) + 0.15f * roll_acc;
        fused_yaw += angular_rate_dps[2] * dt;

        /* 8. 输出（不变） */
        char pitch_str[10], roll_str[10], yaw_str[10];
        float_to_str(fused_pitch, 1, pitch_str);
        float_to_str(fused_roll, 1, roll_str);
        float_to_str(fused_yaw, 1, yaw_str);
        uart_printf("<Angle:Pitch=%s°\tRoll=%s°\tYaw=%s°>\r\n", pitch_str, roll_str, yaw_str);
    }

    HAL_Delay(1);
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