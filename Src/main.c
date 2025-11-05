/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#include "ad7689.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LSM6DSR_CS_HIGH()  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET)
#define LSM6DSR_CS_LOW()   HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void setLED(bool on){
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/**
 * @brief  类似printf的UART发送函数，支持格式化输出
 * @param  format: 格式化字符串（同printf）
 * @param  ...: 可变参数列表
 * @retval 发送的字节数（-1表示缓冲区溢出）
 */
int uart_printf(const char *format, ...) {
  uint8_t buf[256];  // 定义缓冲区（根据需求调整大小）
  va_list args;      // 可变参数列表

  // 初始化可变参数列表
  va_start(args, format);

  // 使用vsnprintf将格式化字符串写入缓冲区
  // 最多写入255字节（留1字节给结束符）
  int len = vsnprintf((char *)buf, sizeof(buf), format, args);

  // 清理可变参数列表
  va_end(args);

  // 检查缓冲区是否溢出（len >= sizeof(buf)表示溢出）
  if (len < 0 || len >= sizeof(buf)) {
    return -1;  // 缓冲区溢出，返回错误
  }

  // 通过HAL库发送缓冲区数据
  HAL_UART_Transmit(&huart3, buf, len, 1000);  // 超时1000ms

  return len;  // 返回实际发送的字节数
}

// 写寄存器
void LSM6DSR_WriteReg(uint8_t reg, uint8_t *value, uint16_t length) {
    LSM6DSR_CS_LOW();

    uint8_t txData[length];
    txData[0] = reg;
    for (uint16_t i=1; i<length; i++) {
      txData[i] = value[i];
    }
    
    HAL_SPI_Transmit(&hspi2, txData, length, HAL_MAX_DELAY);
    
    LSM6DSR_CS_HIGH();
}

// 读寄存器
uint8_t LSM6DSR_ReadReg(uint8_t reg, uint8_t *value, uint16_t length) {
    // 检查参数合法性（避免空指针或长度为0）
    if (value == NULL || length == 0) {
        return 1; // 无效参数
    }

    LSM6DSR_CS_LOW(); // 拉低片选，选中传感器

    // 准备发送数据：第1字节为带读标志的寄存器地址，后续为dummy字节（用于读取数据）
    uint8_t txData[1 + length];
    txData[0] = reg | 0x80; // 最高位置1表示读操作
    for (uint16_t i = 0; i < length; i++) {
        txData[1 + i] = 0x00; // 填充dummy字节（SPI读取时需发送无效数据换取接收数据）
    }

    // 接收缓冲区：长度与发送数据一致（1个地址字节 + length个数据字节）
    uint8_t rxData[1 + length];

    // 执行SPI双向传输：发送地址和dummy，同时接收传感器返回的数据
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(
        &hspi2, 
        txData,       // 发送缓冲区（地址 + dummy）
        rxData,       // 接收缓冲区（无效字节 + 有效数据）
        1 + length,   // 总传输字节数
        HAL_MAX_DELAY // 超时时间
    );

    LSM6DSR_CS_HIGH(); // 拉高片选，结束通信

    // 检查传输状态
    if (status != HAL_OK) {
        return 2; // 传输失败
    }

    // 提取有效数据（跳过接收缓冲区的第1个字节，从第2个字节开始为寄存器数据）
    for (uint16_t i = 0; i < length; i++) {
        value[i] = rxData[1 + i];
    }

    return 0; // 读取成功
}

// 初始化LSM6DSR
void LSM6DSR_Init(void) {
    HAL_Delay(50);  // 等待传感器稳定
    
    uint8_t buffer =0x40;
    // 配置加速度计 (示例：104Hz, ±2g)
    LSM6DSR_WriteReg(0x10, &buffer,1); // CTRL1_XL
    
    // 配置陀螺仪 (示例：104Hz, ±250dps)
    buffer = 0x40;
    LSM6DSR_WriteReg(0x11, &buffer,1); // CTRL2_G
    
    HAL_Delay(10);  // 等待传感器准备好
    
    uint8_t who_am_i = 0;
    uint8_t opCode = LSM6DSR_ReadReg(0x0FU, &who_am_i,1); 
    
    if (who_am_i != 0x6B) {
        // 通信失败处理
        uart_printf("<通信失败，读取到ID：0x%02X，期望：0x6B，读取状态码：%d>", who_am_i, opCode);
        HAL_Delay(1000);
    } else {
        uart_printf("<LSM6DSR初始化成功，ID：0x%02X>", who_am_i);
    }

    // 可以根据需要使能其他功能，如FIFO、中断等
}

// 读取加速度计X轴原始数据（最简单版本）
/*
int16_t Read_Acceleration_X(void) {
    uint8_t low, high;
    LSM6DSR_ReadReg(0x28, &low,1);
    LSM6DSR_ReadReg(0x29, &high,1);
    return (int16_t)((high << 8) | low);
}
*/

void readAll_ADC_Channel(uint16_t value[16]){
  //ADC2在左侧，ADC1在右侧
  //0-15对应从左到右16个光电管

  uint16_t zero[16]={3149,3019,2932,2910,2950,2777,2792,3258,2831,2930,3115,2848,2831,2868,2916,3003};

  //ADC2的0123顺序画PCB画反了
  //至于为什么要把前两个移到末尾去，天知道
  value[0] = AD7689_Get_Data(2, 5);
  value[1] = AD7689_Get_Data(2, 4);
  value[2] = AD7689_Get_Data(2, 0);
  value[3] = AD7689_Get_Data(2, 1);
  value[4] = AD7689_Get_Data(2, 2);
  value[5] = AD7689_Get_Data(2, 3);
  value[6] = AD7689_Get_Data(2, 7);
  value[7] = AD7689_Get_Data(2, 6);
  
  value[8] = AD7689_Get_Data(1, 5);
  value[9] = AD7689_Get_Data(1, 4);
  value[10] = AD7689_Get_Data(1, 3);
  value[11] = AD7689_Get_Data(1, 2);
  value[12] = AD7689_Get_Data(1, 1);
  value[13] = AD7689_Get_Data(1, 0);
  value[14] = AD7689_Get_Data(1, 7);
  value[15] = AD7689_Get_Data(1, 6);

  for (int i=0; i<16; i++) {
    if(value[i]>(zero[i]-1000))value[i] -= (zero[i]-1000);
    else value[i] = 0;
  }
}

/**
  * @brief 根据16个光电管数据确定循迹线所在位置
  * @param value 传入16个光电管数据，0-15从左到右
  * @param pos 返回循迹线角度，中心90度，从右到左0-180度
  * @retval 循迹线存在状态，0不存在，1存在，2离地
   */
uint8_t calcLinePos(uint16_t value[16], int *angle) {
    int indices[16];
    int count = 0;
    // 初始化索引；同时判断超过4000的值的个数，大于10个视为离开地面，返回无循迹线
    for (int i = 0; i < 16; i++) {
      indices[i] = i;

      if(value[i]>2000)count++;
    }
    if(count>=10) return 2;
    
    // 冒泡排序索引数组
    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 15 - i; j++) {
            if (value[indices[j]] > value[indices[j + 1]]) {
                int temp = indices[j];
                indices[j] = indices[j + 1];
                indices[j + 1] = temp;
            }
        }
    }
    
    // 计算中间10个数值的平均数
    float sum = 0;
    for (int i = 3; i < 13; i++) sum += value[indices[i]];
    float average = sum / 10.0f;
    
    // 计算误差并保留绝对值更大的那个
    float min_error = ((float)value[indices[0]] - average) / average * 100.0f;
    float max_error = ((float)value[indices[15]] - average) / average * 100.0f;
    float max_abs_error = (fabsf(min_error) > fabsf(max_error)) ? min_error : max_error;
    
    // 如果最大绝对误差小于20%，返回false，不设置角度
    if (fabsf(max_abs_error) < 10.0f) {
        return 0;
    }
    
    // 计算加权角度，但只使用与平均值相差大于10%的数据
    float weighted_angle_sum = 0;
    float weight_sum = 0;
    int valid_count = 0;
    
    // 选择三个极值的索引
    int selected_indices[3];
    if (max_abs_error >= 0) {
        // 最大绝对误差来自最大值，取最大的三个数
        selected_indices[0] = indices[15];
        selected_indices[1] = indices[14];
        selected_indices[2] = indices[13];
    } else {
        // 最大绝对误差来自最小值，取最小的三个数
        selected_indices[0] = indices[0];
        selected_indices[1] = indices[1];
        selected_indices[2] = indices[2];
    }
    
    // 筛选有效数据：只使用与平均值相差大于10%的数据
    for (int i = 0; i < 3; i++) {
        int idx = selected_indices[i];
        float error = ((float)value[idx] - average) / average * 100.0f;
        
        // 只使用与平均值相差大于10%的数据
        if (fabsf(error) > 5.0f) {
            // 角度转换：索引0(最左)→180度，索引15(最右)→0度
            float angle = 180.0f - idx * (180.0f / 15.0f);
            weighted_angle_sum += angle * value[idx];
            weight_sum += value[idx];
            valid_count++;
        }
    }
    
    // 如果没有有效数据，返回false
    if (valid_count == 0) {
        return 0;
    }
    
    // 计算加权角度
    *angle = (int)(weighted_angle_sum / weight_sum);
    return 1;
}

void motorInit(){
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  // 电机1 IN1（PA8）
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);  // 电机1 IN2（PA9）
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);  // 电机2 IN1（PA10）
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);  // 电机2 IN2（PA11）
}
/**
  * @brief  双电机速度控制（正负控方向+默认快衰减）
  * @param  motor1_speed：电机1速度（-100~100）
  *         正值 = 逆时针旋转（CCW），负值 = 顺时针旋转（CW），0 = 停止
  * @param  motor2_speed：电机2速度（-100~100）
  *         正值 = 逆时针旋转（CCW），负值 = 顺时针旋转（CW），0 = 停止
  * @note   1. 硬件映射：左电机(1)=PA8(CH1)/PA9(CH2)，右电机(2)=PA10(CH3)/PA11(CH4)，TIM1为16位PWM（周期65535）
  *         2. 速度自动限幅（-100~100），超出部分强制钳位
  *         3. 默认快衰减模式（响应迅速，另一通道固定输出高电平65535）
  *         4. 速度=0时强制停止（两通道均为低电平）
  */
void setMotor(float motor1_speed, float motor2_speed)
{
    // ************************** 通用配置 **************************
    const uint16_t DECAY_FAST = 65535; // 默认快衰减（固定高电平）
    const float MAX_SPEED = 100.0f;     // 最大速度百分比

    // ************************** 左电机（通道1/2）处理 **************************
    // 速度限幅（-100~100）+ 取绝对值（用于计算PWM脉冲）
    float speed1_clamped = (motor1_speed > MAX_SPEED) ? MAX_SPEED : 
                           (motor1_speed < -MAX_SPEED) ? -MAX_SPEED : motor1_speed;
    float speed1_abs = (speed1_clamped >= 0) ? speed1_clamped : -speed1_clamped;
    uint16_t pulse1 = (uint16_t)(speed1_abs * 720.0f / MAX_SPEED);

    if (pulse1 == 0) { // 速度=0 → 停止（两通道低电平）
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    } else if (speed1_clamped > 0) { // 正值 → 逆时针（CCW）：CH2输出PWM，CH1快衰减（高电平）
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse1);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, DECAY_FAST);
    } else { // 负值 → 顺时针（CW）：CH1输出PWM，CH2快衰减（高电平）
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse1);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, DECAY_FAST);
    }

    // ************************** 右电机（通道3/4）处理 **************************
    float speed2_clamped = (motor2_speed > MAX_SPEED) ? MAX_SPEED : 
                           (motor2_speed < -MAX_SPEED) ? -MAX_SPEED : motor2_speed;
    float speed2_abs = (speed2_clamped >= 0) ? speed2_clamped : -speed2_clamped;
    uint16_t pulse2 = (uint16_t)(speed2_abs * 720.0f / MAX_SPEED);

    if (pulse2 == 0) { // 速度=0 → 停止
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    } else if (speed2_clamped > 0) { // 正值 → 逆时针（CCW）：CH4输出PWM，CH3快衰减（高电平）
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pulse2);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, DECAY_FAST);
    } else { // 负值 → 顺时针（CW）：CH3输出PWM，CH4快衰减（高电平）
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse2);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, DECAY_FAST);
    }
}

void Tracking(float baseSpeed, uint8_t lineState, int lineAngle){
  /*PID常量定义*/
  const float Kp = 0.01f;
  const float Ki = 0.0f;
  const float Kd = 1.0f;
  const float min_out = -5.0f;
  const float max_out = 5.0f;

  /*PID状态*/
  static int lastError = 0;
  static float integral = 0.0f; //积分累积值
  float p_out, i_out, d_out, total_out;

  /*PID计算*/
  int error = 90 - lineAngle;

  p_out = Kp * error;

  integral += error;
  // 积分限幅：避免积分累积超出输出范围（Ki≠0时才生效）
  if (Ki != 0.0f) {
    float integral_max = 5;
    float integral_min = -5;
    if (integral > integral_max) integral = integral_max;
    if (integral < integral_min) integral = integral_min;
  }
  i_out = Ki * integral;

  d_out = Kd * (error - lastError);

  total_out = p_out + i_out + d_out;
  if (total_out > max_out) total_out = max_out;
  if (total_out < min_out) total_out = min_out;

  /*循迹线判断相关变量*/
  static uint8_t lastState = 0;
  static uint32_t lastTime = 0;

  if(lineState==0){//未识别到循迹线，低速寻线1s
    if(lastState==1 && (HAL_GetTick()-lastTime < 1000)) setMotor(baseSpeed + (total_out * 0.5f), baseSpeed - (total_out * 0.5f));
    else {
      setMotor(0, 0);
      lastState = 0;
      lastError = 0;
      integral = 0;
    }
  }
  else if (lineState == 1) {//正常循迹
    setMotor(baseSpeed + total_out, baseSpeed -total_out);
    lastError = error;
    lastState = 1;
    lastTime = HAL_GetTick();
  }
  else if(lineState == 2) {//检测到离开地面，停止循迹
    setMotor(0, 0);
    lastState = 2;
    lastError = 0;
    integral = 0;
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  uart_printf("<test>");
  HAL_Delay(500);
  LSM6DSR_Init();
  AD7689_Init();
  motorInit();
  //setMotor(10, 10);
  HAL_Delay(200);
  //setMotor(0, 0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /*
    int16_t accel_x = Read_Acceleration_X();
    uart_printf("<X Acceleration: %d>", accel_x);*/
    uint16_t adcData[16];
    readAll_ADC_Channel(adcData);
    uint8_t lineExist;
    int lineAngle;
    lineExist = calcLinePos(adcData, &lineAngle);
    Tracking(0.1, lineExist, lineAngle);
    //setMotor(0.01f, 0.01f);
    
    /*
    uart_printf("<");
    for (int i =0; i<16; i++) {
    uart_printf("%d\n",adcData[i]);
    }
    uart_printf(">");*/
    //uart_printf("<循迹线是否存在：%d | 循迹线所在角度：%d | 当前tick：%d>", lineExist, (int)lineAngle, HAL_GetTick());

    static uint32_t cycleTime;
    
    //uart_printf("<cycle spend %d ms, current lineState: %d>",HAL_GetTick()-cycleTime, lineExist);
    //cycleTime = HAL_GetTick();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
    uart_printf("<init error>");
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
