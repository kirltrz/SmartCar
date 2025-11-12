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
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "custom_bus.h"

#include "UART.h"
#include "motion.h"
#include "IMU.h"
#include "opt.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
bool offGround = false;
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

/* 通过串口控制舵机 */
void uartCtrl(void)
{
    static uint8_t rx_buf[32] = {0};      // 接收缓冲区
    static uint8_t rx_index = 0;          // 缓冲区索引
    uint8_t rx_data;
    
    // 非阻塞读取单字节
    if (HAL_UART_Receive(&huart3, &rx_data, 1, 10) != HAL_OK)
        return;

    // 帧头检测：重置接收状态
    if (rx_data == '<') {
        rx_index = 0;
        rx_buf[rx_index++] = rx_data;
        return;
    }
    
    // 非帧头且缓冲区为空：丢弃数据
    if (rx_index == 0)
        return;
    
    // 缓冲区溢出保护
    if (rx_index >= sizeof(rx_buf)) {
        rx_index = 0;
        return;
    }
    
    // 存储数据
    rx_buf[rx_index++] = rx_data;
    
    // 帧尾检测：处理完整数据帧
    if (rx_data == '>') {
        // 直接解析并执行电机控制
        int8_t left_speed = 0, right_speed = 0;
        uint8_t parse_success = 0;
        
        // 基本格式校验
        if (rx_index >= 6 && rx_buf[0] == '<' && rx_buf[rx_index-1] == '>') {
            // 查找逗号分隔符
            uint8_t comma_pos = 0;
            for (uint8_t i = 1; i < rx_index-1; i++) {
                if (rx_buf[i] == ',') {
                    comma_pos = i;
                    break;
                }
            }
            
            // 提取并转换速度值
            if (comma_pos > 1 && comma_pos < rx_index-2) {
                // 左速度（跳过可能的'L'前缀）
                uint8_t left_start = (rx_buf[1] == 'L') ? 2 : 1;
                char left_str[8] = {0};
                uint8_t left_len = comma_pos - left_start;
                if (left_len > 0 && left_len < sizeof(left_str)) {
                    memcpy(left_str, &rx_buf[left_start], left_len);
                    int left_val = atoi(left_str);
                    
                    // 右速度（跳过可能的'R'前缀）
                    uint8_t right_start = comma_pos + 1;
                    if (rx_buf[right_start] == 'R') right_start++;
                    char right_str[8] = {0};
                    uint8_t right_len = (rx_index - 1) - right_start;
                    
                    if (right_len > 0 && right_len < sizeof(right_str)) {
                        memcpy(right_str, &rx_buf[right_start], right_len);
                        int right_val = atoi(right_str);
                        
                        // 验证速度范围
                        if (left_val >= -100 && left_val <= 100 && 
                            right_val >= -100 && right_val <= 100) {
                            left_speed = (int8_t)left_val;
                            right_speed = (int8_t)right_val;
                            parse_success = 1;
                        }
                    }
                }
            }
        }
        
        // 执行电机控制
        if (parse_success) {
            setMotor((float)left_speed, (float)right_speed);
        } else {
            setMotor(0.0f, 0.0f);  // 解析失败停止电机
        }
        
        rx_index = 0;  // 重置接收状态
    }
}

void Tracking(float baseSpeed, uint8_t lineCount, int lineAngle[4], IMU_Data_t *IMU_Data){
    /*PID常量定义*/
    const float Kp = 1.0f;
    const float Ki = 0.0f;
    const float Kd = 0.0f;
    const float min_out = -50.0f;
    const float max_out = 50.0f;

    /*PID状态*/
    static int lastError = 0;
    static float integral = 0.0f; //积分累积值
    float p_out, i_out, d_out, total_out;

    /*PID计算*/
    int error = 0;

    //首先判断目标线，根据比赛地图设置
    uint8_t targetLineIndex;
    uint8_t targetLineAngle;
    static uint8_t lastTargetLineAngle = 90;
    if (lineCount==0) {
        targetLineAngle = lastTargetLineAngle;
    }else if (lineCount == 1) {
        targetLineIndex = 0; //一条线直接走
        targetLineAngle = lineAngle[targetLineIndex];
        lastTargetLineAngle = targetLineAngle;
    }else if (lineCount == 2) {
        targetLineAngle = lastTargetLineAngle;
    }else if (lineCount ==3) {
        targetLineIndex = 1; //三条线为十字路口，走中间的线
        targetLineAngle = lineAngle[targetLineIndex];
        lastTargetLineAngle = targetLineAngle;
    }else{
        targetLineAngle = lastTargetLineAngle;
    }

    error = 90 - targetLineAngle;

    p_out = error * Kp;

    i_out = 0.0f;

    d_out = Kd * (error - lastError);

    total_out = p_out + i_out + d_out;

    setAndUpdateMotion(baseSpeed, IMU_Data->attitudeAngles[2] + (90 - targetLineAngle), IMU_Data);

    lastError = error;
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
  /* USER CODE BEGIN 2 */

  BSP_SPI2_Init(); //IMU SPI初始化
  
  initIMU();
  initOpt(); //光电管ADC芯片初始化
  initMotor();

  /*
  setMotor(10, 10);
  HAL_Delay(200);
  setMotor(0, 0);
  */

  caliOpt(); //校准光电传感器
  caliIMU(); //校准IMU
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  static bool speedEnabled = true;        // 当前是否允许输出线速度
  static uint32_t lastToggleTime = 0;     // 上次切换时间
  const uint32_t ON_TIME_MS  = 80;        // 每次加速 80ms
  const uint32_t OFF_TIME_MS = 80;        // 然后滑行 40ms（可调）
  const float MAX_BASE_SPEED = 0.4f;     // 最大设定速度
  while (1)
  {
    /**/
    setLED(false);

    
    //uartCtrl(); //串口控制电机
    IMU_Data_t IMU_Data;
    getIMU(&IMU_Data); //读取IMU数据

    //离地检测，防止空转
    if(IMU_Data.attitudeAngles[0]>10.0f||IMU_Data.attitudeAngles[0]<-10.0f
      ||IMU_Data.attitudeAngles[1]>10.0f||IMU_Data.attitudeAngles[1]<-10.0f)
    {
      offGround = true;
      setMotor(0.0f, 0.0f);
      uart_printf("<offGround>");
    }else {
      if(offGround) uart_printf("<onGround>");
      offGround = false;

      // 周期性启停线速度（仅在落地时生效）
        uint32_t now = HAL_GetTick();
        uint32_t elapsed = now - lastToggleTime;

        if (speedEnabled) {
            if (elapsed >= ON_TIME_MS) {
                speedEnabled = false;
                lastToggleTime = now;
            }
        } else {
            if (elapsed >= OFF_TIME_MS) {
                speedEnabled = true;
                lastToggleTime = now;
            }
        }
    }
  //  setAndUpdateMotion(0.0f, -10.0f, &IMU_Data);
    // IMU数据输出调试
    uint8_t lineCount;
    int lineAngle[4];
    lineCount = updateOpt(false, lineAngle); //识别循迹线
    //HAL_Delay(1000);
    // 如果离地，强制速度为0；否则按脉冲使能
    float baseSpeed = 0.0f;
    if (!offGround && speedEnabled) {
        baseSpeed = MAX_BASE_SPEED;
    }
    Tracking(baseSpeed, lineCount, lineAngle, &IMU_Data); //循迹逻辑
    
    /*
    uart_printf("<accX: %d, accY: %d, accZ: %d, \ngyroX: %d, gyroY: %d, gyroZ: %d, \npitch: %d, roll: %d, yaw: %d,\n linearVelocity: %d>",
            (int)(IMU_Data.acceleration_mg[0] * 1.00),    // 加速度X轴（mg）
            (int)(IMU_Data.acceleration_mg[1] * 1.00),    // 加速度Y轴（mg）
            (int)(IMU_Data.acceleration_mg[2] * 1.00),    // 加速度Z轴（mg）
            (int)(IMU_Data.angular_rate_dps[0] * 1.00),   // 角速度X轴（°/s）
            (int)(IMU_Data.angular_rate_dps[1] * 1.00),   // 角速度Y轴（°/s）
            (int)(IMU_Data.angular_rate_dps[2] * 1.00),   // 角速度Z轴（°/s）
            (int)(IMU_Data.attitudeAngles[0] * 1.00),     // 姿态角Pitch（°）
            (int)(IMU_Data.attitudeAngles[1] * 1.00),     // 姿态角Roll（°）
            (int)(IMU_Data.attitudeAngles[2] * 1.00),     // 姿态角Yaw（°）
            (int)(IMU_Data.linearVelocity * 100)         // 线速度（cm/s）
           );
*/
    //循环时间测试
    /*
    static uint32_t cycleTime = 0;
    uart_printf("<cycle spend %d ms, current lineState: %d>",HAL_GetTick()-cycleTime, lineExist);
    cycleTime = HAL_GetTick();
    */

    setLED(true);
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
