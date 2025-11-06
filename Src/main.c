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
#include "custom_bus.h"
#include "spi.h"
#include "stm32f1xx_hal.h"
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

#include "UART.h"
#include "ad7689.h"
#include "IMU.h"
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
void setMotor(float, float);

void setLED(bool on){
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/**
  * @brief  串口电机控制处理（接收、解析、执行）
  * @param  无
  * @retval 无
  */
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

void readAll_ADC_Channel(uint16_t value[16]){
  //ADC2在左侧，ADC1在右侧
  //0-15对应从左到右16个光电管

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
}

/**
   */
void caliLineValue(uint16_t caliValueMax[16], uint16_t caliValueMin[16]){
	uint32_t startCaliTime = HAL_GetTick();
	const uint32_t caliTime = 5000;//5s

    // 初始化校准数组	
	for(int i = 0; i < 16; i++){
        caliValueMax[i] = 0;      // 初始化为最小值
        caliValueMin[i] = 0xFFFF; // 初始化为最大值
    }
    while(HAL_GetTick() - startCaliTime < caliTime){
        uint16_t value[16];
        readAll_ADC_Channel(value);
        for(int i=0;i<16;i++){
            if(value[i]>caliValueMax[i] && (value[i]!=0 && value[i]!=0xFFFF)) caliValueMax[i] = value[i];
            if(value[i]<caliValueMin[i] && (value[i]!=0 && value[i]!=0xFFFF)) caliValueMin[i] = value[i];
        }
        setLED(1);
        HAL_Delay(25);
        setLED(0);
        HAL_Delay(25);
    }
}
/**
  * @brief 根据16个光电管数据确定循迹线所在位置
  * @param activeHighOrLow 高值为线or低值为线
  * @param value 传入16个光电管数据，0-15从左到右
  * @param angle 返回每条循迹线的角度，中心90度，从右到左0-180度
  * @param caliValueMax 校准获得的高值数据
  * @param caliValueMin 校准获得的低值数据
  * @retval 循迹线数量，
   */
uint8_t calcLinePos(bool activeHighOrLow, uint16_t value[16], int angle[4], uint16_t caliValueMax[16], uint16_t caliValueMin[16]) {
    //对传感器数值进行归一化
    float valueAfterNormalized[16];
    for(int i = 0; i < 16; i++){
        valueAfterNormalized[i] = (float)(value[i]-caliValueMin[i]) / (float)(caliValueMax[i]-caliValueMin[i]);
    }

    //二值化
    const float threshold = 0.5f;
    bool valueAfterBinarized[16];//true为有循迹线
    for (int i = 0; i < 16; i++) {
        if(activeHighOrLow == 1){
            if(valueAfterNormalized[i] > threshold) valueAfterBinarized[i] = true;
            else valueAfterBinarized[i] = false;
        }else {
            if(valueAfterNormalized[i] < threshold) valueAfterBinarized[i] = true;
            else valueAfterBinarized[i] = false;
        }
    }

    //判断循迹线数量
    uint8_t lineCount=0;
    uint8_t lineStart[5],lineEnd[5]; //假设最多4根线,索引1-4
    bool inLine = false;

    for (int i = 0; i < 16; i++) {
        if(valueAfterBinarized[i] && !inLine) {
            // 检测到线的开始
            lineCount++;
            lineStart[lineCount] = i;
            inLine = true;
        }
        else if(!valueAfterBinarized[i] && inLine) {
            // 检测到线的结束
            lineEnd[lineCount] = i-1;
            inLine = false;
        }
    }
    // 处理最后一个传感器在线上情况
    if(inLine) {
        lineEnd[lineCount] = 15;
    }
    
    // 计算每根线的加权平均角度
    for(int lineIdx = 1; lineIdx <= lineCount; lineIdx++) {
        float weightedSum = 0;
        float weightSum = 0;
        
        // 对当前线进行加权平均计算
        for(int i = lineStart[lineIdx]; i <= lineEnd[lineIdx]; i++) {
            float weight = valueAfterNormalized[i]; // 使用归一化值作为权重
            weightedSum += i * weight;
            weightSum += weight;
        }
        
        if(weightSum > 0) {
            // 计算该线的加权中心位置
            float lineCenter = weightedSum / weightSum;
            // 转换为角度：传感器0-15对应角度180-0度
            angle[lineIdx-1] = (int)((15.0f - lineCenter) * 12.0f);
        } else {
            angle[lineIdx-1] = 90; // 默认角度
        }
    }
    return lineCount;
}

void motorInit(){
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
    // 左电机：通道2为PWM，通道1为衰减（逆时针时）
    setSingleMotor(&htim1, TIM_CHANNEL_2, TIM_CHANNEL_1, motor1_speed);
    
    // 右电机：通道4为PWM，通道3为衰减（逆时针时）
    setSingleMotor(&htim1, TIM_CHANNEL_4, TIM_CHANNEL_3, motor2_speed);
}

void Tracking(float baseSpeed, uint8_t lineCount, int lineAngle[4]){
    /*PID常量定义*/
    const float Kp = 1.2f;
    const float Ki = 0.0f;
    const float Kd = 5.0f;
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
    if (lineCount==0) {

    }else if (lineCount == 1) {
        targetLineIndex = 0; //一条线直接走
    }else if (lineCount == 2) {
        return; //目前仅在十字路口出现瞬间此情况，直接返回等待三线时判断
    }else if (lineCount ==3) {
        targetLineIndex = 1; //三条线为十字路口，走中间的线
    }else if (lineCount == 4){
        //暂无此情况
    }

    error = 90 - lineAngle[targetLineIndex];

    p_out = error * Kp;

    d_out = Kd * (error - lastError);

    total_out = p_out + i_out + d_out;

    setMotor(baseSpeed + total_out, baseSpeed -total_out);

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
  IMU_Init();

  AD7689_Init(); //光电管ADC芯片初始化
  motorInit();


  //setMotor(10, 10);
  HAL_Delay(200);
  //setMotor(0, 0);
  uint16_t caliValueMax[16], caliValueMin[16];
  //caliLineValue(caliValueMax, caliValueMin);

  //setMotor(50, 50);
  //HAL_Delay(3000);
  //setMotor(0, 0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /*

    uint16_t adcData[16];
    readAll_ADC_Channel(adcData);
    uint8_t lineCount;
    int lineAngle[4];
    lineCount = calcLinePos(true,adcData, lineAngle, caliValueMax, caliValueMin);

    uartCtrl();
    Tracking(40.0f, lineCount, lineAngle);
    //setMotor(0.01f, 0.01f);
*/
//uart_printf("<%d>",HAL_GetTick());
    lsm6dsr_read_angle_data_polling();

    /*
    uart_printf("<");
    for (int i =0; i<16; i++) {
    uart_printf("%d\n",adcData[i]);
    }
    uart_printf(">");*/
    //uart_printf("<循迹线是否存在：%d | 循迹线所在角度：%d | 当前tick：%d>", lineExist, (int)lineAngle, HAL_GetTick());

    //static uint32_t cycleTime;
    
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
