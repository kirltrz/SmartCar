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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"  // 字符串处理（strchr、memset等）
#include "stdlib.h"  // 数值转换（atoi）
#include "stdio.h"   // 调试打印
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// 电机控制模式枚举
typedef enum {
    MOTOR_STOP = 0,     // 停止
    MOTOR_FORWARD,      // 前进
    MOTOR_BACKWARD,     // 后退
    MOTOR_BRAKE         // 刹车
} Motor_State;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_GPIO_Port GPIOC  // LED接GPIOC（常用PC13，可按需修改）
#define LED_Pin GPIO_PIN_13
#define AD_CS2_Pin GPIO_PIN_4 // 原AD芯片CS引脚，保持不变
#define AD_CS1_Pin GPIO_PIN_5
#define SW_Pin GPIO_PIN_0     // 按键引脚，按需修改
#define SW_GPIO_Port GPIOA

// 串口接收配置
#define UART_RX_BUF_SIZE 32  // 接收缓冲区大小（足够存储最长帧：<L100,R-100>）
#define UART_TIMEOUT 10      // 串口读取超时（ms）
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
// 轮询接收相关变量
uint8_t uart3_rx_buf[UART_RX_BUF_SIZE] = {0};  // 接收缓冲区
uint8_t rx_index = 0;                          // 缓冲区索引（记录当前接收位置）
uint8_t uart_rx_data = 0;                      // 单次读取的字节数据

// 电机速度变量（-100~100，对应左/右电机）
int8_t left_motor_speed = 0;       // 左电机（电机1：PA8/PA9）
int8_t right_motor_speed = 0;      // 右电机（电机2：PA10/PA11）
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
// 自定义函数原型
uint8_t parse_uart_motor_data(uint8_t *buf, int8_t *left, int8_t *right);
void setMotorSpeed(uint8_t motor_num, uint8_t speed_percent);
void setMotorState(uint8_t motor_num, Motor_State state, uint8_t speed_percent);
void carForward(uint8_t speed_percent);
void carBackward(uint8_t speed_percent);
void carStop(void);
void carBrake(void);
void carTurnLeft(uint8_t speed_percent);
void carTurnRight(uint8_t speed_percent);
void smoothAccelerate(uint8_t target_speed, uint16_t duration_ms);
void smoothDecelerate(uint16_t duration_ms);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
  MX_SPI2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  // 1. 启动TIM1的PWM输出（电机控制核心）
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  // 电机1 IN1（PA8）
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);  // 电机1 IN2（PA9）
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);  // 电机2 IN1（PA10）
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);  // 电机2 IN2（PA11）

  // 2. 初始化电机为停止状态
  carStop();
  HAL_Delay(100);  // 稳定延时

  // 3. 调试提示：系统初始化完成（串口打印）
  char init_msg[] = "System Init OK. Polling UART...\r\n";
  HAL_UART_Transmit(&huart3, (uint8_t*)init_msg, strlen(init_msg), 500);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // ------------------- 核心：轮询接收串口数据 -------------------
    // 检查USART3是否有数据可读（非阻塞，快速返回）
    if (HAL_UART_GetState(&huart3) == HAL_UART_STATE_READY)
    {
      // 读取1字节数据（超时UART_TIMEOUT ms，避免阻塞）
      if (HAL_UART_Receive(&huart3, &uart_rx_data, 1, UART_TIMEOUT) == HAL_OK)
      {
        // 1. 检测帧头`<`：重置缓冲区（从帧头开始接收，丢弃之前的无效数据）
        if (uart_rx_data == '<')
        {
          rx_index = 0;
          memset(uart3_rx_buf, 0, sizeof(uart3_rx_buf));  // 清空缓冲区
          uart3_rx_buf[rx_index++] = uart_rx_data;        // 保存帧头
        }
        // 2. 已收到帧头，继续接收后续数据（直到帧尾`>`或缓冲区满）
        else if (rx_index > 0)
        {
          uart3_rx_buf[rx_index++] = uart_rx_data;  // 存入缓冲区，索引自增

          // 3. 检测帧尾`>`：帧接收完整，触发解析
          if (uart_rx_data == '>')
          {
            // ------------------- 1. 回显完整数据（调试用） -------------------
            uint8_t echo_len = rx_index;  // 接收长度=当前索引（包含`<`和`>`）
            // 兼容ESP8266可能的\r\n，去除末尾无效字符
            while (echo_len > 0 && (uart3_rx_buf[echo_len-1] == '\r' || uart3_rx_buf[echo_len-1] == '\n'))
            {
              echo_len--;
            }
            if (echo_len > 0)
            {
              HAL_UART_Transmit(&huart3, uart3_rx_buf, echo_len, 100);  // 回显数据
              HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", 2, 50);      // 换行便于查看
            }

            // ------------------- 2. 解析数据帧并控制电机 -------------------
            if (parse_uart_motor_data(uart3_rx_buf, &left_motor_speed, &right_motor_speed) == 1)
            {
              // 左电机控制
              if (left_motor_speed > 0)
                setMotorState(1, MOTOR_FORWARD, left_motor_speed);
              else if (left_motor_speed < 0)
                setMotorState(1, MOTOR_BACKWARD, -left_motor_speed);
              else
                setMotorState(1, MOTOR_STOP, 0);

              // 右电机控制
              if (right_motor_speed > 0)
                setMotorState(2, MOTOR_FORWARD, right_motor_speed);
              else if (right_motor_speed < 0)
                setMotorState(2, MOTOR_BACKWARD, -right_motor_speed);
              else
                setMotorState(2, MOTOR_STOP, 0);

              // LED亮：解析成功
              HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
              // 调试打印：解析结果
              char ok_msg[32];
              sprintf(ok_msg, "Parse OK: Left=%d, Right=%d\r\n", left_motor_speed, right_motor_speed);
              HAL_UART_Transmit(&huart3, (uint8_t*)ok_msg, strlen(ok_msg), 100);
            }
            else
            {
              // 解析失败：停止电机，LED灭
              carStop();
              HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
              HAL_UART_Transmit(&huart3, (uint8_t*)"Parse Failed! Stop Motor.\r\n", 26, 100);
            }

            // ------------------- 3. 重置缓冲区，准备下一次接收 -------------------
            rx_index = 0;
            memset(uart3_rx_buf, 0, sizeof(uart3_rx_buf));
          }
          // 4. 缓冲区溢出保护：超过最大长度时重置
          else if (rx_index >= UART_RX_BUF_SIZE)
          {
            rx_index = 0;
            memset(uart3_rx_buf, 0, sizeof(uart3_rx_buf));
            HAL_UART_Transmit(&huart3, (uint8_t*)"RX Buf Overflow!\r\n", 18, 50);
          }
        }
        // （else：未收到帧头，丢弃当前字节，不处理）
      }
    }

    // ------------------- 系统状态指示：LED慢闪（无数据处理时） -------------------
    if (rx_index == 0)  // 仅当未接收数据时翻转LED（避免解析成功时LED闪烁）
    {
      HAL_Delay(500);
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }

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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;  // HSE=8MHz → PLL=72MHz
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;  // SYSCLK=72MHz
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;         // HCLK=72MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;         // APB1=36MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;         // APB2=72MHz

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{
  /* USER CODE BEGIN SPI1_Init 0 */
  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */
  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;  // 18MHz
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */
  /* USER CODE END SPI1_Init 2 */
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{
  /* USER CODE BEGIN SPI2_Init 0 */
  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */
  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;  // 18MHz
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */
  /* USER CODE END SPI2_Init 2 */
}

/**
  * @brief TIM1 Initialization Function（电机PWM）
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{
  /* USER CODE BEGIN TIM1_Init 0 */
  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */
  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;  // 不分频（72MHz）
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;  // 16位PWM（最高分辨率）
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;  // PWM模式1
  sConfigOC.Pulse = 0;  // 初始占空比0
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;  // 高电平有效
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  // 配置4个PWM通道
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) { Error_Handler(); }
  // 死区配置（DRV8833无需死区）
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);  // 引脚初始化
}

/**
  * @brief USART3 Initialization Function（ESP8266通信）
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{
  /* USER CODE BEGIN USART3_Init 0 */
  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */
  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;  // 与ESP8266一致
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;  // 收发模式
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
  /* USER CODE END USART3_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();  // USART3（PB10/PB11）

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);  // LED初始灭

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AD_CS2_Pin|AD_CS1_Pin, GPIO_PIN_RESET);  // AD_CS初始低

  /*Configure GPIO pin : LED_Pin（PC13） */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SW_Pin（PA0） */
  GPIO_InitStruct.Pin = SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : AD_CS2_Pin AD_CS1_Pin（PA4/PA5） */
  GPIO_InitStruct.Pin = AD_CS2_Pin|AD_CS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  解析ESP8266发送的`<Lx,Ry>`格式数据帧
  * @param  buf：接收缓冲区
  * @param  left：左电机速度（输出）
  * @param  right：右电机速度（输出）
  * @retval 1=解析成功，0=解析失败
  */
uint8_t parse_uart_motor_data(uint8_t *buf, int8_t *left, int8_t *right)
{
  char *frame_start = strchr((char*)buf, '<');  // 找帧头`<`
  char *frame_end = strchr((char*)buf, '>');    // 找帧尾`>`
  char *comma_split;                            // 找逗号分割符

  // 步骤1：校验帧格式（必须包含`<`和`>`，且帧头在帧尾前）
  if (frame_start == NULL || frame_end == NULL || frame_start >= frame_end)
  {
    return 0;
  }

  // 步骤2：提取帧内数据（去除`<`、`>`和末尾\r\n）
  frame_start++;
  *frame_end = '\0';  // 帧尾设为结束符
  char *last_char = frame_end - 1;
  while (last_char >= frame_start && (*last_char == '\r' || *last_char == '\n'))
  {
    *last_char = '\0';  // 删除换行符
    last_char--;
  }

  // 步骤3：用逗号分割左右速度
  comma_split = strchr(frame_start, ',');
  if (comma_split == NULL || comma_split >= frame_end)
  {
    return 0;
  }

  // 步骤4：跳过`L`/`R`前缀，提取速度值
  *comma_split = '\0';
  char *left_str = frame_start;
  if (*left_str == 'L') left_str++;  // 左速度跳过`L`
  char *right_str = comma_split + 1;
  if (*right_str == 'R') right_str++;  // 右速度跳过`R`

  // 步骤5：转换并校验速度范围（-100~100）
  int temp_left = atoi(left_str);
  int temp_right = atoi(right_str);
  if (temp_left < -100 || temp_left > 100 || temp_right < -100 || temp_right > 100)
  {
    return 0;
  }

  // 步骤6：赋值输出
  *left = (int8_t)temp_left;
  *right = (int8_t)temp_right;
  return 1;
}

/**
  * @brief  设置电机PWM速度（简化接口，仅前进）
  * @param  motor_num：电机编号（1=左，2=右）
  * @param  speed_percent：速度百分比（0~100）
  */
void setMotorSpeed(uint8_t motor_num, uint8_t speed_percent)
{
  uint16_t pulse = (uint16_t)((speed_percent > 100 ? 100 : speed_percent) * 65535 / 100);
  switch(motor_num)
  {
    case 1:
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
      break;
    case 2:
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
      break;
  }
}

/**
  * @brief  电机状态与速度控制（核心函数）
  * @param  motor_num：电机编号（1=左，2=右）
  * @param  state：电机状态（停止/前进/后退/刹车）
  * @param  speed_percent：速度百分比（0~100）
  */
void setMotorState(uint8_t motor_num, Motor_State state, uint8_t speed_percent)
{
  uint16_t pulse = (uint16_t)((speed_percent > 100 ? 100 : speed_percent) * 65535 / 100);
  switch(motor_num)
  {
    case 1:  // 左电机（PA8=IN1，PA9=IN2）
      switch(state)
      {
        case MOTOR_STOP:
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
          break;
        case MOTOR_FORWARD:
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
          break;
        case MOTOR_BACKWARD:
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse);
          break;
        case MOTOR_BRAKE:
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 65535);
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 65535);
          break;
      }
      break;

    case 2:  // 右电机（PA10=IN1，PA11=IN2）
      switch(state)
      {
        case MOTOR_STOP:
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
          break;
        case MOTOR_FORWARD:
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse);
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
          break;
        case MOTOR_BACKWARD:
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pulse);
          break;
        case MOTOR_BRAKE:
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 65535);
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 65535);
          break;
      }
      break;
  }
}

/**
  * @brief  小车前进（双电机同步）
  * @param  speed_percent：速度百分比（0~100）
  */
void carForward(uint8_t speed_percent)
{
  setMotorState(1, MOTOR_FORWARD, speed_percent);
  setMotorState(2, MOTOR_FORWARD, speed_percent);
}

/**
  * @brief  小车后退（双电机同步）
  * @param  speed_percent：速度百分比（0~100）
  */
void carBackward(uint8_t speed_percent)
{
  setMotorState(1, MOTOR_BACKWARD, speed_percent);
  setMotorState(2, MOTOR_BACKWARD, speed_percent);
}

/**
  * @brief  小车停止
  */
void carStop(void)
{
  setMotorState(1, MOTOR_STOP, 0);
  setMotorState(2, MOTOR_STOP, 0);
}

/**
  * @brief  小车刹车（快速停稳）
  */
void carBrake(void)
{
  setMotorState(1, MOTOR_BRAKE, 0);
  setMotorState(2, MOTOR_BRAKE, 0);
  HAL_Delay(100);
  carStop();
}

/**
  * @brief  小车左转（左轮退，右轮进）
  * @param  speed_percent：基础速度（0~100）
  */
void carTurnLeft(uint8_t speed_percent)
{
  setMotorState(1, MOTOR_BACKWARD, speed_percent / 2);
  setMotorState(2, MOTOR_FORWARD, speed_percent);
}

/**
  * @brief  小车右转（右轮退，左轮进）
  * @param  speed_percent：基础速度（0~100）
  */
void carTurnRight(uint8_t speed_percent)
{
  setMotorState(1, MOTOR_FORWARD, speed_percent);
  setMotorState(2, MOTOR_BACKWARD, speed_percent / 2);
}

/**
  * @brief  平滑加速（从0到目标速度）
  * @param  target_speed：目标速度（0~100）
  * @param  duration_ms：加速时长（ms）
  */
void smoothAccelerate(uint8_t target_speed, uint16_t duration_ms)
{
  if (target_speed == 0 || duration_ms == 0) return;
  uint8_t current_speed = 0;
  uint16_t step_delay = duration_ms / target_speed;
  while (current_speed < target_speed)
  {
    current_speed++;
    carForward(current_speed);
    HAL_Delay(step_delay);
  }
}

/**
  * @brief  平滑减速（从当前速度到0）
  * @param  duration_ms：减速时长（ms）
  */
void smoothDecelerate(uint16_t duration_ms)
{
  uint8_t current_speed = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1) * 100 / 65535;
  if (current_speed == 0 || duration_ms == 0) return;
  uint16_t step_delay = duration_ms / current_speed;
  while (current_speed > 0)
  {
    current_speed--;
    carForward(current_speed);
    HAL_Delay(step_delay);
  }
  carStop();
}
/* USER CODE END 4 */

/**
  * @brief  错误处理函数
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  // 错误指示：LED快速闪烁
  while (1)
  {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    HAL_Delay(100);  // 100ms翻转一次
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  断言失败处理
  * @param  file：文件名
  * @param  line：行号
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  char err_msg[64];
  sprintf(err_msg, "Assert Failed: File=%s, Line=%d\r\n", file, line);
  HAL_UART_Transmit(&huart3, (uint8_t*)err_msg, strlen(err_msg), 1000);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */