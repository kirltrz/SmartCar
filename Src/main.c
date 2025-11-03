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

  // AD7689各通道配置（关键修改：选择内部2.5V基准）
  // 配置说明：bit10-9=01（内部2.5V基准），bit8=0（单极性），bit7-4=1111（全带宽），bit3=0（禁用序列器），bit2=0（不回读CFG）
  IN_DAT[0] = (0x3849 << 2);  // 通道0（内部2.5V基准配置）
  IN_DAT[1] = (0x38C9 << 2);  // 通道1
  IN_DAT[2] = (0x3949 << 2);  // 通道2
  IN_DAT[3] = (0x39C9 << 2);  // 通道3
  IN_DAT[4] = (0x3A49 << 2);  // 通道4
  IN_DAT[5] = (0x3AC9 << 2);  // 通道5
  IN_DAT[6] = (0x3B49 << 2);  // 通道6
  IN_DAT[7] = (0x3BC9 << 2);  // 通道7

  // 初始化所有8通道的零点偏置数据（2.5V基准下需重新校准！）
  bias_data[0] = BIAS_VOLTAGE_IN0;
  bias_data[1] = BIAS_VOLTAGE_IN1;
  bias_data[2] = BIAS_VOLTAGE_IN2;
  bias_data[3] = BIAS_VOLTAGE_IN3;
  bias_data[4] = BIAS_VOLTAGE_IN4;
  bias_data[5] = BIAS_VOLTAGE_IN5;
  bias_data[6] = BIAS_VOLTAGE_IN6;
  bias_data[7] = BIAS_VOLTAGE_IN7;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /*
    int16_t accel_x = Read_Acceleration_X();
    uart_printf("<X Acceleration: %d>", accel_x);*/
    // 1. 采集8通道AD原始数据
    uint16_t ad7689_cfg[8] = {
      IN_DAT[2], IN_DAT[3], IN_DAT[4], IN_DAT[5],
      IN_DAT[6], IN_DAT[7],IN_DAT[0], IN_DAT[1]
    };
    int i;
    for(i = 0; i < 8; i++){
      Conve_data[i] = AD7689_Get_Data(ad7689_cfg[i]);  // 读取AD原始值
      HAL_Delay(1);
    }

    // 2. 基于2.5V基准计算电压（单位：mV）
    uart_printf("<");
    
    for(i = 0; i < 8; i++){				
      // 公式说明：
      // (AD值 - 偏置AD值)：去除零点误差
      // * REFERENCE_VOLTAGE：乘以基准电压（2500mV）
      // / 0xFFFF：除以AD满量程（16位AD，最大值65535）
      // * (OPA_RES_R1 + OPA_RES_R2)/OPA_RES_R2：运放放大倍数（(R1+R2)/R2）
      // 简化后公式与下方一致
      /*
      voltage_data[i] = (Conve_data[i] - bias_data[i]) 
                      * REFERENCE_VOLTAGE / OPA_RES_R2 
                      * (OPA_RES_R1 + OPA_RES_R2) / 0xFFFF;*/
      uart_printf("%d\n", Conve_data[i]);
    }
    uart_printf(">");

    // 3. 延时控制采集频率
    HAL_Delay(100);
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
