#pragma once

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include <stdint.h>


/* 类型定义 ------------------------------------------------------------------*/
/* 宏定义 --------------------------------------------------------------------*/
#define AD7689_SPIx                 SPI2
#define AD7689_SPIx_CLK_ENABLE()    __HAL_RCC_SPI2_CLK_ENABLE()
#define AD7689_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOB_CLK_ENABLE()

#define AD7689_CS_Pin               GPIO_PIN_12
#define AD7689_CS_Port            	GPIOB

#define AD7689_SCK_Pin              GPIO_PIN_13
#define AD7689_SCK_Port             GPIOB

#define AD7689_MISO_Pin             GPIO_PIN_14
#define AD7689_MISO_Port            GPIOB

#define AD7689_MOSI_Pin             GPIO_PIN_15
#define AD7689_MOSI_Port            GPIOB


#define AD_CS1_HIGH()           HAL_GPIO_WritePin(AD_CS1_GPIO_Port,AD_CS1_Pin,GPIO_PIN_SET);
#define AD_CS1_LOW()            HAL_GPIO_WritePin(AD_CS1_GPIO_Port,AD_CS1_Pin,GPIO_PIN_RESET);

/* AD7689 Register Map */
//#define IN0             (0x3c49 << 2)           // 单极性，全带宽，内部基准4.096V，禁用通道序列器，不回读CFG
//#define IN1             (0x3cc9 << 2)           
//#define IN2             (0x3d49 << 2)  
//#define IN3             (0x3dc9 << 2)  

#define Chanal                8                      // channel num  

/* 私有类型定义 --------------------------------------------------------------*/
typedef enum {
  Conve_False=0,
	Conve_True=1,
}Runstate_TypDef;

/* 私有宏定义 ----------------------------------------------------------------*/
#define AD7689_Speed            10000   // 采集模块获取速率，值越高越慢，实测最小值为20K 单通道，也就是值为5
                                       // 当10000时，大约为1s获取一次八通道值

#define OPA_RES_R1              6800  // 6.8k 运放输入端电阻
#define OPA_RES_R2              2400  // 2k 运放反馈电阻
#define REFERENCE_VOLTAGE       2500  // 参考电压（放大1000倍）
//#define SAMPLE_RESISTANCE       150   // 电流采样电阻


#define BIAS_VOLTAGE_IN0        0x800  // 2.5V基准下的零点偏置（示例值，需实际校准）
#define BIAS_VOLTAGE_IN1        0x800  
#define BIAS_VOLTAGE_IN2        0x800  
#define BIAS_VOLTAGE_IN3        0x800  
#define BIAS_VOLTAGE_IN4        0x800  
#define BIAS_VOLTAGE_IN5        0x800  
#define BIAS_VOLTAGE_IN6        0x800  
#define BIAS_VOLTAGE_IN7        0x800

/* 私有变量 ------------------------------------------------------------------*/
extern uint16_t     IN_DAT[Chanal];             //单极性，全带宽，内部基准4.096，禁用通道序列器，不回读CFG
extern __IO double  voltage_data[8];            // 电压值（单位：mV）
extern __IO double  current_data[8];            // 电流值（单位：uA）
extern __IO int32_t bias_data[8];               // 零点电压的AD转换结果
extern __IO uint8_t Conve_flag;	 // 滤波结果
extern uint16_t Conve_data[8];          // 滤波数值		
  
/* 扩展变量 ------------------------------------------------------------------*/
extern SPI_HandleTypeDef hspi_AD7689;

/* 函数声明 ------------------------------------------------------------------*/
void YS_AD7689_SPI_Init(void);
uint16_t AD7689_Get_Data(uint16_t cmd);


/******************* (C) COPYRIGHT 2019-2030 硬石嵌入式开发团队 *****END OF FILE****/
