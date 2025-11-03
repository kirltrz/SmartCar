/**
  ******************************************************************************
  * 文件名程: bsp_AD7689.c 
  * 作    者: 硬石嵌入式开发团队
  * 版    本: V1.0
  * 编写日期: 2017-03-30
  * 功    能: bsp_AD7689
  ******************************************************************************
  * 说明：
  * 本例程配套硬石stm32开发板YS-F4Pro使用。
  * 
  * 淘宝：
  * 论坛：http://www.ing10bbs.com
  * 版权归硬石嵌入式开发团队所有，请勿商用。
  ******************************************************************************
  */

/* 包含头文件 ----------------------------------------------------------------*/
#include "ad7689.h"
#include "spi.h"

/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
SPI_HandleTypeDef hspi_AD7689;

/* 扩展变量 ------------------------------------------------------------------*/
uint16_t     IN_DAT[Chanal];             //单极性，全带宽，内部基准4.096，禁用通道序列器，不回读CFG
__IO double  voltage_data[8];            // 电压值（单位：mV）
__IO double  current_data[8];            // 电流值（单位：uA）
__IO int32_t bias_data[8];               // 零点电压的AD转换结果
__IO uint8_t Conve_flag = Conve_False;	 // 滤波结果
uint16_t Conve_data[8]={0};          // 滤波数值		
/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
/**
  * 函数功能: SPI初始化
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 该函数被HAL库内部调用
*/
void YS_AD7689_SPI_Init(void)
{
  
  GPIO_InitTypeDef GPIO_InitStruct;
  /* 使能SPI外设以及SPI引脚时钟 */
  AD7689_SPIx_CLK_ENABLE();
  AD7689_GPIO_CLK_ENABLE();

  AD_CS1_HIGH() ;
	
  GPIO_InitStruct.Pin = AD7689_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;	
  HAL_GPIO_Init(AD7689_CS_Port, &GPIO_InitStruct);
	
  GPIO_InitStruct.Pin = AD7689_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  HAL_GPIO_Init(AD7689_SCK_Port, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = AD7689_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  HAL_GPIO_Init(AD7689_MOSI_Port, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = AD7689_MISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  HAL_GPIO_Init(AD7689_MISO_Port, &GPIO_InitStruct);

  /* SPI外设配置 */
  hspi_AD7689.Instance = AD7689_SPIx;
  hspi_AD7689.Init.Mode = SPI_MODE_MASTER;
  hspi_AD7689.Init.Direction = SPI_DIRECTION_2LINES;
  hspi_AD7689.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi_AD7689.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi_AD7689.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi_AD7689.Init.NSS = SPI_NSS_SOFT;
  hspi_AD7689.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi_AD7689.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi_AD7689.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi_AD7689.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi_AD7689.Init.CRCPolynomial = 7;
  HAL_SPI_Init(&hspi_AD7689);

}

/**
  * 函数功能: 简单延时
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 该函数被HAL库内部调用
*/
static void AD7689_Delay(uint16_t time)
{
	while(time-->0);
}

/**
  * 函数功能: 获取AD7689数值
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 该函数被HAL库内部调用
*/
uint16_t AD7689_Get_Data(uint16_t cmd)
{
  uint8_t  registerWord[2];
  unsigned char buf[2] ={0,0};

  registerWord[0] = cmd>>8;
  registerWord[1] = cmd;
  AD7689_Delay(1000);
  /* 片选使能 */
  AD_CS1_LOW();
 
	HAL_SPI_TransmitReceive(&hspi1,registerWord,buf,2,0xFFFF);
	

  AD_CS1_HIGH();

	return ((buf[0]<<8) | buf[1]);
}



/********** (C) COPYRIGHT 2019-2030 硬石嵌入式开发团队 *******END OF FILE************/

