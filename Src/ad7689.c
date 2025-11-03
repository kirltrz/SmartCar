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
#include <stdint.h>

/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
//SPI_HandleTypeDef hspi_AD7689;

/* 扩展变量 ------------------------------------------------------------------*/
uint16_t     IN_DAT[Chanal];             //单极性，全带宽，内部基准4.096，禁用通道序列器，不回读CFG
__IO double  voltage_data[8];            // 电压值（单位：mV）
__IO double  current_data[8];            // 电流值（单位：uA）
__IO int32_t bias_data[8];               // 零点电压的AD转换结果
__IO uint8_t Conve_flag = Conve_False;	 // 滤波结果
uint16_t Conve_data[2][8]={0};          // 滤波数值		
/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
/**
  * 函数功能: SPI初始化
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 该函数被HAL库内部调用
*/

/*
//使用stm32cubemx初始化spi，未用到该段代码
void YS_AD7689_SPI_Init(void)
{
  
  GPIO_InitTypeDef GPIO_InitStruct;
  // 使能SPI外设以及SPI引脚时钟
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

  // SPI外设配置
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
*/
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
void AD7689_Init(){
    for(int i=0;i<2;i++){
        for (int j=0; j<2; j++) {
            AD7689_Get_Data(i+1, 0); //上电初始化：上电时CFG寄存器状态未定义，必须执行2次"哑转换"（无有效数据输出的转换）完成寄存器初始化
        }
    }
    
}
/**
  * 函数功能: 生成读取AD7689所需要发送的CFG寄存器值
  * 输入参数: 需要读取的模拟输入引脚 IN0-IN7
  * 返 回 值: 无
  * 说    明: 
*/
static uint16_t genCFG_REG(uint8_t INx){

    /**/
    uint16_t CFG_REG = ((0b1110 << 12) | (0b0001 << 8) | (0b0000 << 4) | 0b0100);//16bit，但寄存器是14bit，MSB优先即只使用高14位，最后两位舍弃

/*
 * AD7682/AD7689 CFG寄存器配置方法（基于AD7682.pdf文档）(详细内容请翻阅文档)
 * 注：CFG寄存器为14位串行寄存器（CFG[13:0]），需通过SPI接口以"MSB优先"方式写入（前14个SCK上升沿锁存DIN数据）
 * 核心功能：配置芯片输入通道、基准源、滤波器、通道序列器及数据回读模式，是芯片功能控制的核心寄存器
 

// 一、CFG寄存器整体配置前提（AD7682.pdf关键要求）
// 1. 上电初始化：上电时CFG寄存器状态未定义，必须执行2次"哑转换"（无有效数据输出的转换）完成寄存器初始化
// 2. 工厂默认预加载：若需快速加载默认配置，需使DIN引脚保持高电平并完成2次转换，此时CFG[13:0] = 0x3FFF，对应配置：
//    - 输入：单极性（参考GND）、序列器使能
//    - 滤波器：全带宽
//    - 基准源：禁用内部基准（启用外部基准）、禁用温度传感器
//    - 回读：不回读CFG寄存器
// 3. 配置更新规则：所有配置修改需通过CFG[13]（配置更新位）置1触发，否则新配置不生效；配置更新存在1个深度延迟（n-1阶段写入，n+1阶段生效）


// 二、CFG寄存器分位段配置说明（CFG[13:0]，从MSB到LSB）
// 1. CFG[13]：配置更新使能位（控制是否覆盖当前配置）
//    - 0：保持当前配置，不更新寄存器内容（新写入的其他位配置无效）
//    - 1：覆盖当前配置，启用新设置（修改任何其他位前需置1）
// 示例：若需修改基准源，需先将CFG[13]置1，再设置CFG[5:3]

// 2. CFG[12:10]：输入通道类型配置位（定义输入模式及参考电平）
//    位组合（12,11,10） | 配置功能                  | 参考电平/输入特性
//    0 0 X（X=0/1）     | 双极性差分对              | INx-参考VREF/2±0.1V，正通道为偶数（IN0/IN2/IN4/IN6）
//    0 1 0              | 双极性输入                | INx参考COM=VREF/2±0.1V
//    0 1 1              | 温度传感器（内部）        | 采集芯片内部温度，输出直接二进制码（参考GND）
//    1 0 X（X=0/1）     | 单极性差分对              | INx-参考GND±0.1V
//    1 1 0              | 单极性输入（COM参考）     | INx参考COM=GND±0.1V
//    1 1 1              | 单极性输入（直接参考GND） | INx范围0~VREF，参考GND
// 示例：配置单极性输入（参考GND），需设置CFG[12:10] = 111

// 3. CFG[9:7]：输入通道选择位（区分AD7682/AD7689通道数差异）
//    - AD7682（4通道）：Bit9为任意值（X），Bit8:7决定通道（00=IN0，01=IN1，10=IN2，11=IN3）
//    - AD7689（8通道）：Bit9:7决定通道（000=IN0，001=IN1，...，111=IN7）
// 序列器模式下：此位定义"扫描的最后一个通道"，扫描从IN0开始（差分对正通道固定为偶数）
// 示例：AD7689选择IN4，需设置CFG[9:7] = 100

// 4. CFG[6]：低通滤波器带宽选择位（影响吞吐速率与噪声抑制）
//    - 0：1/4带宽（-3dB带宽0.425MHz），需将最高吞吐速率降至1/4（避免违反采集时间tACQ，否则THD上升）
//    - 1：全带宽（默认，-3dB带宽1.7MHz），支持最高250kSPS吞吐速率（AD7682/AD7689标称吞吐速率）

// 5. CFG[5:3]：基准源/缓冲器/温度传感器配置位（核心基准控制，需匹配VDD）
//    位组合（5,4,3） | 配置功能                  | 关键要求（AD7682.pdf约束）
//    0 0 0          | 内部基准（REF=2.5V输出）  | VDD≥3.0V，同步使能温度传感器，REF引脚输出2.5V（典型值）
//    0 0 1          | 内部基准（REF=4.096V输出）| VDD≥4.6V，同步使能温度传感器，REF引脚输出4.096V（典型值）
//    0 1 0          | 外部基准（使能温度传感器）| REF引脚直接接外部基准（0.5V~VDD），需10μF X5R陶瓷电容去耦
//    0 1 1          | 外部基准+内部缓冲（使能温度）| 外部基准接REFIN，缓冲后输出至REF（最大值VDD-0.5V）
//    1 1 0          | 外部基准（禁用温度传感器）| 无温度采集功能，REF引脚接外部基准
//    1 1 1          | 外部基准+内部缓冲（禁用温度）| 外部基准接REFIN，缓冲后输出至REF，无温度采集
// 注：100/101为预留位，无定义；无论内部/外部基准，REF引脚必须接10μF去耦电容（靠近引脚）

// 6. CFG[2:1]：通道序列器控制位（多通道重复扫描场景使用）
//    位组合（2,1） | 配置功能                  | 扫描规则
//    0 0          | 禁用序列器                | 仅采集CFG[9:7]选中的单个通道
//    0 1          | 序列扫描+动态更新配置      | 扫描期间可修改CFG寄存器（需配合CFG[13]置1）
//    1 0          | 扫描至目标通道（无温度）    | 从IN0开始→扫描到CFG[9:7]设置的通道→循环
//    1 1          | 扫描至目标通道+温度采集    | 扫描完所有通道后→采集1次温度传感器数据→循环（如：IN0~IN7→TEMP→IN0...）
// 注：修改CFG[1]（差分/单端）或CFG[9:7]（最后通道）时，序列器会重新初始化（从IN0开始）

// 7. CFG[0]：CFG寄存器回读控制位（控制是否回读当前配置）
//    - 0：回读CFG（转换结果LSB后，额外输出14位CFG数据（MSB优先），需14个SCK下降沿）
//    - 1：不回读CFG（仅输出16位转换结果，无需额外SCK）


// 三、CFG配置关键注意事项（AD7682.pdf重点强调）
// 1. VDD与基准源匹配：使用内部2.5V基准时VDD≥3.0V，4.096V基准时VDD≥4.6V，否则内部基准无法工作（自动切换为外部基准模式）
// 2. 配置更新延迟：CFG寄存器写入后，需在转换结束（tCONV最大值）时更新，且需1个转换周期生效（n-1阶段写入，n+1阶段有效）
// 3. 时序约束：CNV上升沿前后20ns/10ns内，需保持SCK、DIN等数字引脚安静（避免噪声耦合影响采样精度）
// 4. REF引脚去耦：无论内部/外部基准，REF引脚必须接10μF X5R陶瓷电容（靠近引脚，用宽PCB走线连接GND），减少寄生电感
// 5. 通道序列器扫描顺序：差分对模式下，正通道固定为偶数（IN0/IN2/IN4/IN6），负通道为奇数（IN1/IN3/IN5/IN7）
*/
    CFG_REG |= (INx << 9); //位 9:7（CFG [9:7]）：输入通道选择
    return CFG_REG;
}
/**
  * 函数功能: 获取AD7689数值
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 该函数被HAL库内部调用
*/
uint16_t AD7689_Get_Data(uint8_t ID, uint8_t INx)
{
    uint8_t  registerWord[2];
    unsigned char buf[2] ={0,0};

    uint16_t cmd = genCFG_REG(INx);
    registerWord[0] = cmd>>8;
    registerWord[1] = cmd;
    AD7689_Delay(1000);
    /* 片选使能 */
    if(ID==1){
        AD_CS1_LOW();
    }
    else if(ID==2)
        AD_CS2_LOW();
 
	HAL_SPI_TransmitReceive(&hspi1,registerWord,buf,2,0xFFFF);
	

    if(ID==1){
        AD_CS1_HIGH();
    }
    else if(ID==2)
        AD_CS2_HIGH();

	return ((buf[0]<<8) | buf[1]);
}



/********** (C) COPYRIGHT 2019-2030 硬石嵌入式开发团队 *******END OF FILE************/

