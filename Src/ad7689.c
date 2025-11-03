#include <stdint.h>
#include "main.h"
#include "ad7689.h"



// 使用SPI接口的AD7689驱动
HAL_StatusTypeDef AD7689_Init(AD7689_HandleTypeDef *hadc) {
    // 初始化CNV引脚
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    GPIO_InitStruct.Pin = hadc->cnv_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(hadc->cnv_port, &GPIO_InitStruct);
    
    // 初始状态
    HAL_GPIO_WritePin(hadc->cnv_port, hadc->cnv_pin, GPIO_PIN_RESET);
    
    // 初始化默认配置
    hadc->config.bits.CFG = 0;
    hadc->config.bits.INCC = AD7689_INCC_UNIPOLAR_GROUND;
    hadc->config.bits.INX = AD7689_CH0;
    hadc->config.bits.BW = 0;
    hadc->config.bits.REF = AD7689_REF_INTERNAL;
    hadc->config.bits.SEQ = AD7689_SEQ_DISABLE;
    hadc->config.bits.RB = 0;
    hadc->config.bits.reserved = 0;
    
    return HAL_OK;
}

// 使用SPI进行转换
HAL_StatusTypeDef AD7689_ReadChannel_SPI(AD7689_HandleTypeDef *hadc, 
                                        AD7689_Channel_t channel, 
                                        uint16_t *adc_value) {
    uint8_t tx_data[2] = {0};
    uint8_t rx_data[2] = {0};
    
    // 更新通道配置
    hadc->config.bits.INX = channel;
    
    // 准备发送数据（配置字）
    tx_data[0] = (hadc->config.value >> 8) & 0xFF;
    tx_data[1] = hadc->config.value & 0xFF;
    
    // 启动转换
    HAL_GPIO_WritePin(hadc->cnv_port, hadc->cnv_pin, GPIO_PIN_SET);
    HAL_Delay(1);  // 转换时间
    
    // 读取数据（同时写入配置）
    HAL_GPIO_WritePin(hadc->cnv_port, hadc->cnv_pin, GPIO_PIN_RESET);
    
    if(HAL_SPI_TransmitReceive(hadc->hspi, tx_data, rx_data, 2, 100) != HAL_OK) {
        return HAL_ERROR;
    }
    
    // 组合ADC结果
    *adc_value = ((rx_data[0] & 0x0F) << 8) | rx_data[1];
    
    return HAL_OK;
}

// 多通道扫描
HAL_StatusTypeDef AD7689_ScanChannels(AD7689_HandleTypeDef *hadc, 
                                     uint16_t *results, 
                                     uint8_t num_channels) {
    if(num_channels > 8) return HAL_ERROR;
    
    for(uint8_t i = 0; i < num_channels; i++) {
        if(AD7689_ReadChannel_SPI(hadc, (AD7689_Channel_t)i, &results[i]) != HAL_OK) {
            return HAL_ERROR;
        }
        HAL_Delay(1);
    }
    
    return HAL_OK;
}
