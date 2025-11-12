#include "opt.h"
#include "ad7689.h"
#include "UART.h"
#include "main.h"

static uint16_t caliValueMax[16], caliValueMin[16];

static void readAll_ADC_Channel(uint16_t value[16]){
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

void initOpt(void){
    AD7689_Init();
}
/**
 * 功能：光电管循迹线校准
 * 参数：caliValueMax - 各通道最大值（输出），caliValueMin - 各通道最小值（输出）
 * 返回：
 */
void caliOpt(void){
	uint32_t startCaliTime = HAL_GetTick();
	const uint32_t caliTime = 5000;//5s

    // 初始化校准数组	
	for(int i = 0; i < 16; i++){
        caliValueMax[i] = 0;      // 初始化为最小值
        caliValueMin[i] = 0xFFFF; // 初始化为最大值
    }
    while(HAL_GetTick() - startCaliTime < caliTime){
        uint16_t adc_value[16];
        readAll_ADC_Channel(adc_value);
        for(int i=0;i<16;i++){
            if(adc_value[i]>caliValueMax[i] && (adc_value[i]!=0 && adc_value[i]!=0xFFFF)) caliValueMax[i] = adc_value[i];
            if(adc_value[i]<caliValueMin[i] && (adc_value[i]!=0 && adc_value[i]!=0xFFFF)) caliValueMin[i] = adc_value[i];
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
  * @param angle 返回每条循迹线的角度，中心90度，从右到左0-180度
  * @retval 循迹线数量，
   */
uint8_t updateOpt(bool activeHighOrLow, int angle[4]){
    //读取传感器数据
    uint16_t value[16];
    readAll_ADC_Channel(value);

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
            float weight = 1.0f - valueAfterNormalized[i]; // 使用归一化值作为权重
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