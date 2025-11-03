// AD7689配置寄存器位定义
typedef union {
    struct {
        uint16_t CFG      : 3;  // 配置位
        uint16_t INCC     : 3;  // 输入通道配置
        uint16_t INX      : 3;  // 通道选择
        uint16_t BW       : 1;  // 带宽
        uint16_t REF      : 2;  // 参考电压选择
        uint16_t SEQ      : 2;  // 序列器模式
        uint16_t RB       : 1;  // 回读配置
        uint16_t reserved : 1;  // 保留
    } bits;
    uint16_t value;
} AD7689_Config_t;

// 输入通道配置
typedef enum {
    AD7689_INCC_UNIPOLAR_GROUND = 0,     // 单极性，参考GND
    AD7689_INCC_UNIPOLAR_COM,            // 单极性，参考COM
    AD7689_INCC_BIPOLAR,                 // 双极性
    AD7689_INCC_UNIPOLAR_DIFF,           // 单极性差分
    AD7689_INCC_BIPOLAR_DIFF,            // 双极性差分
    AD7689_INCC_TEMPERATURE              // 温度传感器
} AD7689_InputConfig_t;

// 通道选择
typedef enum {
    AD7689_CH0 = 0,
    AD7689_CH1,
    AD7689_CH2,
    AD7689_CH3,
    AD7689_CH4,
    AD7689_CH5,
    AD7689_CH6,
    AD7689_CH7,
    AD7689_TEMP,        // 温度传感器
    AD7689_COM,         // COM输入
    AD7689_DIFF0,       // 差分对0 (CH0-CH1)
    AD7689_DIFF1,       // 差分对1 (CH2-CH3)
    AD7689_DIFF2,       // 差分对2 (CH4-CH5)
    AD7689_DIFF3        // 差分对3 (CH6-CH7)
} AD7689_Channel_t;

// 参考电压选择
typedef enum {
    AD7689_REF_INTERNAL = 0,    // 内部参考
    AD7689_REF_EXTERNAL,        // 外部参考
    AD7689_REF_SUPPLY,          // 电源电压作为参考
    AD7689_REF_RESERVED
} AD7689_RefSelect_t;

// 序列器模式
typedef enum {
    AD7689_SEQ_DISABLE = 0,     // 禁用序列器
    AD7689_SEQ_UPDATE,          // 更新配置
    AD7689_SEQ_SCAN_0_TO_X,     // 扫描0到X
    AD7689_SEQ_RESERVED
} AD7689_Sequencer_t;

// AD7689 HAL驱动
typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cnv_port;
    uint16_t cnv_pin;
    float vref;
    AD7689_Config_t config;
} AD7689_HandleTypeDef;

HAL_StatusTypeDef AD7689_Init(AD7689_HandleTypeDef *);
HAL_StatusTypeDef AD7689_ScanChannels(AD7689_HandleTypeDef *hadc, 
                                     uint16_t *results, 
                                     uint8_t num_channels);