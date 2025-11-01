#include "stm32f1xx_hal.h"
#include "stdbool.h"
#include <stdint.h>
// 校验核心寄存器：WHO_AM_I（地址0x0F，默认值0x6B）
#define LSM6DSR_WHO_AM_I  0x0F
#define LSM6DSR_READ_BIT  0x80  // 读操作掩码（地址|0x80表示读）

bool IMU_init(SPI_HandleTypeDef *);
uint8_t lsm6dsr_spi2_check_connection(void);