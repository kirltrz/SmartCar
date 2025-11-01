#include "IMU.h"
#include <stdbool.h>


SPI_HandleTypeDef *privateHspi;
bool IMU_init(SPI_HandleTypeDef *hspi){
    if(hspi != NULL){
        privateHspi = hspi;
        return true;
    }else {
        return false;
    }
}
static uint8_t lsm6dsr_spi2_read_byte(uint8_t addr) {
  uint8_t tx_buf = addr | LSM6DSR_READ_BIT;  // 拼接读地址（硬件CS自动拉低）
  uint8_t rx_buf = 0;
  // 硬件CS自动激活（传输前拉低）、释放（传输后拉高），无需手动控制
  HAL_SPI_Transmit(privateHspi, &tx_buf, 1, 100);  // 发送地址
  HAL_SPI_Receive(privateHspi, &rx_buf, 1, 100);   // 接收数据
  return rx_buf;
}

uint8_t lsm6dsr_spi2_check_connection(void) {
  uint8_t chip_id = lsm6dsr_spi2_read_byte(LSM6DSR_WHO_AM_I);
  return (chip_id == 0x6B) ? 1 : 0;  // 校验WHO_AM_I默认值
}
