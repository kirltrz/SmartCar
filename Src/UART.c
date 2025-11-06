#include "UART.h"
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