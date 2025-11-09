#pragma once
/*光电管识别相关*/
#include <stdbool.h>
#include <stdint.h>

void initOpt(void);
void caliOpt(void);
uint8_t updateOpt(bool activeHighOrLow, int angle[4]);