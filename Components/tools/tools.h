#ifndef TOOLS_H
#define TOOLS_H

#include <stdint.h>

// 这两个函数是达妙can协议float到uint映射的方法
float uint16_to_float(uint16_t x_uint, float x_min, float x_max, int bits);
uint16_t float_to_uint16(float x_float, float x_min, float x_max, int bits);

#endif
