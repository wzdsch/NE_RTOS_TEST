/*
 * @beforeAnnotation: 
 * Copyright (c) 2026 by 
 * """ The Robomaster team : NEXT-E from Xi'an University of Technology """
 * All Rights Reserved. 
 * 
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2025-11-18 18:34:20
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-01-23 23:44:58
 * @FilePath: \NE_RTOS_TEST\Components\tools\tools.c
 * @Description: 
 */
#include "tools.h"

float uint16_to_float(uint16_t x_uint, float x_min, float x_max, int bits)
{
  return ((float)x_uint) * (x_max - x_min) / ((float)(1 << bits) - 1) + x_min;
}

uint16_t float_to_uint16(float x, float x_min, float x_max, int bits)
{
  return (uint16_t)((x - x_min) * ((float)(1 << bits) - 1) / (x_max - x_min));
}
