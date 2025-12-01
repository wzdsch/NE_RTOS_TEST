/*
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2025-11-18 18:34:20
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2025-11-18 18:36:28
 * @FilePath: \MDK-ARMd:\RoboMaster\code\NE_RTOS_TEST\Components\tools\tools.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
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
