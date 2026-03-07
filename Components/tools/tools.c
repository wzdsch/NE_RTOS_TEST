/*
 * @beforeAnnotation: 
 * Copyright (c) 2026 by 
 * """ The Robomaster team : NEXT-E from Xi'an University of Technology """
 * All Rights Reserved. 
 * 
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2025-11-18 18:34:20
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-03-08 00:18:45
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

float usr_fabsf(float x) {
  return (x < 0) ? -x : x;
}

/**
 * @brief  斜坡规划器函数，用于实现从当前值到目标值的平滑过渡。
 * @param  currentValue: 当前值（float）。
 * @param  targetValue: 目标值（float）。
 * @param  increaseRate: 增加时的变化率（float）。
 * @param  decreaseRate: 减小时的变化率（float）。
 * @return float: 返回当前过渡后的值。
 */
float RampPlanner(float current_value, float target_value, float increase_rate, float decrease_rate) {
  increase_rate = usr_fabsf(increase_rate);
  decrease_rate = usr_fabsf(decrease_rate);
  // 判断目标值与当前值的关系，确定过渡方向
  float fabs_current_value = usr_fabsf(current_value);
  float fabs_target_value = usr_fabsf(target_value);
  float tar_mul_cur = target_value * current_value;
  
  // 如果目标值等于当前值，直接返回
  if (target_value <= current_value + 0.0001f && target_value >= current_value - 0.0001f)
  {
    return current_value;
  }

  // 目标值绝对值大于当前绝对值，并且目标值和当前值都为正，则为正向加速 
  else if ((fabs_target_value > fabs_current_value && current_value >= 0 && target_value > 0))
  {
      current_value += increase_rate;  // 每次增加变化率
      if (current_value > target_value) {
          current_value = target_value;  // 防止超过目标值
      }
      return current_value;
  }

  // 目标值绝对值大于当前绝对值，并且目标值和当前值都为负，则为负向加速
  else if (fabs_target_value > fabs_current_value && current_value <= 0 && target_value < 0)
  {
      current_value -= increase_rate;
      if (current_value < target_value) {
          current_value = target_value;  // 防止低于目标值
      }
      return current_value;
  }

  /*
    正向减速：
      1.当前值为正，且目标值为负
      2.目标值绝对值小于当前值绝对值，且目标值与当前值都为正
  */
  else if ((current_value > 0 && target_value < 0)
    || (fabs_target_value < fabs_current_value && target_value >= 0 && current_value > 0)) 
  {
    current_value -= decrease_rate;
    if (current_value < target_value) // 防止低于目标值
    {
      current_value = target_value;
    }
    return current_value;
  }

  /*
    负向减速：
      1.当前值为负，且目标值为正
      2.目标值绝对值小于当前值绝对值，且目标值与当前值都为负
  */
  else if ((current_value < 0 && target_value > 0) || \
  (fabs_target_value < fabs_current_value && target_value <= 0 && current_value < 0))
  {
    current_value += decrease_rate;
    if (current_value > target_value) // 防止高于目标值
    {
      current_value = target_value;
    }
    return current_value;
  }
  return current_value;
}
