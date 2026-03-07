/*
 * @beforeAnnotation: 
 * Copyright (c) 2026 by 
 * """ The Robomaster team : NEXT-E from Xi'an University of Technology """
 * All Rights Reserved. 
 * 
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2025-11-18 18:34:33
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-03-08 00:06:40
 * @FilePath: \NE_RTOS_TEST\Components\tools\tools.h
 * @Description: 
 */
#ifndef TOOLS_H
#define TOOLS_H

#include <stdint.h>

// 这两个函数是达妙can协议float到uint映射的方法
float uint16_to_float(uint16_t x_uint, float x_min, float x_max, int bits);
uint16_t float_to_uint16(float x_float, float x_min, float x_max, int bits);

/**
 * @brief  斜坡规划器函数，用于实现从当前值到目标值的平滑过渡。
 * @param  currentValue: 当前值（float）。
 * @param  targetValue: 目标值（float）。
 * @param  increaseRate: 增加时的变化率（float）。
 * @param  decreaseRate: 减小时的变化率（float）。
 * @return float: 返回当前过渡后的值。
 */
float RampPlanner(float current_value, float target_value, float increase_rate, float decrease_rate);

#endif
