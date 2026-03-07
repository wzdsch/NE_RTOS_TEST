/*
 * @beforeAnnotation: 
 * Copyright (c) 2026 by 
 * """ The Robomaster team : NEXT-E from Xi'an University of Technology """
 * All Rights Reserved. 
 * 
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2025-10-26 17:03:40
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-02-28 13:34:36
 * @FilePath: \NE_RTOS_TEST\Components\tools\pid.h
 * @Description: 
 */
#include "struct_typedef.h"
#include "main.h"
#ifndef PID_H
#define PID_H

typedef enum PID_MODE
{
  PID_POSITION = 0,
  PID_DELTA
}PID_MODE_e;

typedef struct pidData
{
  PID_MODE_e mode;
  // PID数据
  fp32 kp;
  fp32 ki;
  fp32 kd;

  fp32 out_limit;  // 输出限幅
  fp32 i_out_limit; // 积分限幅

  fp32 set; // 设定目标值
  fp32 fdb; // 反馈值

  fp32 out; // 总输出
  fp32 p_out; // P项输出
  fp32 i_out; // I项输出
  fp32 d_out; // D项输出
  fp32 d_buf[3]; // 
  fp32 error[3]; // 误差缓存
} Pid_t;

typedef struct {
  PID_MODE_e mode;
  fp32 kp;
  fp32 ki;
  fp32 kd;
  fp32 out_limit;
  fp32 i_out_limit;
} PID_Init_t;

void PID_Init(Pid_t *p_pid_t, PID_Init_t *p_init);
fp32 PID_Calc(Pid_t *p_pid_t, fp32 ref, fp32 set);
void PID_Clear(Pid_t *p_pid_t);

#endif
