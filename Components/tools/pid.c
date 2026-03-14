/*
 * @beforeAnnotation: 
 * Copyright (c) 2026 by 
 * """ The Robomaster team : NEXT-E from Xi'an University of Technology """
 * All Rights Reserved. 
 * 
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2025-10-26 17:03:40
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-02-28 13:36:17
 * @FilePath: \NE_RTOS_TEST\Components\tools\pid.c
 * @Description: 
 */
#include "pid.h"
#include "main.h"
#include "pidData.h"

#define LimitMax(input, max)                                                   \
  {                                                                            \
    if (input > max) {                                                         \
      input = max;                                                             \
    } else if (input < -max) {                                                 \
      input = -max;                                                            \
    }                                                                          \
  }

void PID_Init(Pid_t *p_pid_t, PID_Init_t *p_init) {
  if (p_pid_t == NULL) {
    return;
  }
  p_pid_t->mode = p_init->mode;
  p_pid_t->kp = p_init->kp;
  p_pid_t->ki = p_init->ki;
  p_pid_t->kd = p_init->kd;
  p_pid_t->out_limit = p_init->out_limit;
  p_pid_t->i_out_limit = p_init->i_out_limit;
  p_pid_t->d_buf[0] = p_pid_t->d_buf[1] = p_pid_t->d_buf[2] = 0.0f;
  p_pid_t->error[0] = p_pid_t->error[1] = p_pid_t->error[2] = p_pid_t->p_out = p_pid_t->i_out =
  p_pid_t->d_out = p_pid_t->out = 0.0f;
}

fp32 PID_Calc(Pid_t *p_pid_t, fp32 ref, fp32 set) {
  if (p_pid_t == NULL) {
    return 0.0f;
  }
  p_pid_t->error[2] = p_pid_t->error[1];
  p_pid_t->error[1] = p_pid_t->error[0];
  p_pid_t->set = set;
  p_pid_t->fdb = ref;
  p_pid_t->error[0] = set - ref;
  switch (p_pid_t->mode) {
  case PID_POSITION:
    p_pid_t->p_out = p_pid_t->kp * p_pid_t->error[0];
    p_pid_t->i_out += p_pid_t->ki * p_pid_t->error[0];
    p_pid_t->d_buf[2] = p_pid_t->d_buf[1];
    p_pid_t->d_buf[1] = p_pid_t->d_buf[0];
    p_pid_t->d_buf[0] = (p_pid_t->error[0] - p_pid_t->error[1]);
    p_pid_t->d_out = p_pid_t->kd * p_pid_t->d_buf[0];
    LimitMax(p_pid_t->i_out, p_pid_t->i_out_limit);
    p_pid_t->out = p_pid_t->p_out + p_pid_t->i_out + p_pid_t->d_out;
    LimitMax(p_pid_t->out, p_pid_t->out_limit);
    break;
  case PID_DELTA:
    p_pid_t->p_out = p_pid_t->kp * (p_pid_t->error[0] - p_pid_t->error[1]);
    p_pid_t->i_out = p_pid_t->ki * p_pid_t->error[0];
    p_pid_t->d_buf[2] = p_pid_t->d_buf[1];
    p_pid_t->d_buf[1] = p_pid_t->d_buf[0];
    p_pid_t->d_buf[0] = (p_pid_t->error[0] - 2.0f * p_pid_t->error[1] + p_pid_t->error[2]);
    p_pid_t->d_out = p_pid_t->kd * p_pid_t->d_buf[0];
    p_pid_t->out += p_pid_t->p_out + p_pid_t->i_out + p_pid_t->d_out;
    LimitMax(p_pid_t->out, p_pid_t->out_limit);
    break;
  }
  return p_pid_t->out;
}

void PID_Clear(Pid_t *p_pid) {
  if (p_pid == NULL) {
    return;
  }
  p_pid->error[0] = p_pid->error[1] = p_pid->error[2] = 0.0f;
  p_pid->d_buf[0] = p_pid->d_buf[1] = p_pid->d_buf[2] = 0.0f;
  p_pid->out = p_pid->p_out = p_pid->i_out = p_pid->d_out = 0.0f;
  p_pid->fdb = p_pid->set = 0.0f;
}

