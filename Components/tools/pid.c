/*
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2025-10-26 17:03:40
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2025-10-30 20:15:58
 * @FilePath: \MDK-ARMd:\RoboMaster\code\NE_RTOS_TEST\Components\tools\pid.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
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

void PID_Init(Pid_t *p_pid_t, PID_MODE_e mode, const fp32 Kp, const fp32 Ki, const fp32 Kd, const fp32 max_out, const fp32 max_iout) {
  if (p_pid_t == NULL) {
    return;
  }
  p_pid_t->mode = mode;
  p_pid_t->Kp = Kp;
  p_pid_t->Ki = Ki;
  p_pid_t->Kd = Kd;
  p_pid_t->max_out = max_out;
  p_pid_t->max_iout = max_iout;
  p_pid_t->Dbuf[0] = p_pid_t->Dbuf[1] = p_pid_t->Dbuf[2] = 0.0f;
  p_pid_t->error[0] = p_pid_t->error[1] = p_pid_t->error[2] = p_pid_t->Pout = p_pid_t->Iout =
  p_pid_t->Dout = p_pid_t->out = 0.0f;
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
    p_pid_t->Pout = p_pid_t->Kp * p_pid_t->error[0];
    p_pid_t->Iout += p_pid_t->Ki * p_pid_t->error[0];
    p_pid_t->Dbuf[2] = p_pid_t->Dbuf[1];
    p_pid_t->Dbuf[1] = p_pid_t->Dbuf[0];
    p_pid_t->Dbuf[0] = (p_pid_t->error[0] - p_pid_t->error[1]);
    p_pid_t->Dout = p_pid_t->Kd * p_pid_t->Dbuf[0];
    LimitMax(p_pid_t->Iout, p_pid_t->max_iout);
    p_pid_t->out = p_pid_t->Pout + p_pid_t->Iout + p_pid_t->Dout;
    LimitMax(p_pid_t->out, p_pid_t->max_out);
    break;
  case PID_DELTA:
    p_pid_t->Pout = p_pid_t->Kp * (p_pid_t->error[0] - p_pid_t->error[1]);
    p_pid_t->Iout = p_pid_t->Ki * p_pid_t->error[0];
    p_pid_t->Dbuf[2] = p_pid_t->Dbuf[1];
    p_pid_t->Dbuf[1] = p_pid_t->Dbuf[0];
    p_pid_t->Dbuf[0] = (p_pid_t->error[0] - 2.0f * p_pid_t->error[1] + p_pid_t->error[2]);
    p_pid_t->Dout = p_pid_t->Kd * p_pid_t->Dbuf[0];
    p_pid_t->out += p_pid_t->Pout + p_pid_t->Iout + p_pid_t->Dout;
    LimitMax(p_pid_t->out, p_pid_t->max_out);
    break;
  }
  return p_pid_t->out;
}

void PID_Clear(Pid_t *p_pid_t) {
  if (p_pid_t == NULL) {
    return;
  }
  p_pid_t->error[0] = p_pid_t->error[1] = p_pid_t->error[2] = 0.0f;
  p_pid_t->Dbuf[0] = p_pid_t->Dbuf[1] = p_pid_t->Dbuf[2] = 0.0f;
  p_pid_t->out = p_pid_t->Pout = p_pid_t->Iout = p_pid_t->Dout = 0.0f;
  p_pid_t->fdb = p_pid_t->set = 0.0f;
}

