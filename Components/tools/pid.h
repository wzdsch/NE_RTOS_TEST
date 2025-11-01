/*
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2025-10-26 17:03:40
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2025-10-30 20:04:16
 * @FilePath: \MDK-ARMd:\RoboMaster\code\NE_RTOS_TEST\Components\tools\pid.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
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
  fp32 Kp;
  fp32 Ki;
  fp32 Kd;

  fp32 max_out;  // 输出限幅
  fp32 max_iout; // 积分限幅

  fp32 set; // 设定目标值
  fp32 fdb; // 反馈值

  fp32 out; // 总输出
  fp32 Pout; // P项输出
  fp32 Iout; // I项输出
  fp32 Dout; // D项输出
  fp32 Dbuf[3]; // 
  fp32 error[3]; // 误差缓存
} Pid_t;

void PID_Init(Pid_t *p_pid_t, PID_MODE_e mode, const fp32 Kp, const fp32 Ki, const fp32 Kd, const fp32 max_out, const fp32 max_iout);
fp32 PID_Calc(Pid_t *p_pid_t, fp32 ref, fp32 set);
void PID_Clear(Pid_t *p_pid_t);

#endif
