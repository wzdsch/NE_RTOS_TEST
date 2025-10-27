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
} pids;

extern void PID_init(pids *pid, uint8_t mode, const fp32 Kp, const fp32 Ki, const fp32 Kd, const fp32 max_out, const fp32 max_iout);
extern fp32 PID_calc(pids *pid, fp32 ref, fp32 set);
extern void PID_clear(pids *pid);

#endif
