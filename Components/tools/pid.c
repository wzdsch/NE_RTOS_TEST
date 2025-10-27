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

void PID_init(pids *pid, PID_MODE_e mode, const fp32 Kp, const fp32 Ki, const fp32 Kd, const fp32 max_out, const fp32 max_iout) {
  if (pid == NULL) {
    return;
  }
  pid->mode = mode;
  pid->Kp = Kp;
  pid->Ki = Ki;
  pid->Kd = Kd;
  pid->max_out = max_out;
  pid->max_iout = max_iout;
  pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
  pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout =
      pid->Dout = pid->out = 0.0f;
}

fp32 PID_calc(pids *pid, fp32 ref, fp32 set) {
  if (pid == NULL) {
    return 0.0f;
  }
  pid->error[2] = pid->error[1];
  pid->error[1] = pid->error[0];
  pid->set = set;
  pid->fdb = ref;
  pid->error[0] = set - ref;
  switch (pid->mode) {
  case PID_POSITION:
    pid->Pout = pid->Kp * pid->error[0];
    pid->Iout += pid->Ki * pid->error[0];
    pid->Dbuf[2] = pid->Dbuf[1];
    pid->Dbuf[1] = pid->Dbuf[0];
    pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
    pid->Dout = pid->Kd * pid->Dbuf[0];
    LimitMax(pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    LimitMax(pid->out, pid->max_out);
    break;
  case PID_DELTA:
    pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
    pid->Iout = pid->Ki * pid->error[0];
    pid->Dbuf[2] = pid->Dbuf[1];
    pid->Dbuf[1] = pid->Dbuf[0];
    pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
    pid->Dout = pid->Kd * pid->Dbuf[0];
    pid->out += pid->Pout + pid->Iout + pid->Dout;
    LimitMax(pid->out, pid->max_out);
    break;
  }
  return pid->out;
}

void PID_clear(pids *pid) {
  if (pid == NULL) {
    return;
  }
  pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
  pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
  pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
  pid->fdb = pid->set = 0.0f;
}

