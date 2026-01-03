/*
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2025-10-29 12:09:26
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-01-03 14:06:48
 * @FilePath: \MDK-ARMd:\RoboMaster\code\NE_RTOS_TEST\Components\motor\motor_common.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "MOtorCtrl.h"
#include "string.h"

void MotorCtrl_Init(MotorCtrl_t *p_motor_ctrl, MotorCtrl_PidMode_e ctrl_mode, uint32_t out_values, float max_out, void *p_owner_moudle)
{
  if (p_motor_ctrl == NULL)
  {
    while(1) {
      
    }
  }

  p_motor_ctrl->state = MOTOR_CTRL_DISABLE;
  p_motor_ctrl->ctrl_mode = ctrl_mode;
  p_motor_ctrl->out_values = out_values;
  p_motor_ctrl->max_out = max_out;

  p_motor_ctrl->set_target = 0;

  p_motor_ctrl->pCustomCtrlAlgorithm = NULL;
  p_motor_ctrl->pFeedForward = NULL;
  p_motor_ctrl->pPreProcess = NULL;
  p_motor_ctrl->pPostProcess = NULL;

  p_motor_ctrl->pid_out = 0;
  p_motor_ctrl->custom_ctrl_algorithm_out = 0;
  p_motor_ctrl->feed_forward_out = 0;
  p_motor_ctrl->pre_process_out = 0;
  p_motor_ctrl->post_process_out = 0;
  p_motor_ctrl->final_out = 0;

  p_motor_ctrl->p_pid_ext_fdb = 0;
  p_motor_ctrl->p_pid_int_fdb = 0;

  p_motor_ctrl->p_owner_moudle = p_owner_moudle;

  memset(&p_motor_ctrl->pid_external, 0, sizeof(p_motor_ctrl->pid_external));
  memset(&p_motor_ctrl->pid_internal, 0, sizeof(p_motor_ctrl->pid_internal));
}

void MotorCtrl_ExternalPid_Init(MotorCtrl_t *p_motor_ctrl, PID_MODE_e pid_mode, float *p_ref, float p_pid_data[5]) {
  if (p_motor_ctrl == NULL || p_ref == NULL || p_pid_data == NULL)
  {
    while(1) {

    }
  }

  p_motor_ctrl->p_pid_ext_fdb = p_ref;
  PID_Init(&p_motor_ctrl->pid_external, pid_mode, p_pid_data[0], p_pid_data[1], p_pid_data[2], p_pid_data[3], p_pid_data[4]);
}

void MotorCtrl_InternalPid_Init(MotorCtrl_t *p_motor_ctrl, PID_MODE_e pid_mode, float *p_ref, float p_pid_data[5]) {
  if (p_motor_ctrl == NULL || p_ref == NULL || p_pid_data == NULL)
  {
    while(1) {

    }
  }

  p_motor_ctrl->p_pid_int_fdb = p_ref;
  PID_Init(&p_motor_ctrl->pid_internal, pid_mode, p_pid_data[0], p_pid_data[1], p_pid_data[2], p_pid_data[3], p_pid_data[4]);
}

float _MotorCtrl_PID_Calc(MotorCtrl_t *p_motor_ctrl)
{
  // 单环使用内环
  if (p_motor_ctrl->ctrl_mode == MOTOR_CTRL_PID_INTERNAL) {
    PID_Calc(&p_motor_ctrl->pid_internal, *p_motor_ctrl->p_pid_int_fdb, p_motor_ctrl->set_target);
    return p_motor_ctrl->pid_internal.out;
  }
  // 双环, 外环输出作为内环输入
  else if (p_motor_ctrl->ctrl_mode == MOTOR_CTRL_PID_DOUBLE) {
    PID_Calc(&p_motor_ctrl->pid_external, *p_motor_ctrl->p_pid_ext_fdb, p_motor_ctrl->set_target);
    PID_Calc(&p_motor_ctrl->pid_internal, *p_motor_ctrl->p_pid_int_fdb, p_motor_ctrl->pid_external.out);
    return p_motor_ctrl->pid_internal.out;
  }
  return 0;
}

void MotorCtrl_Calc(MotorCtrl_t *p_motor_ctrl)
{
  if (p_motor_ctrl->state == MOTOR_CTRL_DISABLE)
  {
    // 注意: 电机为失能模式，并不会清零final_out, 而是在具体的电机类中发送失能数据
    return;
  }
  else if (p_motor_ctrl->state == MOTOR_CTRL_ENABLE)
  {
    // 预处理函数
    if (p_motor_ctrl->pPreProcess != NULL)
    {
      p_motor_ctrl->pre_process_out = p_motor_ctrl->pPreProcess(p_motor_ctrl);
    }

    // 计算pid
    p_motor_ctrl->pid_out = _MotorCtrl_PID_Calc(p_motor_ctrl);

    // 自定义控制算法
    if (p_motor_ctrl->pCustomCtrlAlgorithm != NULL)
    {
      p_motor_ctrl->custom_ctrl_algorithm_out = p_motor_ctrl->pCustomCtrlAlgorithm(p_motor_ctrl);
    }

    // 前馈算法
    if (p_motor_ctrl->pFeedForward != NULL)
    {
      p_motor_ctrl->feed_forward_out = p_motor_ctrl->pFeedForward(p_motor_ctrl);
    }

    // 后处理函数
    if (p_motor_ctrl->pPostProcess != NULL)
    {
      p_motor_ctrl->post_process_out = p_motor_ctrl->pPostProcess(p_motor_ctrl);
    }

    // 通过设定输出来计算输出值
    float temp_out = 0;
    if (p_motor_ctrl->out_values & MOTOR_CTRL_OUT_FEEDFWD)
    {
      temp_out += p_motor_ctrl->feed_forward_out;
    }
    if (p_motor_ctrl->out_values & MOTOR_CTRL_OUT_PID)
    {
      temp_out += p_motor_ctrl->pid_out;
    }
    if (p_motor_ctrl->out_values & MOTOR_CTRL_OUT_PREPROCESS)
    {
      temp_out += p_motor_ctrl->pre_process_out;
    }
    if (p_motor_ctrl->out_values & MOTOR_CTRL_OUT_CUSTOM)
    {
      temp_out += p_motor_ctrl->custom_ctrl_algorithm_out;
    }
    if (p_motor_ctrl->out_values & MOTOR_CTRL_OUT_POSTPROCESS)
    {
      temp_out += p_motor_ctrl->post_process_out;
    }

    // 输出限幅
    if (temp_out > p_motor_ctrl->max_out)
    {
      temp_out = p_motor_ctrl->max_out;
    }
    else if (temp_out < -p_motor_ctrl->max_out)
    {
      temp_out = -p_motor_ctrl->max_out;
    }
    p_motor_ctrl->final_out = temp_out; // 最终输出
  }
}

void MotorCtrl_SetTarget(MotorCtrl_t *p_motor_ctrl, float target) {
  p_motor_ctrl->set_target = target;
}

void MotorCtrl_Enable(MotorCtrl_t *p_motor_ctrl) {
  p_motor_ctrl->state = MOTOR_CTRL_ENABLE;
}

void MotorCtrl_Disable(MotorCtrl_t *p_motor_ctrl) {
  p_motor_ctrl->state = MOTOR_CTRL_DISABLE;
}

void MotorCtrl_SetCustomCtrlAlgorithm(MotorCtrl_t *p_motor_ctrl, float (*pCustomCtrlAlgorithm)(struct _MotorCtrl_t *p_motor_ctrl)) {
  p_motor_ctrl->pCustomCtrlAlgorithm = pCustomCtrlAlgorithm;
}

void MotorCtrl_SetPreProcess(MotorCtrl_t *p_motor_ctrl, float (*pPreProcess)(struct _MotorCtrl_t *p_motor_ctrl)) {
  p_motor_ctrl->pPreProcess = pPreProcess;
}

void MotorCtrl_SetPostProcess(MotorCtrl_t *p_motor_ctrl, float (*pPostProcess)(struct _MotorCtrl_t *p_motor_ctrl)) {

}

void MotorCtrl_SetFeedForward(MotorCtrl_t *p_motor_ctrl, float (*FeedFwd)(MotorCtrl_t *p_motor_ctrl)) {
  p_motor_ctrl->pFeedForward = FeedFwd;
}
