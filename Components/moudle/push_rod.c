/*
 * @beforeAnnotation:
 * Copyright (c) 2026 by
 * """ The Robomaster team : NEXT-E from Xi'an University of Technology """
 * All Rights Reserved.
 *
 * @Author: Custom PushRod Module
 * @Date: 2026-02-27
 * @FilePath: \NE_RTOS_TEST\Components\motor\pushRod.c
 * @Description: 推杆机构控制模块实现，内置前馈
 */
#include "push_rod.h"
#include <string.h>
#include "tools.h"

/// @brief 同步双电机累计编码误差
/// @param p_push_rod 推杆结构体指针
static void _PushRod_SyncMotorEcdError(PushRod_t *p_push_rod)
{
  p_push_rod->err_total_ecd = p_push_rod->motor1.processed_measure.pos_total_ecd_f - p_push_rod->motor2.processed_measure.pos_total_ecd_f;
  PID_Calc(&p_push_rod->err_pid, p_push_rod->err_total_ecd, 0);
}

/// @brief 电机1前馈函数
/// @param p_motor 电机结构体指针
/// @return 前馈力矩 (电流编码值)
static float _PushRod_Motor1_FeedForward(MotorCtrl_t *p_ctrl)
{
  return ((PushRod_t *)(p_ctrl->p_owner_moudle))->err_pid.out;
}

/// @brief 电机2前馈函数
/// @param p_motor_ctrl 控制模块结构体指针
/// @return 前馈力矩 (电流编码值)
static float _PushRod_Motor2_FeedForward(MotorCtrl_t *p_motor_ctrl)
{
  PushRod_t *p_push_rod = (PushRod_t *)(((DJI_Motor_t *)(p_motor_ctrl->p_owner_moudle))->p_owner_moudle);
  return -p_push_rod->err_pid.out;
}

/// @brief 推杆机构初始化
void PushRod_Init(PushRod_t *p_push_rod, PushRod_Init_t *p_init)
{
  // 参数合法性检查
  if (p_push_rod == NULL || p_init == NULL)
  {
    while (1) {
      // param err
    }
  }

  // 初始化推杆结构体基础参数
  memset(p_push_rod, 0, sizeof(PushRod_t));
  p_push_rod->state = PUSH_ROD_STATE_DISABLE;
  p_push_rod->calib_current = p_init->calib_current;
  p_push_rod->calib_target_count = p_init->calib_target_count;
  p_push_rod->calib_count = 0;
  p_push_rod->max_ecd = p_init->max_ecd;
  p_push_rod->min_ecd = p_init->min_ecd;

  p_init->motor1_init.p_owner_moudle = p_push_rod;
  p_init->motor2_init.p_owner_moudle = p_push_rod;

  DJI_Motor_Init(&p_push_rod->motor1, &p_init->motor1_init);
  DJI_Motor_Init(&p_push_rod->motor2, &p_init->motor2_init);

  // 初始化电机1控制模块 (双环PID，累计编码位置控制)
  MotorCtrl_Init(&p_push_rod->motor1_ctrl, \
                 MOTOR_CTRL_PID_DOUBLE, \
                 MOTOR_CTRL_OUT_PID | MOTOR_CTRL_OUT_FEEDFWD, \
                 p_init->motor1_max_out, \
                 &p_push_rod->motor1);
  MotorCtrl_SetFeedForward(&p_push_rod->motor1_ctrl, _PushRod_Motor1_FeedForward);

  MotorCtrl_ExternalPid_Init(&p_push_rod->motor1_ctrl, \
                             &p_push_rod->motor1.processed_measure.pos_total_ecd_f, \
                             &p_init->motor1_pos_params);
  MotorCtrl_InternalPid_Init(&p_push_rod->motor1_ctrl, \
                             &p_push_rod->motor1.processed_measure.spd_rpm_f, \
                             &p_init->motor1_spd_params);

  // 初始化电机2控制模块 (双环PID，累计编码位置控制)
  MotorCtrl_Init(&p_push_rod->motor2_ctrl, \
                 MOTOR_CTRL_PID_DOUBLE, \
                 MOTOR_CTRL_OUT_PID | MOTOR_CTRL_OUT_FEEDFWD, \
                 p_init->motor2_max_out, \
                 &p_push_rod->motor2);
  MotorCtrl_SetFeedForward(&p_push_rod->motor2_ctrl, _PushRod_Motor2_FeedForward);

  // 初始化累计编码误差PID
  PID_Init(&p_push_rod->err_pid, &p_init->err_pid_params);

  // 绑定电机2的累计编码反馈
  MotorCtrl_ExternalPid_Init(&p_push_rod->motor2_ctrl, \
                             &p_push_rod->motor2.processed_measure.pos_total_ecd_f, \
                             &p_init->motor2_pos_params);
  MotorCtrl_InternalPid_Init(&p_push_rod->motor2_ctrl, \
                             &p_push_rod->motor2.processed_measure.spd_rpm_f, \
                             &p_init->motor2_spd_params);

  // 失能电机控制
  MotorCtrl_Disable(&p_push_rod->motor1_ctrl);
  MotorCtrl_Disable(&p_push_rod->motor2_ctrl);
  DJI_Motor_Disable(&p_push_rod->motor1);
  DJI_Motor_Disable(&p_push_rod->motor2);
}

/// @brief 设置推杆目标累计编码值
void PushRod_SetTarget(PushRod_t *p_push_rod, float target_ecd)
{
  if (p_push_rod == NULL || p_push_rod->state != PUSH_ROD_STATE_ENABLE)
  {
    return;
  }

  if (target_ecd > p_push_rod->max_ecd) {
    target_ecd = p_push_rod->max_ecd;
  }
  else if (target_ecd < p_push_rod->min_ecd) {
    target_ecd = p_push_rod->min_ecd;
  }

  float real_total_ecd = (p_push_rod->motor1.processed_measure.pos_total_ecd +\
                         p_push_rod->motor2.processed_measure.pos_total_ecd) / 2;

  p_push_rod->target_total_ecd = RampPlanner(real_total_ecd, target_ecd, 2000, 2000);
  // 设置电机目标值
  MotorCtrl_SetTarget(&p_push_rod->motor1_ctrl, p_push_rod->target_total_ecd);
  MotorCtrl_SetTarget(&p_push_rod->motor2_ctrl, p_push_rod->target_total_ecd);
}

/// @brief 推杆控制计算
void PushRod_Calc(PushRod_t *p_push_rod)
{
  if (p_push_rod == NULL)
  {
    return;
  }

  switch (p_push_rod->state)
  {
  case PUSH_ROD_STATE_DISABLE:
  {
    // 失能状态，不做任何处理
    break;
  }

  case PUSH_ROD_STATE_CALIBRATING:
  {
    // 校准中状态
    p_push_rod->calib_count++;

    // 给两个电机设置校准电流
    DJI_Motor_SetCmd(&p_push_rod->motor1, p_push_rod->calib_current);
    DJI_Motor_SetCmd(&p_push_rod->motor2, p_push_rod->calib_current);

    // 检查是否达到校准目标计数
    if (p_push_rod->calib_count >= p_push_rod->calib_target_count)
    {
      // 校准完成，将当前累计编码值设为原始值 (当前单圈编码值)
      // 电机1校准
      p_push_rod->motor1.processed_measure.pos_total_ecd = p_push_rod->motor1.processed_measure.pos_ecd;
      p_push_rod->motor1.processed_measure.pos_total_ecd_f = (float)p_push_rod->motor1.processed_measure.pos_ecd;
      p_push_rod->motor1.processed_measure.pos_last_ecd = p_push_rod->motor1.processed_measure.pos_ecd;

      // 电机2校准 (与电机1保持一致)
      p_push_rod->motor2.processed_measure.pos_total_ecd = p_push_rod->motor1.processed_measure.pos_total_ecd;
      p_push_rod->motor2.processed_measure.pos_total_ecd_f = p_push_rod->motor1.processed_measure.pos_total_ecd_f;
      p_push_rod->motor2.processed_measure.pos_last_ecd = p_push_rod->motor2.processed_measure.pos_ecd;

      // 重置目标值为当前校准后的位置
      p_push_rod->target_total_ecd = p_push_rod->motor1.processed_measure.pos_total_ecd_f;
      MotorCtrl_SetTarget(&p_push_rod->motor1_ctrl, p_push_rod->target_total_ecd);
      MotorCtrl_SetTarget(&p_push_rod->motor2_ctrl, p_push_rod->target_total_ecd);

      // 退出校准中模式，变为使能模式
      p_push_rod->state = PUSH_ROD_STATE_ENABLE;
      MotorCtrl_Enable(&p_push_rod->motor1_ctrl);
      MotorCtrl_Enable(&p_push_rod->motor2_ctrl);
      p_push_rod->calib_count = 0;
    }

    break;
  }

  case PUSH_ROD_STATE_ENABLE:
  {
    // 使能状态，正常控制

    // 同步双电机累计编码误差
    _PushRod_SyncMotorEcdError(p_push_rod);

    // 电机1计算并设定控制值
    MotorCtrl_Calc(&p_push_rod->motor1_ctrl);
    DJI_Motor_SetCmd(&p_push_rod->motor1, (int16_t)p_push_rod->motor1_ctrl.final_out);

    // 电机2计算并设定控制值
    MotorCtrl_Calc(&p_push_rod->motor2_ctrl);
    DJI_Motor_SetCmd(&p_push_rod->motor2, (int16_t)p_push_rod->motor2_ctrl.final_out);

    break;
  }

  default:
    break;
  }
}

/// @brief 推杆累计编码校准 (使能模式下调用)
void PushRod_Calibrate(PushRod_t *p_push_rod)
{
  if (p_push_rod == NULL)
  {
    return;
  }

  // 只有在使能模式下才能进入校准
  if (p_push_rod->state != PUSH_ROD_STATE_ENABLE)
  {
    return;
  }

  // 失能电机控制模块
  MotorCtrl_Disable(&p_push_rod->motor1_ctrl);
  MotorCtrl_Disable(&p_push_rod->motor2_ctrl);

  // 重置校准计数
  p_push_rod->calib_count = 0;

  // 进入校准中状态
  p_push_rod->state = PUSH_ROD_STATE_CALIBRATING;
}

/// @brief 使能推杆控制
void PushRod_Enable(PushRod_t *p_push_rod)
{
  if (p_push_rod == NULL)
  {
    return;
  }

  // 如果在校准中，不切换状态
  if (p_push_rod->state == PUSH_ROD_STATE_CALIBRATING)
  {
    return;
  }

  p_push_rod->state = PUSH_ROD_STATE_ENABLE;
  DJI_Motor_Enable(&p_push_rod->motor1);
  DJI_Motor_Enable(&p_push_rod->motor2);
  MotorCtrl_Enable(&p_push_rod->motor1_ctrl);
  MotorCtrl_Enable(&p_push_rod->motor2_ctrl);
}

/// @brief 失能推杆控制
void PushRod_Disable(PushRod_t *p_push_rod)
{
  if (p_push_rod == NULL)
  {
    return;
  }

  // 无论当前状态如何，都切换到失能
  p_push_rod->state = PUSH_ROD_STATE_DISABLE;
  MotorCtrl_Disable(&p_push_rod->motor1_ctrl);
  MotorCtrl_Disable(&p_push_rod->motor2_ctrl);
  DJI_Motor_Disable(&p_push_rod->motor1);
  DJI_Motor_Disable(&p_push_rod->motor2);

  // 清零电机控制命令
  DJI_Motor_SetCmd(&p_push_rod->motor1, 0);
  DJI_Motor_SetCmd(&p_push_rod->motor2, 0);

  // 重置校准计数
  p_push_rod->calib_count = 0;
}
