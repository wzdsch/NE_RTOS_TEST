/*
 * @beforeAnnotation: 
 * Copyright (c) 2026 by 
 * """ The Robomaster team : NEXT-E from Xi'an University of Technology """
 * All Rights Reserved. 
 * 
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2025-12-29 10:35:45
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-01-23 23:48:39
 * @FilePath: \NE_RTOS_TEST\Components\moudle\arm.c
 * @Description: 
 */
#include "arm.h"

float PITCH2_3508_EXT_PID_DATA[5] = {1.f, .0f, .0f, 8000.f, 2000.f};
float PITCH2_3508_INT_PID_DATA[5] = {1.f, .0f, .0f, 16384.f, 5000.f};

float _Arm_Yaw1_GravertyFeedFwd(MotorCtrl_t *motor_ctrl);
float _Arm_Pitch1_GravertyFeedFwd(MotorCtrl_t *motor_ctrl);
float _Arm_Pitch2_GravertyFeedFwd(MotorCtrl_t *motor_ctrl);
float _Arm_Yaw2_GravertyFeedFwd(MotorCtrl_t *motor_ctrl);

void Arm_Init(Arm_t *arm, ArmInit_t* init) {
  arm->status = ARM_DISABLE;
  arm->load = ARM_LOAD_NONE;
  DM_Motor_Init(&arm->yaw1_8009p, &init->yaw1_8009p_init);
  MotorCtrl_Init(&arm->yaw1_ctrl, MOTOR_CTRL_PID_NONE, MOTOR_CTRL_OUT_FEEDFWD, 40.0f, &arm->yaw1_8009p);
  MotorCtrl_SetFeedForward(&arm->yaw1_ctrl, _Arm_Yaw1_GravertyFeedFwd);

  DM_Motor_Init(&arm->pitch1_8009p, &init->pitch1_8009p_init);
  MotorCtrl_Init(&arm->pitch1_ctrl, MOTOR_CTRL_PID_NONE, MOTOR_CTRL_OUT_FEEDFWD, 40.0f, &arm->pitch1_8009p);
  MotorCtrl_SetFeedForward(&arm->pitch1_ctrl, _Arm_Pitch1_GravertyFeedFwd);

  DJI_Motor_Init(&arm->pitch2_3508, &init->pitch2_3508_init);
  MotorCtrl_Init(&arm->pitch2_ctrl, MOTOR_CTRL_PID_DOUBLE, MOTOR_CTRL_OUT_PID | MOTOR_CTRL_OUT_FEEDFWD, 16384.0f, &arm->pitch2_3508);
  MotorCtrl_ExternalPid_Init(&arm->pitch2_ctrl, PID_POSITION, &arm->pitch2_3508.processed_measure.pos_total_ecd_f, PITCH2_3508_EXT_PID_DATA);
  MotorCtrl_InternalPid_Init(&arm->pitch2_ctrl, PID_POSITION, &arm->pitch2_3508.processed_measure.spd_rpm_f, PITCH2_3508_INT_PID_DATA);
  MotorCtrl_SetFeedForward(&arm->pitch2_ctrl, _Arm_Pitch2_GravertyFeedFwd);
  
  DM_Motor_Init(&arm->yaw2_4310, &init->yaw2_4310_init);
  MotorCtrl_Init(&arm->yaw2_ctrl, MOTOR_CTRL_PID_NONE, MOTOR_CTRL_OUT_FEEDFWD, 40.0f, &arm->yaw2_4310);
  MotorCtrl_SetFeedForward(&arm->yaw2_ctrl, _Arm_Yaw2_GravertyFeedFwd);
}

inline void Arm_SetTarget(Arm_t *arm, ArmLoad_t load, float yaw1_target, float pitch1_target, float pitch2_target, float yaw2_target) {
  arm->load = load;
  arm->yaw1_ctrl.set_target = yaw1_target;
  arm->pitch1_ctrl.set_target = pitch1_target;
  arm->pitch2_ctrl.set_target = pitch2_target;
  arm->yaw2_ctrl.set_target = yaw2_target;
  DM_Motor_MIT_SetPos(&arm->yaw1_8009p, arm->yaw1_ctrl.set_target);
  DM_Motor_MIT_SetPos(&arm->pitch1_8009p, arm->pitch1_ctrl.set_target);
  DM_Motor_MIT_SetPos(&arm->yaw2_4310, arm->yaw2_ctrl.set_target);
}

void Arm_Enable(Arm_t *arm) {
  arm->status = ARM_ENABLE;
  DM_Motor_Enable(&arm->yaw1_8009p);
  DM_Motor_Enable(&arm->pitch1_8009p);
  DJI_Motor_Enable(&arm->pitch2_3508);
  DM_Motor_Enable(&arm->yaw2_4310);

  MotorCtrl_Enable(&arm->yaw1_ctrl);
  MotorCtrl_Enable(&arm->pitch1_ctrl);
  MotorCtrl_Enable(&arm->pitch2_ctrl);
  MotorCtrl_Enable(&arm->yaw2_ctrl);
}

void Arm_Disable(Arm_t *arm) {
  arm->status = ARM_DISABLE;
  DM_Motor_Disable(&arm->yaw1_8009p);
  DM_Motor_Disable(&arm->pitch1_8009p);
  DJI_Motor_Disable(&arm->pitch2_3508);
  DM_Motor_Disable(&arm->yaw2_4310);

  MotorCtrl_Disable(&arm->yaw1_ctrl);
  MotorCtrl_Disable(&arm->pitch1_ctrl);
  MotorCtrl_Disable(&arm->pitch2_ctrl);
  MotorCtrl_Disable(&arm->yaw2_ctrl);
}

void Arm_Calc(Arm_t *arm) {
  if (arm->status == ARM_DISABLE) {
    return;
  } else if (arm->status == ARM_ENABLE) {
    MotorCtrl_Calc(&arm->yaw1_ctrl);
    MotorCtrl_Calc(&arm->pitch1_ctrl);
    MotorCtrl_Calc(&arm->pitch2_ctrl);
    MotorCtrl_Calc(&arm->yaw2_ctrl);

    DM_Motor_MIT_SetTorq(&arm->yaw1_8009p, arm->yaw1_ctrl.final_out);
    DM_Motor_MIT_SetTorq(&arm->pitch1_8009p, arm->pitch1_ctrl.final_out);
    DJI_Motor_SetCmd(&arm->pitch2_3508, arm->pitch2_ctrl.final_out);
    DM_Motor_MIT_SetTorq(&arm->yaw2_4310, arm->yaw2_ctrl.final_out);
  }
}

float _Arm_Yaw1_GravertyFeedFwd(MotorCtrl_t *motor_ctrl) {
  return 0;
}

float _Arm_Pitch1_GravertyFeedFwd(MotorCtrl_t *motor_ctrl) {
  return 0;
}

float _Arm_Pitch2_GravertyFeedFwd(MotorCtrl_t *motor_ctrl) {
  return 0;
}

float _Arm_Yaw2_GravertyFeedFwd(MotorCtrl_t *motor_ctrl) {
  return 0;
}
