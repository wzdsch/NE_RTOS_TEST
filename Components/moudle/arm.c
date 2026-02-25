/*
 * @beforeAnnotation: 
 * Copyright (c) 2026 by 
 * """ The Robomaster team : NEXT-E from Xi'an University of Technology """
 * All Rights Reserved. 
 * 
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2025-12-29 10:35:45
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-01-29 08:11:01
 * @FilePath: \NE_RTOS_TEST\Components\moudle\arm.c
 * @Description: 
 */
#include "arm.h"
#include "JY_ME01.h"

Arm_t arm;

float PITCH2_3508_EXT_PID_DATA[5] = {1.f, .0f, .0f, 8000.f, 2000.f};
float PITCH2_3508_INT_PID_DATA[5] = {1.f, .0f, .0f, 5000.f, 2000.f};

float END1_2006_EXT_PID_DATA[5] = {0.05f, .0f, .01f, 8000.f, 2000.f};
float END1_2006_INT_PID_DATA[5] = {1.f, .0f, .0f, 16384.f, 2000.f};

float END2_2006_EXT_PID_DATA[5] = {0.05f, .0f, .01f, 8000.f, 2000.f};
float END2_2006_INT_PID_DATA[5] = {1.f, .0f, .0f, 16384.f, 2000.f};

float _Arm_Yaw1_GravertyFeedFwd(MotorCtrl_t *motor_ctrl);
float _Arm_Pitch1_GravertyFeedFwd(MotorCtrl_t *motor_ctrl);
float _Arm_Pitch2_GravertyFeedFwd(MotorCtrl_t *motor_ctrl);
float _Arm_Yaw2_GravertyFeedFwd(MotorCtrl_t *motor_ctrl);

void Arm_Init(Arm_t *arm, ArmInit_t* init) {
  JY_ME01_Init(&JY_ME01, &huart1);
  arm->state = ARM_DISABLE;
  arm->load = ARM_LOAD_NONE;
  DM_Motor_Init(&arm->yaw1_8009p, &init->yaw1_8009p_init);
  DM_Motor_MIT_SetPD(&arm->yaw1_8009p, 20.f, 0.5f);
  MotorCtrl_Init(&arm->yaw1_ctrl, MOTOR_CTRL_PID_NONE, MOTOR_CTRL_OUT_FEEDFWD, 0.0f, &arm->yaw1_8009p);
  MotorCtrl_SetFeedForward(&arm->yaw1_ctrl, _Arm_Yaw1_GravertyFeedFwd);

  DM_Motor_Init(&arm->pitch1_8009p, &init->pitch1_8009p_init);
  DM_Motor_MIT_SetPD(&arm->pitch1_8009p, 20.f, 0.5f);
  MotorCtrl_Init(&arm->pitch1_ctrl, MOTOR_CTRL_PID_NONE, MOTOR_CTRL_OUT_FEEDFWD, 0.0f, &arm->pitch1_8009p);
  MotorCtrl_SetFeedForward(&arm->pitch1_ctrl, _Arm_Pitch1_GravertyFeedFwd);

  DJI_Motor_Init(&arm->pitch2_3508, &init->pitch2_3508_init);
  MotorCtrl_Init(&arm->pitch2_ctrl, MOTOR_CTRL_PID_DOUBLE, MOTOR_CTRL_OUT_PID | MOTOR_CTRL_OUT_FEEDFWD, 16384.0f, &arm->pitch2_3508);
  MotorCtrl_ExternalPid_Init(&arm->pitch2_ctrl, PID_POSITION, &JY_ME01.angle, PITCH2_3508_EXT_PID_DATA);
  MotorCtrl_InternalPid_Init(&arm->pitch2_ctrl, PID_POSITION, &arm->pitch2_3508.processed_measure.spd_rpm_f, PITCH2_3508_INT_PID_DATA);
  MotorCtrl_SetFeedForward(&arm->pitch2_ctrl, _Arm_Pitch2_GravertyFeedFwd);
  
  DM_Motor_Init(&arm->yaw2_4310, &init->yaw2_4310_init);
  DM_Motor_MIT_SetPD(&arm->yaw2_4310, 1.f, 0.1f);
  MotorCtrl_Init(&arm->yaw2_ctrl, MOTOR_CTRL_PID_NONE, MOTOR_CTRL_OUT_FEEDFWD, 0.0f, &arm->yaw2_4310);
  MotorCtrl_SetFeedForward(&arm->yaw2_ctrl, _Arm_Yaw2_GravertyFeedFwd);

  DJI_Motor_Init(&arm->end1_2006, &init->end1_2006_init);
  MotorCtrl_Init(&arm->end1_ctrl, MOTOR_CTRL_PID_DOUBLE, MOTOR_CTRL_OUT_PID, 16384.0f, &arm->end1_2006);
  MotorCtrl_ExternalPid_Init(&arm->end1_ctrl, PID_POSITION, &arm->end1_2006.processed_measure.pos_total_ecd_f, END1_2006_EXT_PID_DATA);
  MotorCtrl_InternalPid_Init(&arm->end1_ctrl, PID_POSITION, &arm->end1_2006.processed_measure.spd_rpm_f, END1_2006_INT_PID_DATA);

  DJI_Motor_Init(&arm->end2_2006, &init->end2_2006_init);
  MotorCtrl_Init(&arm->end2_ctrl, MOTOR_CTRL_PID_DOUBLE, MOTOR_CTRL_OUT_PID, 16384.0f, &arm->end2_2006);
  MotorCtrl_ExternalPid_Init(&arm->end2_ctrl, PID_POSITION, &arm->end2_2006.processed_measure.pos_total_ecd_f, END2_2006_EXT_PID_DATA);
  MotorCtrl_InternalPid_Init(&arm->end2_ctrl, PID_POSITION, &arm->end2_2006.processed_measure.spd_rpm_f, END2_2006_INT_PID_DATA);
}

inline void Arm_SetTarget(Arm_t *arm, ArmLoad_t load, float yaw1_target, float pitch1_target, float pitch2_target, float yaw2_target, \
                         float end_pitch, float end_yaw) {
  arm->load = load;
  arm->yaw1_ctrl.target = yaw1_target;
  arm->pitch1_ctrl.target = pitch1_target;
  arm->pitch2_ctrl.target = pitch2_target;
  arm->yaw2_ctrl.target = yaw2_target;
  arm->end1_ctrl.target = end_pitch + end_yaw;
  arm->end2_ctrl.target = -end_pitch + end_yaw;

  DM_Motor_MIT_SetPos(&arm->yaw1_8009p, arm->yaw1_ctrl.target);
  DM_Motor_MIT_SetPos(&arm->pitch1_8009p, arm->pitch1_ctrl.target);
  DM_Motor_MIT_SetPos(&arm->yaw2_4310, arm->yaw2_ctrl.target);
}

void Arm_Enable(Arm_t *arm) {
  arm->state = ARM_ENABLE;
  DM_Motor_Enable(&arm->yaw1_8009p);
  DM_Motor_Enable(&arm->pitch1_8009p);
  DJI_Motor_Enable(&arm->pitch2_3508);
  DM_Motor_Enable(&arm->yaw2_4310);
  DJI_Motor_Enable(&arm->end1_2006);
  DJI_Motor_Enable(&arm->end2_2006);

  MotorCtrl_Enable(&arm->yaw1_ctrl);
  MotorCtrl_Enable(&arm->pitch1_ctrl);
  MotorCtrl_Enable(&arm->pitch2_ctrl);
  MotorCtrl_Enable(&arm->yaw2_ctrl);
  MotorCtrl_Enable(&arm->end1_ctrl);
  MotorCtrl_Enable(&arm->end2_ctrl);
}

void Arm_Disable(Arm_t *arm) {
  arm->state = ARM_DISABLE;
  DM_Motor_Disable(&arm->yaw1_8009p);
  DM_Motor_Disable(&arm->pitch1_8009p);
  DJI_Motor_Disable(&arm->pitch2_3508);
  DM_Motor_Disable(&arm->yaw2_4310);
  DJI_Motor_Disable(&arm->end1_2006);
  DJI_Motor_Disable(&arm->end2_2006);

  MotorCtrl_Disable(&arm->yaw1_ctrl);
  MotorCtrl_Disable(&arm->pitch1_ctrl);
  MotorCtrl_Disable(&arm->pitch2_ctrl);
  MotorCtrl_Disable(&arm->yaw2_ctrl);
  MotorCtrl_Disable(&arm->end1_ctrl);
  MotorCtrl_Disable(&arm->end2_ctrl);
}

void Arm_Calc(Arm_t *arm) {
  if (arm->state == ARM_DISABLE) {
    DM_Motor_MIT_SetTorq(&arm->yaw1_8009p, 0);
    DM_Motor_MIT_SetTorq(&arm->pitch1_8009p, 0);
    DJI_Motor_SetCmd(&arm->pitch2_3508, 0);
    DM_Motor_MIT_SetTorq(&arm->yaw2_4310, 0);
    DJI_Motor_SetCmd(&arm->end1_2006, 0);
    DJI_Motor_SetCmd(&arm->end2_2006, 0);
    return;
  } else if (arm->state == ARM_ENABLE) {
    MotorCtrl_Calc(&arm->yaw1_ctrl);
    MotorCtrl_Calc(&arm->pitch1_ctrl);
    MotorCtrl_Calc(&arm->pitch2_ctrl);
    MotorCtrl_Calc(&arm->yaw2_ctrl);
    MotorCtrl_Calc(&arm->end1_ctrl);
    MotorCtrl_Calc(&arm->end2_ctrl);

    DM_Motor_MIT_SetTorq(&arm->yaw1_8009p, arm->yaw1_ctrl.final_out);
    DM_Motor_MIT_SetTorq(&arm->pitch1_8009p, arm->pitch1_ctrl.final_out);
    DJI_Motor_SetCmd(&arm->pitch2_3508, arm->pitch2_ctrl.final_out);
    DM_Motor_MIT_SetTorq(&arm->yaw2_4310, arm->yaw2_ctrl.final_out);
    DJI_Motor_SetCmd(&arm->end1_2006, arm->end1_ctrl.final_out);
    DJI_Motor_SetCmd(&arm->end2_2006, arm->end2_ctrl.final_out);
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
