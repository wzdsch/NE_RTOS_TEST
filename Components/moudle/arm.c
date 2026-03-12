/*
 * @beforeAnnotation: 
 * Copyright (c) 2026 by 
 * """ The Robomaster team : NEXT-E from Xi'an University of Technology """
 * All Rights Reserved. 
 * 
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2025-12-29 10:35:45
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-03-12 20:46:32
 * @FilePath: \NE_RTOS_TEST\Components\moudle\arm.c
 * @Description: 
 */
#include "arm.h"
#include "string.h"
#include "arm_math.h"

#define K_DEG_TO_RAD 0.0174532925f

// (PI / 4096) / (36 * 2)
#define K_ARM_END_PITCH_ECD_TO_RAD 7.1017629068779686561022573587593e-6f
#define K_ARM_END_PITCH_RAD_TO_ECD 140810.10773135111178681914527115f

#define K_ARM_END_YAW_ECD_TO_RAD 3.5508814534389843280511286793795e-6f
#define K_ARM_END_YAW_RAD_TO_ECD 281620.21546270222357363829054229f

#define LIMIT(val, min, max) ((val) < (min) ? (min) : ((val) > (max) ? (max) : (val)))

float _Arm_Yaw1_GravertyFeedFwd(MotorCtrl_t *motor_ctrl);
float _Arm_Pitch1_GravertyFeedFwd(MotorCtrl_t *motor_ctrl);
float _Arm_Pitch2_GravertyFeedFwd(MotorCtrl_t *motor_ctrl);
float _Arm_Yaw2_GravertyFeedFwd(MotorCtrl_t *motor_ctrl);
float _Arm_End1_GravertyFeedFwd(MotorCtrl_t *motor_ctrl);
float _Arm_End2_GravertyFeedFwd(MotorCtrl_t *motor_ctrl);

void _Arm_End_Motor_Callback(DJI_Motor_t *p_motor);

void Arm_Init(Arm_t *p_arm, ArmInit_t* p_init) {
  JY_ME01_Init(&JY_ME01, &p_init->JY_ME01);
  p_arm->state = ARM_STATE_DISABLE;
  p_arm->load = ARM_LOAD_NONE;

  memset(&p_arm->target, 0, sizeof(ArmTarget_t));

  p_init->yaw1_8009p_init.p_owner_moudle = p_arm;
  p_init->pitch1_8009p_init.p_owner_moudle = p_arm;
  p_init->yaw2_4310_init.p_owner_moudle = p_arm;
  p_init->pitch2_3508_init.p_owner_moudle = p_arm;
  p_init->end1_2006_init.p_owner_moudle = p_arm;
  p_init->end2_2006_init.p_owner_moudle = p_arm;

  p_init->end1_2006_init.MotorRxCallback = _Arm_End_Motor_Callback;
  p_init->end2_2006_init.MotorRxCallback = _Arm_End_Motor_Callback;

  p_arm->real_end_pitch_rad = 0.f;
  p_arm->real_end_yaw_rad = 0.f;

  DM_Motor_Init(&p_arm->yaw1_8009p, &p_init->yaw1_8009p_init);
  DM_Motor_MIT_SetPD(&p_arm->yaw1_8009p, 20.f, 0.5f);
  MotorCtrl_Init(&p_arm->yaw1_ctrl, MOTOR_CTRL_PID_NONE, MOTOR_CTRL_OUT_FEEDFWD, 0.0f, &p_arm->yaw1_8009p);
  MotorCtrl_SetFeedForward(&p_arm->yaw1_ctrl, _Arm_Yaw1_GravertyFeedFwd);

  DM_Motor_Init(&p_arm->pitch1_8009p, &p_init->pitch1_8009p_init);
  DM_Motor_MIT_SetPD(&p_arm->pitch1_8009p, 20.f, 0.5f);
  MotorCtrl_Init(&p_arm->pitch1_ctrl, MOTOR_CTRL_PID_NONE, MOTOR_CTRL_OUT_FEEDFWD, 0.0f, &p_arm->pitch1_8009p);
  MotorCtrl_SetFeedForward(&p_arm->pitch1_ctrl, _Arm_Pitch1_GravertyFeedFwd);

  DJI_Motor_Init(&p_arm->pitch2_3508, &p_init->pitch2_3508_init);
  MotorCtrl_Init(&p_arm->pitch2_ctrl, MOTOR_CTRL_PID_DOUBLE, MOTOR_CTRL_OUT_PID | MOTOR_CTRL_OUT_FEEDFWD, 16384.0f, &p_arm->pitch2_3508);
  MotorCtrl_ExternalPid_Init(&p_arm->pitch2_ctrl, &JY_ME01.processed_angle, &p_init->pid_pitch2_ext);
  MotorCtrl_InternalPid_Init(&p_arm->pitch2_ctrl, &p_arm->pitch2_3508.processed_measure.spd_rpm_f, &p_init->pid_pitch2_int);
  MotorCtrl_SetFeedForward(&p_arm->pitch2_ctrl, _Arm_Pitch2_GravertyFeedFwd);
  
  DM_Motor_Init(&p_arm->yaw2_4310, &p_init->yaw2_4310_init);
  DM_Motor_MIT_SetPD(&p_arm->yaw2_4310, 1.f, 0.1f);
  MotorCtrl_Init(&p_arm->yaw2_ctrl, MOTOR_CTRL_PID_NONE, MOTOR_CTRL_OUT_FEEDFWD, 0.0f, &p_arm->yaw2_4310);
  MotorCtrl_SetFeedForward(&p_arm->yaw2_ctrl, _Arm_Yaw2_GravertyFeedFwd);

  DJI_Motor_Init(&p_arm->end1_2006, &p_init->end1_2006_init);
  MotorCtrl_Init(&p_arm->end1_ctrl, MOTOR_CTRL_PID_DOUBLE, MOTOR_CTRL_OUT_PID, 16384.0f, &p_arm->end1_2006);
  MotorCtrl_ExternalPid_Init(&p_arm->end1_ctrl, &p_arm->end1_2006.processed_measure.pos_total_ecd_f, &p_init->pid_end1_ext);
  MotorCtrl_InternalPid_Init(&p_arm->end1_ctrl, &p_arm->end1_2006.processed_measure.spd_rpm_f, &p_init->pid_end1_int);

  DJI_Motor_Init(&p_arm->end2_2006, &p_init->end2_2006_init);
  MotorCtrl_Init(&p_arm->end2_ctrl, MOTOR_CTRL_PID_DOUBLE, MOTOR_CTRL_OUT_PID, 16384.0f, &p_arm->end2_2006);
  MotorCtrl_ExternalPid_Init(&p_arm->end2_ctrl, &p_arm->end2_2006.processed_measure.pos_total_ecd_f, &p_init->pid_end2_ext);
  MotorCtrl_InternalPid_Init(&p_arm->end2_ctrl, &p_arm->end2_2006.processed_measure.spd_rpm_f, &p_init->pid_end2_int);
}

inline void Arm_SetLoad(Arm_t *p_arm, ArmLoad_t load) {
  p_arm->load = load;
}

void Arm_SetTarget(Arm_t *p_arm, float yaw1_rad, float pitch1_rad, float pitch2_deg, float yaw2_rad, float end_pitch_rad, float end_yaw_rad) {
  if (p_arm->state == ARM_STATE_DISABLE) {
    p_arm->target.pitch1_rad = p_arm->pitch1_8009p.processed_measure.pos_rad;
    p_arm->target.yaw1_rad = p_arm->yaw1_8009p.processed_measure.pos_rad;
    p_arm->target.pitch2_deg = JY_ME01.processed_angle;
    p_arm->target.yaw2_rad = p_arm->yaw2_4310.processed_measure.pos_rad;
    p_arm->target.end_pitch_rad = p_arm->real_end_pitch_rad;
    p_arm->target.end_yaw_rad = p_arm->real_end_yaw_rad;
  }
  p_arm->target.yaw1_rad = LIMIT(yaw1_rad, p_arm->yaw1_min_rad, p_arm->yaw1_max_rad);
  p_arm->target.pitch1_rad = LIMIT(pitch1_rad, p_arm->pitch1_min_rad, p_arm->pitch1_max_rad);
  p_arm->target.pitch2_deg = LIMIT(pitch2_deg, p_arm->pitch2_min_deg, p_arm->pitch2_max_deg);
  p_arm->target.yaw2_rad = LIMIT(yaw2_rad, p_arm->yaw2_min_rad, p_arm->yaw2_max_rad);
  p_arm->target.end_pitch_rad = LIMIT(end_pitch_rad, p_arm->end_pitch_min_rad, p_arm->end_pitch_max_rad);
  p_arm->target.end_yaw_rad = end_yaw_rad;

  p_arm->yaw1_ctrl.target = p_arm->target.yaw1_rad;
  p_arm->pitch1_ctrl.target = p_arm->target.pitch1_rad;
  p_arm->pitch2_ctrl.target = p_arm->target.pitch2_deg;
  p_arm->yaw2_ctrl.target = p_arm->target.yaw2_rad;
  p_arm->end1_ctrl.target = p_arm->target.end_pitch_rad * K_ARM_END_PITCH_RAD_TO_ECD + \
                            p_arm->target.end_yaw_rad * K_ARM_END_YAW_RAD_TO_ECD;
  p_arm->end2_ctrl.target = -p_arm->target.end_pitch_rad * K_ARM_END_PITCH_RAD_TO_ECD + \
                            p_arm->target.end_yaw_rad * K_ARM_END_YAW_RAD_TO_ECD;

  DM_Motor_MIT_SetPos(&p_arm->yaw1_8009p, p_arm->target.yaw1_rad);
  DM_Motor_MIT_SetPos(&p_arm->pitch1_8009p, p_arm->target.pitch1_rad);
  DM_Motor_MIT_SetPos(&p_arm->yaw2_4310, p_arm->target.yaw2_rad);
}

void Arm_Enable(Arm_t *p_arm) {
  p_arm->state = ARM_STATE_ENABLE;
  DM_Motor_Enable(&p_arm->yaw1_8009p);
  DM_Motor_Enable(&p_arm->pitch1_8009p);
  DJI_Motor_Enable(&p_arm->pitch2_3508);
  DM_Motor_Enable(&p_arm->yaw2_4310);
  DJI_Motor_Enable(&p_arm->end1_2006);
  DJI_Motor_Enable(&p_arm->end2_2006);

  MotorCtrl_Enable(&p_arm->yaw1_ctrl);
  MotorCtrl_Enable(&p_arm->pitch1_ctrl);
  MotorCtrl_Enable(&p_arm->pitch2_ctrl);
  MotorCtrl_Enable(&p_arm->yaw2_ctrl);
  MotorCtrl_Enable(&p_arm->end1_ctrl);
  MotorCtrl_Enable(&p_arm->end2_ctrl);
}

void Arm_Disable(Arm_t *p_arm) {
  p_arm->state = ARM_STATE_DISABLE;
  DM_Motor_Disable(&p_arm->yaw1_8009p);
  DM_Motor_Disable(&p_arm->pitch1_8009p);
  DJI_Motor_Disable(&p_arm->pitch2_3508);
  DM_Motor_Disable(&p_arm->yaw2_4310);
  DJI_Motor_Disable(&p_arm->end1_2006);
  DJI_Motor_Disable(&p_arm->end2_2006);

  MotorCtrl_Disable(&p_arm->yaw1_ctrl);
  MotorCtrl_Disable(&p_arm->pitch1_ctrl);
  MotorCtrl_Disable(&p_arm->pitch2_ctrl);
  MotorCtrl_Disable(&p_arm->yaw2_ctrl);
  MotorCtrl_Disable(&p_arm->end1_ctrl);
  MotorCtrl_Disable(&p_arm->end2_ctrl);
}

void Arm_Calc(Arm_t *p_arm) {
  if (p_arm->state == ARM_STATE_DISABLE) {
    DM_Motor_MIT_SetTorq(&p_arm->yaw1_8009p, 0);
    DM_Motor_MIT_SetTorq(&p_arm->pitch1_8009p, 0);
    DJI_Motor_SetCmd(&p_arm->pitch2_3508, 0);
    DM_Motor_MIT_SetTorq(&p_arm->yaw2_4310, 0);
    DJI_Motor_SetCmd(&p_arm->end1_2006, 0);
    DJI_Motor_SetCmd(&p_arm->end2_2006, 0);
    return;
  } else if (p_arm->state == ARM_STATE_ENABLE) {
    MotorCtrl_Calc(&p_arm->yaw1_ctrl);
    MotorCtrl_Calc(&p_arm->pitch1_ctrl);
    MotorCtrl_Calc(&p_arm->pitch2_ctrl);
    MotorCtrl_Calc(&p_arm->yaw2_ctrl);
    MotorCtrl_Calc(&p_arm->end1_ctrl);
    MotorCtrl_Calc(&p_arm->end2_ctrl);

    DM_Motor_MIT_SetTorq(&p_arm->yaw1_8009p, p_arm->yaw1_ctrl.final_out);
    DM_Motor_MIT_SetTorq(&p_arm->pitch1_8009p, p_arm->pitch1_ctrl.final_out);
    DJI_Motor_SetCmd(&p_arm->pitch2_3508, p_arm->pitch2_ctrl.final_out);
    DM_Motor_MIT_SetTorq(&p_arm->yaw2_4310, p_arm->yaw2_ctrl.final_out);
    DJI_Motor_SetCmd(&p_arm->end1_2006, p_arm->end1_ctrl.final_out);
    DJI_Motor_SetCmd(&p_arm->end2_2006, p_arm->end2_ctrl.final_out);
  }
}

float tau5 = 0.01f;
float tau4 = 0.01f;
float tau3 = 0.01f;
float tau34_min = 0.01f;
float tau34_max = 0.02f;
float tau2 = 0.01f;

float _Arm_Yaw1_GravertyFeedFwd(MotorCtrl_t *p_ctrl) {
  return 0;
}

float _Arm_Pitch1_GravertyFeedFwd(MotorCtrl_t *p_ctrl) {
  return 0;
}

float _Arm_Pitch2_GravertyFeedFwd(MotorCtrl_t *p_ctrl) {
  return 0;
}

float _Arm_Yaw2_GravertyFeedFwd(MotorCtrl_t *p_ctrl) {
  return 0;
}

float _Arm_End1_GravertyFeedFwd(MotorCtrl_t *p_ctrl) {
  return 0;
}

float _Arm_End2_GravertyFeedFwd(MotorCtrl_t *p_ctrl) {
  Arm_t* p_arm = (Arm_t *)(((DJI_Motor_t *)(p_ctrl->p_owner_moudle))->p_owner_moudle);
  return tau5 * arm_cos_f32(p_arm->pitch1_8009p.processed_measure.pos_rad + \
                            JY_ME01.processed_angle * K_DEG_TO_RAD + \
                            p_arm->real_end_pitch_rad * cos(p_arm->yaw1_8009p.processed_measure.pos_rad));
}

void _Arm_End_Motor_Callback(DJI_Motor_t *p_motor) {
  Arm_t* p_arm = (Arm_t *)(p_motor->p_owner_moudle);
  p_arm->real_end_pitch_rad = K_ARM_END_PITCH_ECD_TO_RAD * \
    (p_arm->end1_2006.processed_measure.pos_total_ecd - p_arm->end2_2006.processed_measure.pos_total_ecd) / 2.f;
  p_arm->real_end_yaw_rad = K_ARM_END_YAW_ECD_TO_RAD * \
    (p_arm->end1_2006.processed_measure.pos_total_ecd + p_arm->end2_2006.processed_measure.pos_total_ecd) / 2.f;
}
