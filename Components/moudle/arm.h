/*
 * @beforeAnnotation: 
 * Copyright (c) 2026 by 
 * """ The Robomaster team : NEXT-E from Xi'an University of Technology """
 * All Rights Reserved. 
 * 
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2025-12-29 10:35:45
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-03-07 16:49:25
 * @FilePath: \NE_RTOS_TEST\Components\moudle\arm.h
 * @Description: 
 */
#ifndef ARM_H
#define ARM_H

#include "DM_Motor.h"
#include "DJI_Motor.h"
#include "JY_ME01.h"

typedef enum {
  ARM_DISABLE, // 失能
  ARM_ENABLE,  // 使能
  ARM_CALIB    // 校准中
} ArmState_e;

typedef enum
{
  ARM_LOAD_NONE,  // 无额外负载
  ARM_LOAD_EU     // Energy Unit 能量单元
} ArmLoad_t;

typedef struct {
  JY_ME01_Init_t JY_ME01;
  DM_Motor_Init_t yaw1_8009p_init;
  DM_Motor_Init_t pitch1_8009p_init;
  DJI_Motor_Init_t pitch2_3508_init;
  DM_Motor_Init_t yaw2_4310_init;
  DJI_Motor_Init_t end1_2006_init; // 末端执行器1
  DJI_Motor_Init_t end2_2006_init; // 末端执行器2
  PID_Init_t pid_pitch2_ext; // pitch2外环PID参数
  PID_Init_t pid_pitch2_int; // pitch2内环PID参数
  PID_Init_t pid_end1_ext; // end1外环PID参数
  PID_Init_t pid_end1_int; // end1内环PID参数
  PID_Init_t pid_end2_ext; // end2外环PID参数
  PID_Init_t pid_end2_int; // end2内环PID参数
} ArmInit_t;

typedef struct {
  float yaw1;
  float pitch1;
  float pitch2;
  float yaw2;
  float end_pitch;
  float end_yaw;
} ArmTarget_t;

typedef struct {
  ArmState_e state;
  ArmLoad_t load;

  ArmTarget_t target;

  DM_Motor_t yaw1_8009p;
  MotorCtrl_t yaw1_ctrl;

  DM_Motor_t pitch1_8009p;
  MotorCtrl_t pitch1_ctrl;

  DJI_Motor_t pitch2_3508;
  MotorCtrl_t pitch2_ctrl;

  DM_Motor_t yaw2_4310;
  MotorCtrl_t yaw2_ctrl;

  DJI_Motor_t end1_2006;
  MotorCtrl_t end1_ctrl;

  DJI_Motor_t end2_2006;
  MotorCtrl_t end2_ctrl;

  float real_end_pitch_rad;
  float real_end_yaw_rad;
} Arm_t;

extern Arm_t arm;

void Arm_Init(Arm_t *p_arm, ArmInit_t *init);

void Arm_SetLoad(Arm_t *p_arm, ArmLoad_t load);

/// @brief 从模块内部的target结构体设定电机目标值, 进行限幅等处理
/// @note 需要将目标值写入target结构体后调用此函数, 会设定目标并自动更新target
/// @param p_arm 
void Arm_SetTarget(Arm_t *p_arm);

void Arm_Enable(Arm_t *p_arm);

void Arm_Disable(Arm_t *p_arm);

void Arm_Calc(Arm_t *p_arm);

#endif
