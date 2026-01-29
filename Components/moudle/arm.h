/*
 * @beforeAnnotation: 
 * Copyright (c) 2026 by 
 * """ The Robomaster team : NEXT-E from Xi'an University of Technology """
 * All Rights Reserved. 
 * 
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2025-12-29 10:35:45
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-01-29 02:48:06
 * @FilePath: \NE_RTOS_TEST\Components\moudle\arm.h
 * @Description: 
 */
#ifndef ARM_H
#define ARM_H

#include "DM_Motor.h"
#include "DJI_Motor.h"

typedef enum {
  ARM_DISABLE, // 失能
  ARM_ENABLE,  // 使能
  ARM_CALIB    // 校准中
} ArmStatus_e;

typedef enum
{
  ARM_LOAD_NONE,  // 无额外负载
  ARM_LOAD_EU     // Energy Unit 能量单元
} ArmLoad_t;

typedef struct {
  DM_Motor_Init_t yaw1_8009p_init;
  DM_Motor_Init_t pitch1_8009p_init;
  DJI_Motor_Init_t pitch2_3508_init;
  DM_Motor_Init_t yaw2_4310_init;
  DJI_Motor_Init_t end1_2006_init; // 末端执行器1
  DJI_Motor_Init_t end2_2006_init; // 末端执行器2
} ArmInit_t;

typedef struct {
  ArmStatus_e status;

  ArmLoad_t load;

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
} Arm_t;

void Arm_Init(Arm_t *arm, ArmInit_t *init);

void Arm_SetTarget(Arm_t *arm, ArmLoad_t load, float yaw1_target, float pitch1_target, float pitch2_target, float yaw2_target, \
                   float end1_target, float end2_target);

void Arm_Enable(Arm_t *arm);

void Arm_Disable(Arm_t *arm);

void Arm_Calc(Arm_t *arm);

#endif
