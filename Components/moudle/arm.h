/*
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2025-12-29 10:35:45
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-01-02 20:38:47
 * @FilePath: \MDK-ARMd:\RoboMaster\code\NE_RTOS_TEST\Components\moudle\arm.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
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
} Arm_t;

void Arm_Init(Arm_t *arm, ArmInit_t *init);

void Arm_SetTarget(Arm_t *arm, ArmLoad_t load, float yaw1_target, float pitch1_target, float pitch2_target, float yaw2_target);

void Arm_Enable(Arm_t *arm);

void Arm_Disable(Arm_t *arm);

void Arm_Calc(Arm_t *arm);

#endif
