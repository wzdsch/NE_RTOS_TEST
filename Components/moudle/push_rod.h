/*
 * @beforeAnnotation: 
 * Copyright (c) 2026 by 
 * """ The Robomaster team : NEXT-E from Xi'an University of Technology """
 * All Rights Reserved. 
 * 
 * @Author: Custom PushRod Module
 * @Date: 2026-02-27
 * @FilePath: \NE_RTOS_TEST\Components\motor\pushRod.h
 * @Description: 推杆机构控制模块，双M3508电机协同控制，内置前馈
 */
#ifndef PUSH_ROD_H
#define PUSH_ROD_H

#include "DJI_Motor.h"
#include "MotorCtrl.h"

#define PUSH_ROD_MAX_POS 10000
#define PUSH_ROD_MIN_POS 0

/// @brief 推杆机构状态枚举
typedef enum
{
  PUSH_ROD_STATE_DISABLE,    // 失能
  PUSH_ROD_STATE_ENABLE,     // 使能
  PUSH_ROD_STATE_CALIBRATING // 校准中
} PushRod_State_e;

/// @brief 推杆机构核心结构体
typedef struct _PushRod_t
{
  PushRod_State_e state;                  // 推杆整体状态
  DJI_Motor_t motor1;                     // 电机1指针 (主电机)
  DJI_Motor_t motor2;                     // 电机2指针 (从电机)
  MotorCtrl_t motor1_ctrl;                // 电机1控制模块
  MotorCtrl_t motor2_ctrl;                // 电机2控制模块
  
  float target_total_ecd;                 // 推杆目标累计编码值
  
  // 校准相关变量
  uint32_t calib_count;                   // 校准计数
  uint32_t calib_target_count;            // 校准目标计数 (保持电流的时间)
  int16_t calib_current;                  // 校准用电流 (编码值)

  int32_t err_total_ecd;                  // 两个电机累计编码差
  Pid_t err_pid;                          // 两个电机累计编码差的修正
} PushRod_t;

typedef struct {
  DJI_Motor_Init_t motor1_init;
  DJI_Motor_Init_t motor2_init;
  PID_Init_t motor1_pos_params;
  PID_Init_t motor1_spd_params;
  PID_Init_t motor2_pos_params;
  PID_Init_t motor2_spd_params;
  PID_Init_t err_pid_params;
  float motor1_max_out;
  float motor2_max_out;
  int16_t calib_current;
  uint32_t calib_target_count;
} PushRod_Init_t;

void PushRod_Init(PushRod_t *p_push_rod, PushRod_Init_t* p_init);

/// @brief 设置推杆目标累计编码值
/// @param p_push_rod 推杆结构体指针
/// @param target_ecd 目标累计编码值
void PushRod_SetTarget(PushRod_t *p_push_rod, float target_ecd);

/// @brief 推杆控制计算 (需在定时中断中调用，建议1ms)
/// @param p_push_rod 推杆结构体指针
void PushRod_Calc(PushRod_t *p_push_rod);

/// @brief 推杆累计编码校准 (使能模式下调用)
/// @param p_push_rod 推杆结构体指针
void PushRod_Calibrate(PushRod_t *p_push_rod);

/// @brief 使能推杆控制
/// @param p_push_rod 推杆结构体指针
void PushRod_Enable(PushRod_t *p_push_rod);

/// @brief 失能推杆控制
/// @param p_push_rod 推杆结构体指针
void PushRod_Disable(PushRod_t *p_push_rod);

#endif
