/*
 * @beforeAnnotation: 
 * Copyright (c) 2026 by 
 * """ The Robomaster team : NEXT-E from Xi'an University of Technology """
 * All Rights Reserved.
 * 
 * @Description: 双M3508电机推杆机构控制模块
 */
#ifndef PUSH_ROD_H
#define PUSH_ROD_H

#include "DJI_Motor.h"
#include "MotorCtrl.h"
#include "pid.h"

// 推杆状态枚举
typedef enum
{
  PUSH_ROD_STATE_DISABLE,    // 失能
  PUSH_ROD_STATE_ENABLE,     // 使能
  PUSH_ROD_STATE_CALIBRATING // 校准中
} PushRod_State_e;

// 推杆结构体
typedef struct _PushRod_t
{
  PushRod_State_e state; // 推杆整体状态

  // 电机实例
  DJI_Motor_t motor_1;  // 电机1
  DJI_Motor_t motor_2;  // 电机2

  // 电机控制实例 (位置环)
  MotorCtrl_t ctrl_1;  // 电机1控制
  MotorCtrl_t ctrl_2;  // 电机2控制

  // 双电机误差控制PID
  Pid_t err_pid;
  float err_pid_out; // 误差PID输出

  // 控制参数
  float target_total_ecd; // 目标累计编码值
  float max_allowed_err;  // 双电机最大允许误差

  // 前馈相关
  float gravity_feedforward; // 重力前馈基础值

  // 校准与掉电检测
  uint32_t last_update_tick_1;  // 电机1最后更新时间
  uint32_t last_update_tick_2;  // 电机2最后更新时间
  uint32_t offline_timeout;      // 离线超时时间 (ms)
  uint8_t is_calibrated;         // 是否校准完成
  uint8_t need_calib_1;          // 电机1需要校准
  uint8_t need_calib_2;          // 电机2需要校准

} PushRod_t;

// 推杆初始化结构体
typedef struct
{
  // 电机1配置
  CAN_HandleTypeDef *hcan_1;
  DJI_Motor_TxID_e tx_id_1;
  uint32_t rx_id_1;
  DJI_Motor_Dir_e dir_1;
  int16_t zero_offset_1;

  // 电机2配置
  CAN_HandleTypeDef *hcan_2;
  DJI_Motor_TxID_e tx_id_2;
  uint32_t rx_id_2;
  DJI_Motor_Dir_e dir_2;
  int16_t zero_offset_2;

  // 位置环PID参数 (共用)
  float pos_kp;
  float pos_ki;
  float pos_kd;
  float pos_out_limit;
  float pos_int_limit;
  float max_out; // 电机最大输出

  // 误差环PID参数
  float err_kp;
  float err_ki;
  float err_kd;
  float err_out_limit;
  float err_int_limit;

  // 配置参数
  float max_allowed_err;
  float gravity_feedforward;
  uint32_t offline_timeout;
} PushRod_Init_t;

/// @brief 推杆初始化
/// @param p_rod 推杆指针
/// @param init 初始化结构体
void PushRod_Init(PushRod_t *const p_rod, const PushRod_Init_t *const init);

/// @brief 推杆使能
/// @param p_rod 推杆指针
void PushRod_Enable(PushRod_t *const p_rod);

/// @brief 推杆失能
/// @param p_rod 推杆指针
void PushRod_Disable(PushRod_t *const p_rod);

/// @brief 设置目标位置 (累计编码值)
/// @param p_rod 推杆指针
/// @param target 目标累计编码值
void PushRod_SetTarget(PushRod_t *const p_rod, float target);

/// @brief 推杆控制计算 (需在任务/定时器中周期调用)
/// @param p_rod 推杆指针
void PushRod_Calc(PushRod_t *const p_rod);

/// @brief 触发手动校准
/// @param p_rod 推杆指针
void PushRod_StartCalibration(PushRod_t *const p_rod);

#endif
