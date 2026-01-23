/*
 * @beforeAnnotation: 
 * Copyright (c) 2026 by 
 * """ The Robomaster team : NEXT-E from Xi'an University of Technology """
 * All Rights Reserved. 
 * 
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2025-11-18 12:34:54
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-01-23 23:48:17
 * @FilePath: \NE_RTOS_TEST\Components\motor\DM_Motor_typedef.h
 * @Description: 
 */
#ifndef DM_MOTOR_TYPEDEF_H
#define DM_MOTOR_TYPEDEF_H

#include <stdint.h>

// 是否启用相关发送模式 1:启用 0:不启用
// 不使用建议给0 减少RAM占用
#define DM_TX_USE_MIT_MODE 1
#define DM_TX_USE_POS_SPD_MODE 1
#define DM_TX_USE_SPD_MODE 1

// 达妙不同模式发送id的偏移量
#define DM_POS_SPD_MODE_ID_BASE 0x100
#define DM_SPD_MODE_ID_BASE 0x200

#if DM_TX_USE_MIT_MODE == 1
// -------------------- MIT ------------------- //

#define DM_MIT_POS_BITS 16
#define DM_MIT_SPD_BITS 16
#define DM_MIT_KP_BITS 12
#define DM_MIT_KD_BITS 12
#define DM_MIT_TOR_BITS 12

typedef struct {
  float pos_rad; // 位置 单位: 弧度
  float spd_radps; // 速度 单位: 弧度/秒

  float kp; // 位置系数
  float kd; // 速度系数 (对位置进行控制时，kd 不能赋 0，否则会造成电机震荡，甚至失控)

  float torq_nm; // 力矩 单位: 牛米
} _DM_Motor_MIT_CtrlData_t;

typedef struct {
  uint8_t txd[8];
  _DM_Motor_MIT_CtrlData_t ctrl_data;
} DM_Motor_MIT_Data_t;

#endif

#if DM_TX_USE_POS_SPD_MODE == 1
// -------------------- POS_SPD ------------------- //

#pragma pack(1)
typedef struct {
  float pos_rad;
  float spd_radps;
} _DM_Motor_POS_SPD_CtrlData_t;

typedef struct {
  uint8_t txd[8];
  _DM_Motor_POS_SPD_CtrlData_t ctrl_data;
} DM_Motor_POS_SPD_Data_t;
#pragma pack()

#endif

#if DM_TX_USE_SPD_MODE == 1
// -------------------- SPD ------------------- //

typedef struct {
  uint8_t txd[4];
  float spd_radps;
} DM_Motor_SPD_Data_t;

#endif

#endif
