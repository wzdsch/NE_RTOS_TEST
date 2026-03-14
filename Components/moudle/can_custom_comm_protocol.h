
/*
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2026-02-17 19:36:22
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-03-13 16:54:53
 * @FilePath: \proj_chassisd:\RoboMaster\code\Enginner\Components\moudle\can_custom_comm_protocol.h
 * @Description: CAN自定义通信模块，通信结构体及打包/解包函数
 */
#ifndef CAN_CUSTOM_PROTOCOL_H
#define CAN_CUSTOM_PROTOCOL_H

#include "arm.h"
#include "mecnum_chassis.h"
#include "push_rod.h"

#pragma pack(1)

typedef struct {
  ArmState_e state;
  float yaw1_tar_rad;
  float pitch1_tar_rad;
  float pitch2_tar_deg;
  float yaw2_tar_rad;
  float end_pitch_tar_rad;
  float end_yaw_tar_rad;
} CAN_Custom_ArmCtrlData_t;

/// @brief ArmSetTarget后, arm模块会自动处理并设置target
/// 这个结构体就是另一个机械臂反馈的处理后的target
/// 也相当于ArmTarget_t
typedef struct {
  float yaw_rad;
  float pitch1_rad;
  float pitch2_deg;
  float yaw2_rad;
  float end_pitch_rad;
  float end_yaw_rad;
} CAN_Custom_ArmFdbTarget_t;

typedef struct {
  Chassis_State_e state;
  int16_t spd_x;
  int16_t spd_y;
  int16_t spd_z;
} CAN_Custom_ChassisCtrlData_t;

typedef struct {
  PushRod_State_e state_f;  // 前推杆状态
  PushRod_State_e state_b;  // 后推杆状态
  uint8_t pos_f : 1;        // 前推杆伸出还是收回 1:伸出 0:收回
  uint8_t pos_b : 1;        // 后推杆伸出还是收回 1:伸出 0:收回
} CAN_Custom_PushRodsCtrlData_t;

#pragma pack()

#endif
