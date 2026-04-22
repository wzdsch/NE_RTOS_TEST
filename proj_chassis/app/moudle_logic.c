/*
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2026-02-22 19:49:50
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-04-07 20:52:37
 * @FilePath: \NE_RTOS_TEST\Components\app\robot_logic.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "moudle_logic.h"
#include "push_rod.h"
#include "mecnum_chassis.h"
#include "can_custom_structs.h"

extern Chassis_t chassis;    // 底盘结构体
extern PushRod_t push_rod_f; // 前推杆结构体
extern PushRod_t push_rod_b; // 后推杆结构体

extern CAN_Custom_ChassisCtrlData_t chassis_ctrl_data;   // 底盘控制数据
extern CAN_Custom_PushRodsCtrlData_t push_rods_ctrl_data; // 推杆控制数据

#define RC_JOY_DEADBAND 10

void PushRod_Logic() {
  if (push_rods_ctrl_data.state_f == PUSH_ROD_STATE_ENABLE) {
    PushRod_Enable(&push_rod_f);
    if (push_rods_ctrl_data.pos_f == 1) {
      PushRod_SetTarget(&push_rod_f, push_rod_f.max_ecd);
    }
    else {
      PushRod_SetTarget(&push_rod_f, push_rod_f.min_ecd);
    }
    PushRod_Calc(&push_rod_f);
  }
  else if (push_rods_ctrl_data.state_f == PUSH_ROD_STATE_CALIBRATING) {
    PushRod_Calibrate(&push_rod_f);
  }
  else {
    PushRod_Disable(&push_rod_f);
  }

  if (push_rods_ctrl_data.state_b == PUSH_ROD_STATE_ENABLE) {
    PushRod_Enable(&push_rod_b);
    if (push_rods_ctrl_data.pos_b == 1) {
      PushRod_SetTarget(&push_rod_b, push_rod_b.max_ecd);
    }
    else {
      PushRod_SetTarget(&push_rod_b, push_rod_b.min_ecd);
    }
    PushRod_Calc(&push_rod_b);
  }
  else if (push_rods_ctrl_data.state_f == PUSH_ROD_STATE_CALIBRATING) {
    PushRod_Calibrate(&push_rod_b);
  }
  else {
    PushRod_Disable(&push_rod_b);
  }
}

void Chassis_Logic() {
  if (chassis_ctrl_data.state == CHASSIS_STATE_ENABLE) {
    Chassis_Enable(&chassis);
    Chassis_Calc(&chassis, chassis_ctrl_data.spd_x, chassis_ctrl_data.spd_y, chassis_ctrl_data.spd_z);
  }
  else {
    Chassis_Disable(&chassis);
  }
}
