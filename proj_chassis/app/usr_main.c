/*
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2026-02-24 12:46:53
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-03-13 17:48:27
 * @FilePath: \proj_chassis\app\usr_main.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "usr_main.h"

#include "bsp_can.h"
#include "DJI_Motor.h"
#include "referee.h"

#include "mecnum_chassis.h"
#include "push_rod.h"
#include "can_custom_comm.h"

Chassis_t chassis;    // 底盘结构体
PushRod_t push_rod_f; // 前推杆结构体
PushRod_t push_rod_b; // 后推杆结构体

CAN_Custom_ChassisCtrlData_t chassis_ctrl_data;   // 底盘控制数据
CAN_CustomComm_Rx_t comm_chassis_ctrl;      // 自定义can通信底盘控制

CAN_Custom_PushRodsCtrlData_t push_rods_ctrl_data = {
  .state_f = PUSH_ROD_STATE_DISABLE,
  .state_b = PUSH_ROD_STATE_DISABLE,
  .pos_f = 0,
  .pos_b = 0,
}; // 推杆控制数据
CAN_CustomComm_Rx_t comm_push_rods_ctrl;    // 自定义can通信推杆控制

void Usr_Main_Init(void) {
  BSP_CAN_InitAll();
  DJI_Motor_TxInitAll();
  refereeINIT();
}

