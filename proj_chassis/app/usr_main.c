/*
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2026-02-24 12:46:53
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-04-07 20:24:44
 * @FilePath: \proj_chassis\app\usr_main.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "usr_main.h"

#include "bsp_can.h"
#include "DJI_Motor.h"
#include "referee.h"
#include "remote_receive.h"

#include "mecnum_chassis.h"
#include "push_rod.h"

Chassis_t chassis;    // 底盘结构体
PushRod_t push_rod_f; // 前推杆结构体
PushRod_t push_rod_b; // 后推杆结构体

CAN_Custom_ChassisCtrlData_t chassis_ctrl_data;   // 底盘控制数据

CAN_Custom_PushRodsCtrlData_t push_rods_ctrl_data = {
  .state_f = PUSH_ROD_STATE_DISABLE,
  .state_b = PUSH_ROD_STATE_DISABLE,
  .pos_f = 0,
  .pos_b = 0,
}; // 推杆控制数据

// 左臂控制数据
CAN_Custom_ArmCtrlData_t arm_l_ctrl_data = {
  .state = ARM_STATE_DISABLE,
};
CAN_Custom_Tx_t comm_arm_l_ctrl;

// 左臂反馈数据
CAN_Custom_ArmFdbTarget_t arm_l_fdb_target;
CAN_Custom_Rx_t comm_arm_l_fdb_target;

// 右臂控制数据
CAN_Custom_ArmCtrlData_t arm_r_ctrl_data = {
  .state = ARM_STATE_DISABLE,
};
CAN_Custom_Tx_t comm_arm_r_ctrl;

// 右臂反馈数据
CAN_Custom_ArmFdbTarget_t arm_r_fdb_target;
CAN_Custom_Rx_t comm_arm_r_fdb_target;


void Usr_Main_Init(void) {
  BSP_CAN_InitAll();
  DJI_Motor_TxInitAll();
  refereeINIT();
  FSI6_BUS_IDLEHandler_Init();
}
