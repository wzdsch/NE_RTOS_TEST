/*
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2026-03-11 21:35:10
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-03-13 21:41:18
 * @FilePath: \proj_left_arm\app\usr_main.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "usr_main.h"

#include "bsp_can.h"
#include "DJI_Motor.h"
#include "remote_receive.h"
#include "referee.h"

Arm_t arm;

// 此板子控制的臂 -> arm
CAN_Custom_ArmCtrlData_t arm1_ctrl_data;

// 另一个板子控制的臂
CAN_CustomComm_Tx_t comm_arm2_ctrl;
CAN_Custom_ArmCtrlData_t arm2_ctrl_data;

// 底盘
CAN_CustomComm_Tx_t comm_chassis_ctrl;
CAN_Custom_ChassisCtrlData_t chassis_ctrl_data;

// 推杆
CAN_CustomComm_Tx_t comm_push_rods_ctrl;
CAN_Custom_PushRodsCtrlData_t push_rods_ctrl_data;

// 另一个臂的反馈target
CAN_CustomComm_Rx_t comm_arm2_fdb_target;
ArmTarget_t arm2_fdb_target;

void Usr_Main_Init(void) {
  BSP_CAN_InitAll();
  DJI_Motor_TxInitAll();
  FSI6_BUS_IDLEHandler_Init();
  refereeINIT();

}

