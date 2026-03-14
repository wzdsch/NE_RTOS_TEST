#ifndef USR_MAIN_H
#define USR_MAIN_H

#include "can_custom_comm.h"

extern Arm_t arm;

// 此板子控制的臂 -> arm
extern CAN_Custom_ArmCtrlData_t arm1_ctrl_data;

// 另一个板子控制的臂
extern CAN_CustomComm_Tx_t comm_arm2_ctrl;
extern CAN_Custom_ArmCtrlData_t arm2_ctrl_data;

// 底盘
extern CAN_CustomComm_Tx_t comm_chassis_ctrl;
extern CAN_Custom_ChassisCtrlData_t chassis_ctrl_data;

// 推杆
extern CAN_CustomComm_Tx_t comm_push_rods_ctrl;
extern CAN_Custom_PushRodsCtrlData_t push_rods_ctrl_data;

// 另一个臂的反馈target
extern CAN_CustomComm_Rx_t comm_arm2_fdb_target;
extern ArmTarget_t arm2_fdb_target;

void Usr_Main_Init(void);

#endif
