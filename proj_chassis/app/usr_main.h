/*
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2026-02-24 12:46:39
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-03-25 12:34:26
 * @FilePath: \proj_chassis\app\usr_main.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef USR_MAIN_H
#define USR_MAIN_H

#include "can_custom.h"

extern Chassis_t chassis;    // 底盘结构体
extern PushRod_t push_rod_f; // 前推杆结构体
extern PushRod_t push_rod_b; // 后推杆结构体

extern CAN_Custom_ChassisCtrlData_t chassis_ctrl_data;   // 底盘控制数据

extern CAN_Custom_PushRodsCtrlData_t push_rods_ctrl_data; // 推杆控制数据

// 左臂控制数据
extern CAN_Custom_ArmCtrlData_t arm_l_ctrl_data;
extern CAN_Custom_Tx_t comm_arm_l_ctrl;

// 左臂反馈数据
extern CAN_Custom_ArmFdbTarget_t arm_l_fdb_target;
extern CAN_Custom_Rx_t comm_arm_l_fdb_target;

// 右臂控制数据
extern CAN_Custom_ArmCtrlData_t arm_r_ctrl_data;
extern CAN_Custom_Tx_t comm_arm_r_ctrl;

// 右臂反馈数据
extern CAN_Custom_ArmFdbTarget_t arm_r_fdb_target;
extern CAN_Custom_Rx_t comm_arm_r_fdb_target;

void Usr_Main_Init(void);

#endif
