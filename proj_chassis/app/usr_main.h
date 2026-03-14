/*
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2026-02-24 12:46:39
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-03-13 22:21:46
 * @FilePath: \proj_chassis\app\usr_main.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef USR_MAIN_H
#define USR_MAIN_H

#include "can_custom_comm.h"

extern Chassis_t chassis;    // 底盘结构体
extern PushRod_t push_rod_f; // 前推杆结构体
extern PushRod_t push_rod_b; // 后推杆结构体

extern CAN_Custom_ChassisCtrlData_t chassis_ctrl_data;   // 底盘控制数据
extern CAN_CustomComm_Rx_t comm_chassis_ctrl;      // 自定义can通信底盘控制

extern CAN_Custom_PushRodsCtrlData_t push_rods_ctrl_data; // 推杆控制数据
extern CAN_CustomComm_Rx_t comm_push_rods_ctrl;    // 自定义can通信推杆控制

void Usr_Main_Init(void);

#endif
