/*
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2026-03-11 21:35:10
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-03-25 12:16:25
 * @FilePath: \proj_left_arm\app\usr_main.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef USR_MAIN_H
#define USR_MAIN_H

#include "can_custom.h"

extern Arm_t arm;

// 此板子控制的臂 -> arm
extern CAN_Custom_ArmCtrlData_t arm_l_ctrl_data;

// 机械臂反馈数据
extern CAN_Custom_Tx_t comm_arm_l_fdb;
extern CAN_Custom_ArmFdbTarget_t arm_l_fdb_data;

// 接收机械臂控制数据
extern CAN_Custom_Rx_t comm_arm_l_ctrl;
extern CAN_Custom_ArmCtrlData_t arm_l_ctrl_data;

void Usr_Main_Init(void);

#endif
