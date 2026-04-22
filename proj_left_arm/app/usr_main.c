/*
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2026-03-11 21:35:10
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-03-25 12:43:02
 * @FilePath: \proj_left_arm\app\usr_main.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "usr_main.h"

#include "bsp_can.h"
#include "DJI_Motor.h"
#include "remote_receive.h"

Arm_t arm;

// 此板子控制的臂 -> arm
CAN_Custom_ArmCtrlData_t arm_l_ctrl_data;

// 发送机械臂target数据
CAN_Custom_Tx_t comm_arm_l_fdb;
CAN_Custom_ArmFdbTarget_t arm_l_fdb_data;

// 接收机械臂控制数据
CAN_Custom_Rx_t comm_arm_l_ctrl;
CAN_Custom_ArmCtrlData_t arm_l_ctrl_data;

void Usr_Main_Init(void) {
  BSP_CAN_InitAll();
  DJI_Motor_TxInitAll();
}
