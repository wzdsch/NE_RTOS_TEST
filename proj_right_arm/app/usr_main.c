/*
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2026-03-11 21:53:53
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-03-13 17:32:28
 * @FilePath: \proj_right_arm\app\usr_main.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "usr_main.h"

#include "bsp_can.h"
#include "DJI_Motor.h"
#include "can_custom_comm.h"

CAN_Custom_ArmCtrlData_t arm_ctrl_data;
CAN_CustomComm_Rx_t comm_arm_ctrl;

// 反馈给主控板(左臂板)的目标位置
CAN_CustomComm_Tx_t comm_arm_fdb_target;
ArmTarget_t arm_fdb_target;

Arm_t arm;

void Usr_Main_Init(void) {
  BSP_CAN_InitAll();
  DJI_Motor_TxInitAll();
}

