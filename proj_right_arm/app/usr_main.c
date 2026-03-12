/*
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2026-03-11 21:53:53
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-03-12 20:55:13
 * @FilePath: \proj_right_arm\app\usr_main.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "usr_main.h"

#include "bsp_can.h"
#include "DJI_Motor.h"
#include "can_custom_comm.h"

CAN_Custom_ArmCtrlData_t arm_ctrl_data_rx;
CAN_CustomComm_Rx_t comm_arm_ctrl_rx;

Arm_t arm;

void Usr_Main_Init(void) {
  BSP_CAN_InitAll();
  DJI_Motor_TxInitAll();
  CAN_CustomComm_Rx_Init_t comm_arm_ctrl_rx_init = {
    .hcan = &hcan1,
    .IDE = CAN_ID_STD,
    .p_buf = &arm_ctrl_data_rx,
    .size = sizeof(arm_ctrl_data_rx),
    .pUnpackFunc = NULL
  };
  CAN_CustomComm_Rx_Init(&comm_arm_ctrl_rx, &comm_arm_ctrl_rx_init);
}

