/*
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2025-10-29 12:12:13
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2025-11-15 14:48:31
 * @FilePath: \MDK-ARMd:\RoboMaster\code\NE_RTOS_TEST\Components\motor\DM_Motor.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef DM_MOTOR_H
#define DM_MOTOR_H

#include "bsp_can.h"
#include "motor_ctrl.h"

typedef enum {
  DM_MOTOR_STATE_DISABLE,
  DM_MOTOR_STATE_ENABLE
} DM_Motor_State_e;

typedef struct {

} DM_Motor_Measure_t;

typedef struct _DM_Motor_t {
  DM_Motor_State_e state;

  uint32_t id; // 上位机中设置的发送id
  uint32_t mst_id; // 上位机中设置的反馈id

  BSP_CAN_TxInstance *tx_instance;
  BSP_CAN_RxInstance *rx_instance;
} DM_Motor_t;

#endif
