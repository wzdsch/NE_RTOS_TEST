/*
 * @beforeAnnotation: 
 * Copyright (c) 2026 by 
 * """ The Robomaster team : NEXT-E from Xi'an University of Technology """
 * All Rights Reserved. 
 * 
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2025-10-29 12:11:38
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-03-12 16:39:05
 * @FilePath: \NE_RTOS_TEST\Components\motor\DJI_Motor.c
 * @Description: 
 */
#include "DJI_Motor.h"
#include <string.h>

#include "cmsis_os.h"
extern osMessageQueueId_t canTxMsgQueueHandle;

#define DJI_MMOTOR_MAX_I_CMD 16384 // 大疆电机电流编码控制最大值
#define DJI_MMOTOR_MAX_U_CMD 25000 // 大疆电机电压编码控制最大值 (6020电压控制模式)

#define DJI_MOTOR_ECD_CMD_ROUND 8192      // 大疆电机一圈编码值
#define DJI_MOTOR_ECD_CMD_HALF_ROUND 4096 // 大疆电机半圈编码值

#define K_M3508_CRT_CMD_TO_CRT_A 0.001220703125f                        // (20 / 16384)
#define K_M3508_CRT_CMD_TO_TOR_NM 1.9070299446532999164578111946533e-5f // (20 / 16384) * 0.3 * 187 / 3591
#define K_M3508_CRT_NM_TO_CRT_CMD 52437.56149732620320855614973262f     // 1 / K_M3508_CRT_CMD_TO_TOR_NM

typedef struct
{
  uint8_t en; // 1为使能，0为关闭
  BSP_CAN_TxInstance p_tx_instance;
  DJI_Motor_t *p_motor[4]; // 属于本发送端口的电机指针
} _DJI_Motor_TxGroup_t;

// 全局大疆电机发送端口及数据
static _DJI_Motor_TxGroup_t sg_dji_motor_can1_tx200_group;
static _DJI_Motor_TxGroup_t sg_dji_motor_can1_tx1ff_group;
static _DJI_Motor_TxGroup_t sg_dji_motor_can1_tx2ff_group;
static _DJI_Motor_TxGroup_t sg_dji_motor_can1_tx1fe_group;
static _DJI_Motor_TxGroup_t sg_dji_motor_can1_tx2fe_group;
static _DJI_Motor_TxGroup_t sg_dji_motor_can2_tx200_group;
static _DJI_Motor_TxGroup_t sg_dji_motor_can2_tx1ff_group;
static _DJI_Motor_TxGroup_t sg_dji_motor_can2_tx2ff_group;
static _DJI_Motor_TxGroup_t sg_dji_motor_can2_tx1fe_group;
static _DJI_Motor_TxGroup_t sg_dji_motor_can2_tx2fe_group;

void _DJI_Motor_RxCallback(BSP_CAN_RxInstance *p_rx_instance);

/// @brief 初始化所有大疆电机发送端口
/// @param
void DJI_Motor_TxInitAll(void)
{
  sg_dji_motor_can1_tx200_group.en = 0;
  sg_dji_motor_can1_tx1ff_group.en = 0;
  sg_dji_motor_can1_tx2ff_group.en = 0;
  sg_dji_motor_can1_tx1fe_group.en = 0;
  sg_dji_motor_can1_tx2fe_group.en = 0;

  sg_dji_motor_can2_tx200_group.en = 0;
  sg_dji_motor_can2_tx1ff_group.en = 0;
  sg_dji_motor_can2_tx2ff_group.en = 0;
  sg_dji_motor_can2_tx1fe_group.en = 0;
  sg_dji_motor_can2_tx2fe_group.en = 0;

  sg_dji_motor_can1_tx200_group.p_motor[0] = NULL;
  sg_dji_motor_can1_tx200_group.p_motor[1] = NULL;
  sg_dji_motor_can1_tx200_group.p_motor[2] = NULL;
  sg_dji_motor_can1_tx200_group.p_motor[3] = NULL;

  sg_dji_motor_can1_tx1ff_group.p_motor[0] = NULL;
  sg_dji_motor_can1_tx1ff_group.p_motor[1] = NULL;
  sg_dji_motor_can1_tx1ff_group.p_motor[2] = NULL;
  sg_dji_motor_can1_tx1ff_group.p_motor[3] = NULL;

  sg_dji_motor_can1_tx2ff_group.p_motor[0] = NULL;
  sg_dji_motor_can1_tx2ff_group.p_motor[1] = NULL;
  sg_dji_motor_can1_tx2ff_group.p_motor[2] = NULL;
  sg_dji_motor_can1_tx2ff_group.p_motor[3] = NULL;

  sg_dji_motor_can1_tx1fe_group.p_motor[0] = NULL;
  sg_dji_motor_can1_tx1fe_group.p_motor[1] = NULL;
  sg_dji_motor_can1_tx1fe_group.p_motor[2] = NULL;
  sg_dji_motor_can1_tx1fe_group.p_motor[3] = NULL;

  sg_dji_motor_can1_tx2fe_group.p_motor[0] = NULL;
  sg_dji_motor_can1_tx2fe_group.p_motor[1] = NULL;
  sg_dji_motor_can1_tx2fe_group.p_motor[2] = NULL;
  sg_dji_motor_can1_tx2fe_group.p_motor[3] = NULL;

  sg_dji_motor_can2_tx200_group.p_motor[0] = NULL;
  sg_dji_motor_can2_tx200_group.p_motor[1] = NULL;
  sg_dji_motor_can2_tx200_group.p_motor[2] = NULL;
  sg_dji_motor_can2_tx200_group.p_motor[3] = NULL;

  sg_dji_motor_can2_tx1ff_group.p_motor[0] = NULL;
  sg_dji_motor_can2_tx1ff_group.p_motor[1] = NULL;
  sg_dji_motor_can2_tx1ff_group.p_motor[2] = NULL;
  sg_dji_motor_can2_tx1ff_group.p_motor[3] = NULL;

  sg_dji_motor_can2_tx2ff_group.p_motor[0] = NULL;
  sg_dji_motor_can2_tx2ff_group.p_motor[1] = NULL;
  sg_dji_motor_can2_tx2ff_group.p_motor[2] = NULL;
  sg_dji_motor_can2_tx2ff_group.p_motor[3] = NULL;

  sg_dji_motor_can2_tx1fe_group.p_motor[0] = NULL;
  sg_dji_motor_can2_tx1fe_group.p_motor[1] = NULL;
  sg_dji_motor_can2_tx1fe_group.p_motor[2] = NULL;
  sg_dji_motor_can2_tx1fe_group.p_motor[3] = NULL;

  sg_dji_motor_can2_tx2fe_group.p_motor[0] = NULL;
  sg_dji_motor_can2_tx2fe_group.p_motor[1] = NULL;
  sg_dji_motor_can2_tx2fe_group.p_motor[2] = NULL;
  sg_dji_motor_can2_tx2fe_group.p_motor[3] = NULL;

  BSP_CAN_Tx_Init(&sg_dji_motor_can1_tx200_group.p_tx_instance, &hcan1,\
                  0x200, CAN_ID_STD, 8, CAN_RTR_DATA);
  BSP_CAN_Tx_Init(&sg_dji_motor_can1_tx1ff_group.p_tx_instance, &hcan1, \
                  0x1ff, CAN_ID_STD, 8, CAN_RTR_DATA);
  BSP_CAN_Tx_Init(&sg_dji_motor_can1_tx2ff_group.p_tx_instance, &hcan1, \
                  0x2ff, CAN_ID_STD, 8, CAN_RTR_DATA);
  BSP_CAN_Tx_Init(&sg_dji_motor_can1_tx1fe_group.p_tx_instance, &hcan1, \
                  0x1fe, CAN_ID_STD, 8, CAN_RTR_DATA);
  BSP_CAN_Tx_Init(&sg_dji_motor_can1_tx2fe_group.p_tx_instance, &hcan1, \
                  0x2fe, CAN_ID_STD, 8, CAN_RTR_DATA);

  BSP_CAN_Tx_Init(&sg_dji_motor_can2_tx200_group.p_tx_instance, &hcan2, \
                  0x200, CAN_ID_STD, 8, CAN_RTR_DATA);
  BSP_CAN_Tx_Init(&sg_dji_motor_can2_tx1ff_group.p_tx_instance, &hcan2, \
                  0x1ff, CAN_ID_STD, 8, CAN_RTR_DATA);
  BSP_CAN_Tx_Init(&sg_dji_motor_can2_tx2ff_group.p_tx_instance, &hcan2, \
                  0x2ff, CAN_ID_STD, 8, CAN_RTR_DATA);
  BSP_CAN_Tx_Init(&sg_dji_motor_can2_tx1fe_group.p_tx_instance, &hcan2, \
                  0x1fe, CAN_ID_STD, 8, CAN_RTR_DATA);
  BSP_CAN_Tx_Init(&sg_dji_motor_can2_tx2fe_group.p_tx_instance, &hcan2, \
                  0x2fe, CAN_ID_STD, 8, CAN_RTR_DATA);
}

/// @brief 大疆电机初始化
/// @param p_motor 电机指针
/// @param p_hcan can句柄
/// @param motor_type 电机类型
/// @param rx_id 接收id
/// @param callback 回调函数
/// @param gm6020_ctrl_mode 6020控制模式 ( 电压 / 电流 ), 仅当电机是6020时有效
void DJI_Motor_Init(DJI_Motor_t *const p_motor, const DJI_Motor_Init_t *const init)
{
  // 参数检查
  uint8_t param_err = 0;
  if (init == NULL || p_motor == NULL || init->hcan == NULL)
  {
    param_err++;
  }

  if (init->hcan != &hcan1 && init->hcan != &hcan2) {
    param_err++;
  }

  if (init->type != DJI_MOTOR_TYPE_GM6020 && init->type != DJI_MOTOR_TYPE_M3508 && init->type != DJI_MOTOR_TYPE_M2006) {
    param_err++;
  }

  switch (init->tx_id) {
    case DJI_MOTOR_TX_200:
      if (init->rx_id != 0x201 && init->rx_id != 0x202 && init->rx_id != 0x203 && init->rx_id != 0x204) {
        param_err++;
      }
      break;
    case DJI_MOTOR_TX_1FF:
    case DJI_MOTOR_TX_1FE:
      if (init->rx_id != 0x205 && init->rx_id != 0x206 && init->rx_id != 0x207 && init->rx_id != 0x208) {
        param_err++;
      }
      break;
    case DJI_MOTOR_TX_2FF:
    case DJI_MOTOR_TX_2FE:
      if (init->rx_id != 0x209 && init->rx_id != 0x20A && init->rx_id != 0x20B) {
        param_err++;
      }
      break;
    default:
      param_err++;
      break;
  }

  // 参数错误
  if (param_err > 0)
  {
    while(1) {

    }
  }

  p_motor->type = init->type;
  p_motor->p_hcan = init->hcan;
  p_motor->rx_id = init->rx_id;
  p_motor->dir = init->dir;
  p_motor->zero_offset = init->zero_offset;
  p_motor->MotorRxCallback = init->MotorRxCallback;

  p_motor->p_owner_moudle = init->p_owner_moudle;

  p_motor->set_cmd = 0;
  p_motor->state = DJI_MOTOR_STATE_DISABLE;
  memset(&p_motor->measure, 0, sizeof(p_motor->measure));
  memset(&p_motor->processed_measure, 0, sizeof(p_motor->processed_measure));

  // 绑定对应的发送端口并使能
  switch (init->tx_id) {
    case DJI_MOTOR_TX_200:
      p_motor->tx_id = 0x200;
      if (init->hcan == &hcan1) {
        p_motor->_p_can_tx_instance = &sg_dji_motor_can1_tx200_group.p_tx_instance;
        sg_dji_motor_can1_tx200_group.en = 1;
        sg_dji_motor_can1_tx200_group.p_motor[(init->rx_id - 0x200 - 1) % 4] = p_motor;
      } else if (init->hcan == &hcan2) {
        p_motor->_p_can_tx_instance = &sg_dji_motor_can2_tx200_group.p_tx_instance;
        sg_dji_motor_can2_tx200_group.en = 1;
        sg_dji_motor_can2_tx200_group.p_motor[(init->rx_id - 0x200 - 1) % 4] = p_motor;
      }
      break;
    case DJI_MOTOR_TX_1FF:
      p_motor->tx_id = 0x1FF;
      if (init->hcan == &hcan1) {
        p_motor->_p_can_tx_instance = &sg_dji_motor_can1_tx1ff_group.p_tx_instance;
        sg_dji_motor_can1_tx1ff_group.en = 1;
        sg_dji_motor_can1_tx1ff_group.p_motor[(init->rx_id - 0x200 - 1) % 4] = p_motor;
      } else if (init->hcan == &hcan2) {
        p_motor->_p_can_tx_instance = &sg_dji_motor_can2_tx1ff_group.p_tx_instance;
        sg_dji_motor_can2_tx1ff_group.en = 1;
        sg_dji_motor_can2_tx1ff_group.p_motor[(init->rx_id - 0x200 - 1) % 4] = p_motor;
      }
      break;
    case DJI_MOTOR_TX_2FF:
      p_motor->tx_id = 0x2FF;
      if (init->hcan == &hcan1) {
        p_motor->_p_can_tx_instance = &sg_dji_motor_can1_tx2ff_group.p_tx_instance;
        sg_dji_motor_can1_tx2ff_group.en = 1;
        sg_dji_motor_can1_tx2ff_group.p_motor[(init->rx_id - 0x200 - 1) % 4] = p_motor;
      } else if (init->hcan == &hcan2) {
        p_motor->_p_can_tx_instance = &sg_dji_motor_can2_tx2ff_group.p_tx_instance;
        sg_dji_motor_can2_tx2ff_group.en = 1;
        sg_dji_motor_can2_tx2ff_group.p_motor[(init->rx_id - 0x200 - 1) % 4] = p_motor;
      }
      break;
    case DJI_MOTOR_TX_1FE:
      p_motor->tx_id = 0x1FE;
      if (init->hcan == &hcan1) {
        p_motor->_p_can_tx_instance = &sg_dji_motor_can1_tx1fe_group.p_tx_instance;
        sg_dji_motor_can1_tx1fe_group.en = 1;
        sg_dji_motor_can1_tx1fe_group.p_motor[(init->rx_id - 0x200 - 1) % 4 ] = p_motor;
      } else if (init->hcan == &hcan2) {
        p_motor->_p_can_tx_instance = &sg_dji_motor_can2_tx1fe_group.p_tx_instance;
        sg_dji_motor_can2_tx1fe_group.en = 1;
        sg_dji_motor_can2_tx1fe_group.p_motor[(init->rx_id - 0x200 - 1) % 4] = p_motor;
      }
      break;
    case DJI_MOTOR_TX_2FE:
      p_motor->tx_id = 0x2FE;
      if (init->hcan == &hcan1) {
        p_motor->_p_can_tx_instance = &sg_dji_motor_can1_tx2fe_group.p_tx_instance;
        sg_dji_motor_can1_tx2fe_group.en = 1;
        sg_dji_motor_can1_tx2fe_group.p_motor[(init->rx_id - 0x200 - 1) % 4] = p_motor;
      } else if (init->hcan == &hcan2) {
        p_motor->_p_can_tx_instance = &sg_dji_motor_can2_tx2fe_group.p_tx_instance;
        sg_dji_motor_can2_tx2fe_group.en = 1;
        sg_dji_motor_can2_tx2fe_group.p_motor[(init->rx_id - 0x200 - 1) % 4] = p_motor;
      }
      break;
  }

  BSP_CAN_RxRegister(&p_motor->can_rx_instance, init->hcan, init->rx_id, CAN_ID_STD, p_motor, _DJI_Motor_RxCallback);
}

void DJI_Motor_SetCmd(DJI_Motor_t *p_motor, int16_t cmd) {
  p_motor->set_cmd = cmd;
}

void _DJI_Motor_GroupUpdateSend(_DJI_Motor_TxGroup_t *p_group)
{
  if (p_group->en == 0)
  {
    return;
  }
  
  for (uint8_t i = 0; i < 4; i++)
  {
    if (p_group->p_motor[i] != NULL)
    {
      int16_t tmp_set_cmd = 0;
      if (p_group->p_motor[i]->state == DJI_MOTOR_STATE_ENABLE) {
        tmp_set_cmd = p_group->p_motor[i]->set_cmd;
        if (p_group->p_motor[i]->dir == DJI_MOTOR_DIR_REVERSE)
        {
          tmp_set_cmd = -tmp_set_cmd;
        }
      }
      p_group->p_tx_instance.tx_buf[i * 2] = (tmp_set_cmd >> 8) & 0xFF;
      p_group->p_tx_instance.tx_buf[i * 2 + 1] = tmp_set_cmd & 0xFF;
    }
  }

  osMessageQueuePut(canTxMsgQueueHandle, &p_group->p_tx_instance, 0, 0);
  // BSP_CAN_Transmit(&p_group->p_tx_instance);
}

void DJI_Motor_GroupTransmit(CAN_HandleTypeDef *p_hcan, DJI_Motor_TxID_e tx_id)
{
  if (p_hcan == &hcan1)
  {
    if (tx_id == DJI_MOTOR_TX_200)
    {
      _DJI_Motor_GroupUpdateSend(&sg_dji_motor_can1_tx200_group);
    }
    else if (tx_id == DJI_MOTOR_TX_1FF)
    {
      _DJI_Motor_GroupUpdateSend(&sg_dji_motor_can1_tx1ff_group);
    }
    else if (tx_id == DJI_MOTOR_TX_2FF)
    {
      _DJI_Motor_GroupUpdateSend(&sg_dji_motor_can1_tx2ff_group);
    }
    else if (tx_id == DJI_MOTOR_TX_1FE)
    {
      _DJI_Motor_GroupUpdateSend(&sg_dji_motor_can1_tx1fe_group);
    }
    else if (tx_id == DJI_MOTOR_TX_2FE)
    {
      _DJI_Motor_GroupUpdateSend(&sg_dji_motor_can1_tx2fe_group);
    }
  }
  else if (p_hcan == &hcan2)
  {
    if (tx_id == DJI_MOTOR_TX_200)
    {
      _DJI_Motor_GroupUpdateSend(&sg_dji_motor_can2_tx200_group);
    }
    else if (tx_id == DJI_MOTOR_TX_1FF)
    {
      _DJI_Motor_GroupUpdateSend(&sg_dji_motor_can2_tx1ff_group);
    }
    else if (tx_id == DJI_MOTOR_TX_2FF)
    {
      _DJI_Motor_GroupUpdateSend(&sg_dji_motor_can2_tx2ff_group);
    }
    else if (tx_id == DJI_MOTOR_TX_1FE)
    {
      _DJI_Motor_GroupUpdateSend(&sg_dji_motor_can2_tx1fe_group);
    }
    else if (tx_id == DJI_MOTOR_TX_2FE)
    {
      _DJI_Motor_GroupUpdateSend(&sg_dji_motor_can2_tx2fe_group);
    }
  }
}

/// @brief 大疆电机计算累计编码值, 仅模块内使用
/// @param p_motor 电机结构体指针
inline void _DJI_Motor_TotalEcdCalc(DJI_Motor_t *const p_motor)
{
  int16_t delta_ecd = p_motor->processed_measure.pos_ecd - p_motor->processed_measure.pos_last_ecd;

  // 累计编码过零处理
  if (delta_ecd < -DJI_MOTOR_ECD_CMD_HALF_ROUND)
  {
    delta_ecd += DJI_MOTOR_ECD_CMD_ROUND;
  }
  else if (delta_ecd > DJI_MOTOR_ECD_CMD_HALF_ROUND)
  {
    delta_ecd -= DJI_MOTOR_ECD_CMD_ROUND;
  }

  p_motor->processed_measure.pos_total_ecd += delta_ecd;
  p_motor->processed_measure.pos_total_ecd_f = p_motor->processed_measure.pos_total_ecd;
}

/// @brief 大疆电机解包, 仅模块内使用
/// @param p_motor 电机结构体指针
void _DJI_Motor_UnpackRXD(DJI_Motor_t *p_motor)
{
  // -------------------- 解包原始数据 --------------------
  p_motor->measure.pos_ecd = (p_motor->can_rx_instance.rx_buff[0] << 8) | p_motor->can_rx_instance.rx_buff[1];
  p_motor->measure.spd_rpm = (p_motor->can_rx_instance.rx_buff[2] << 8) | p_motor->can_rx_instance.rx_buff[3];
  p_motor->measure.tor_crt_ecd = (p_motor->can_rx_instance.rx_buff[4] << 8) | p_motor->can_rx_instance.rx_buff[5];

  // 2006没有温度反馈
  if (p_motor->type != DJI_MOTOR_TYPE_M2006)
  {
    p_motor->measure.temperature = p_motor->can_rx_instance.rx_buff[6];
  }

  // -------------------- 处理数据 --------------------

  // 上一次编码器位置更新
  p_motor->processed_measure.pos_last_ecd = p_motor->processed_measure.pos_ecd;

  // 软编码计算
  int16_t tmp_ecd = p_motor->measure.pos_ecd - p_motor->zero_offset;

  if (tmp_ecd < 0)
  {
    tmp_ecd += DJI_MOTOR_ECD_CMD_ROUND;
  }
  else if (tmp_ecd >= DJI_MOTOR_ECD_CMD_ROUND)
  {
    tmp_ecd -= DJI_MOTOR_ECD_CMD_ROUND;
  }

  if (p_motor->dir == DJI_MOTOR_DIR_REVERSE) {
    tmp_ecd = DJI_MOTOR_ECD_CMD_ROUND - tmp_ecd;

    p_motor->processed_measure.spd_rpm = -p_motor->measure.spd_rpm;
    p_motor->processed_measure.spd_rpm_f = -p_motor->measure.spd_rpm;

    p_motor->processed_measure.tor_crt_ecd = -p_motor->measure.tor_crt_ecd;
    p_motor->processed_measure.tor_crt_ecd_f = -p_motor->measure.tor_crt_ecd;
  }
  else {
    p_motor->processed_measure.spd_rpm = p_motor->measure.spd_rpm;
    p_motor->processed_measure.spd_rpm_f = p_motor->measure.spd_rpm;

    p_motor->processed_measure.tor_crt_ecd = p_motor->measure.tor_crt_ecd;
    p_motor->processed_measure.tor_crt_ecd_f = p_motor->measure.tor_crt_ecd;
  }
  p_motor->processed_measure.pos_ecd = tmp_ecd;
  p_motor->processed_measure.pos_ecd_f = (float)tmp_ecd;
  _DJI_Motor_TotalEcdCalc(p_motor);
}

/// @brief 大疆电机回调函数, 仅模块内使用
/// @param p_rx_instance
void _DJI_Motor_RxCallback(BSP_CAN_RxInstance *p_rx_instance)
{
  _DJI_Motor_UnpackRXD((DJI_Motor_t *)p_rx_instance->p_owner_moudle);

  if (((DJI_Motor_t *)p_rx_instance->p_owner_moudle)->MotorRxCallback != NULL)
  {
    ((DJI_Motor_t *)p_rx_instance->p_owner_moudle)->MotorRxCallback((DJI_Motor_t *)p_rx_instance->p_owner_moudle);
  }
}

void DJI_Motor_Disable(DJI_Motor_t *const p_motor)
{
  p_motor->state = DJI_MOTOR_STATE_DISABLE;
}

void DJI_Motor_Enable(DJI_Motor_t *const p_motor)
{
  p_motor->state = DJI_MOTOR_STATE_ENABLE;
}
