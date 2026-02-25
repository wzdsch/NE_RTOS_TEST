/*
 * @beforeAnnotation: 
 * Copyright (c) 2026 by 
 * """ The Robomaster team : NEXT-E from Xi'an University of Technology """
 * All Rights Reserved. 
 * 
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2025-10-29 12:12:28
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-02-05 19:38:45
 * @FilePath: \MDK-ARMd:\RoboMaster\code\NE_RTOS_TEST\Components\motor\DM_Motor.c
 * @Description: 
 */
#include "dm_motor.h"
#include "tools.h"
#include "string.h"

#include "cmsis_os.h"
extern osMessageQueueId_t canTxMsgQueueHandle;

uint8_t DM_MOTOR_CLEAR_ERR_TXD[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};
uint8_t DM_MOTOR_ENABLE_TXD[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
uint8_t DM_MOTOR_DISABLE_TXD[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
uint8_t DM_MOTOR_SAVE_ZERO_POS_TXD[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};

static void _DM_Motor_RxCallback(BSP_CAN_RxInstance *p_rx_instance);

void DM_Motor_Init(DM_Motor_t *const p_motor, DM_Motor_Init_t* init)
{
  if (p_motor == NULL || init == NULL || init->hcan == NULL)
  {
    while (1)
    {
    }
  }

  p_motor->id = init->id;
  p_motor->mst_id = init->mst_id;
  p_motor->hcan = init->hcan;
  p_motor->MotorRxCallback = init->MotorRxCallback;
  p_motor->PMAX = init->PMAX;
  p_motor->VMAX = init->VMAX;
  p_motor->TMAX = init->TMAX;
  p_motor->dir = init->dir;
  p_motor->zero_offset_rad = init->zero_offset_rad;

  p_motor->state = DM_MOTOR_STATE_DISABLE;
  p_motor->p_owner_moudle = init->p_owner_moudle;

  BSP_CAN_RxRegister(&p_motor->rx_instance, p_motor->hcan, p_motor->mst_id, CAN_ID_STD, p_motor, _DM_Motor_RxCallback);

  memset(&p_motor->measure, 0, sizeof(p_motor->measure));
  memset(&p_motor->processed_measure, 0, sizeof(p_motor->processed_measure));

#if DM_TX_USE_MIT_MODE == 1
  BSP_CAN_Tx_Init(&p_motor->mit_tx_instance, p_motor->hcan, p_motor->id, CAN_ID_STD, 0x8, CAN_RTR_DATA);
  memset(&p_motor->mit_mode_data, 0, sizeof(p_motor->mit_mode_data));
#endif

#if DM_TX_USE_POS_SPD_MODE == 1
  BSP_CAN_Tx_Init(&p_motor->pos_spd_tx_instance, p_motor->hcan, p_motor->id + DM_POS_SPD_MODE_ID_BASE, CAN_ID_STD, 0x8, CAN_RTR_DATA);
  memset(&p_motor->pos_spd_mode_data, 0, sizeof(p_motor->pos_spd_mode_data));
#endif

#if DM_TX_USE_SPD_MODE == 1
  BSP_CAN_Tx_Init(&p_motor->spd_tx_instance, p_motor->hcan, p_motor->id + DM_SPD_MODE_ID_BASE, CAN_ID_STD, 0x4, CAN_RTR_DATA);
  memset(&p_motor->spd_mode_data, 0, sizeof(p_motor->spd_mode_data));
#endif
}

// ----------------------------- 四个特殊控制帧 -----------------------------

void DM_Motor_ClearErr(DM_Motor_t *const p_motor)
{
  CAN_TxHeaderTypeDef tx_header = {
    .StdId = p_motor->id,
    .ExtId = 0x00,
    .IDE = CAN_ID_STD,
    .RTR = CAN_RTR_DATA,
    .DLC = 8,
    .TransmitGlobalTime = DISABLE,
  };

  while (HAL_CAN_AddTxMessage(p_motor->hcan, &tx_header, DM_MOTOR_CLEAR_ERR_TXD, NULL) != HAL_OK)
    ;
}

void DM_Motor_Enable(DM_Motor_t *const p_motor)
{
  CAN_TxHeaderTypeDef tx_header = {
    .StdId = p_motor->id,
    .ExtId = 0x00,
    .IDE = CAN_ID_STD,
    .RTR = CAN_RTR_DATA,
    .DLC = 8,
    .TransmitGlobalTime = DISABLE,
  };

  p_motor->state = DM_MOTOR_STATE_ENABLE;
  while (HAL_CAN_AddTxMessage(p_motor->hcan, &tx_header, DM_MOTOR_ENABLE_TXD, NULL) != HAL_OK)
    ;
}

void DM_Motor_Disable(DM_Motor_t *const p_motor)
{
  CAN_TxHeaderTypeDef tx_header = {
    .StdId = p_motor->id,
    .ExtId = 0x00,
    .IDE = CAN_ID_STD,
    .RTR = CAN_RTR_DATA,
    .DLC = 8,
    .TransmitGlobalTime = DISABLE,
  };

  p_motor->state = DM_MOTOR_STATE_DISABLE;
  while (HAL_CAN_AddTxMessage(p_motor->hcan, &tx_header, DM_MOTOR_DISABLE_TXD, NULL) != HAL_OK)
    ;
}

void DM_Motor_SaveZeroPos(DM_Motor_t *const p_motor)
{
  CAN_TxHeaderTypeDef tx_header = {
    .StdId = p_motor->id,
    .ExtId = 0x00,
    .IDE = CAN_ID_STD,
    .RTR = CAN_RTR_DATA,
    .DLC = 8,
    .TransmitGlobalTime = DISABLE,
  };
  
  while (HAL_CAN_AddTxMessage(p_motor->hcan, &tx_header, DM_MOTOR_SAVE_ZERO_POS_TXD, NULL) != HAL_OK)
    ;
}

// ------------------------------- 接收数据处理 -----------------------------

static inline void _DM_Motor_TotalPosCalc(DM_Motor_t *const p_motor)
{
  float delta_pos = p_motor->processed_measure.pos_rad - p_motor->processed_measure.last_pos_rad;

  // 累计位置过零处理
  // DM_Motor的位置范围是[-PMAX, PMAX]，总范围是2*PMAX
  // 当位置变化超过PMAX时，认为发生了过零
  if (delta_pos < -p_motor->PMAX)
  {
    delta_pos += 2 * p_motor->PMAX;
  }
  else if (delta_pos > p_motor->PMAX)
  {
    delta_pos -= 2 * p_motor->PMAX;
  }

  p_motor->processed_measure.total_pos_rad += delta_pos;
}

static void _DM_Motor_UnpackRXD(DM_Motor_t *const p_motor)
{
  // -------------------- 解包原始数据 --------------------
  p_motor->measure.err_code = (DM_Motor_Err_e)((p_motor->rx_instance.rx_buff[0] & 0xF0) >> 4); // 4bit
  p_motor->measure.id = p_motor->rx_instance.rx_buff[0] & 0x0F;                                // 4bit

  p_motor->measure.pos_rad =
      uint16_to_float((p_motor->rx_instance.rx_buff[1] << 8) | p_motor->rx_instance.rx_buff[2],
                      -p_motor->PMAX, p_motor->PMAX, 16); // 16bit

  p_motor->measure.spd_radps =
      uint16_to_float(p_motor->rx_instance.rx_buff[3] << 4 | ((p_motor->rx_instance.rx_buff[4] & 0xF0) >> 4),
                      -p_motor->VMAX, p_motor->VMAX, 12); // 12bit

  p_motor->measure.torq_nm =
      uint16_to_float((p_motor->rx_instance.rx_buff[4] & 0x0F) << 8 | p_motor->rx_instance.rx_buff[5],
                      -p_motor->TMAX, p_motor->TMAX, 12); // 12bit

  p_motor->measure.t_mos = p_motor->rx_instance.rx_buff[6];   // 8bit
  p_motor->measure.t_rotor = p_motor->rx_instance.rx_buff[7]; // 8bit

  // -------------------- 处理数据 --------------------

  // 上一次编码器位置更新
  p_motor->processed_measure.last_pos_rad = p_motor->processed_measure.pos_rad;

  // 软零点处理
  p_motor->processed_measure.pos_rad = p_motor->measure.pos_rad - p_motor->zero_offset_rad;

  // 处理范围 [-PMAX, PMAX]
  if (p_motor->processed_measure.pos_rad > p_motor->PMAX)
  {
    p_motor->processed_measure.pos_rad -= 2 * p_motor->PMAX;
  }
  else if (p_motor->processed_measure.pos_rad < -p_motor->PMAX)
  {
    p_motor->processed_measure.pos_rad += 2 * p_motor->PMAX;
  }

  // 方向反转处理
  if (p_motor->dir == DM_MOTOR_DIR_REVERSE)
  {
    p_motor->processed_measure.pos_rad = -p_motor->processed_measure.pos_rad;
    p_motor->processed_measure.spd_radps = -p_motor->measure.spd_radps;
    p_motor->processed_measure.torq_nm = -p_motor->measure.torq_nm;
  }
  else
  {
    p_motor->processed_measure.spd_radps = p_motor->measure.spd_radps;
    p_motor->processed_measure.torq_nm = p_motor->measure.torq_nm;
  }

  // 累计位置计算
  _DM_Motor_TotalPosCalc(p_motor);
}

static void _DM_Motor_RxCallback(BSP_CAN_RxInstance *p_rx_instance)
{
  _DM_Motor_UnpackRXD(p_rx_instance->p_owner_moudle);

  if (((DM_Motor_t *)(p_rx_instance->p_owner_moudle))->MotorRxCallback != NULL)
  {
    ((DM_Motor_t *)(p_rx_instance->p_owner_moudle))->MotorRxCallback(p_rx_instance->p_owner_moudle);
  }
}

// ------------------------------- MIT -----------------------------
#if DM_TX_USE_MIT_MODE == 1
void DM_Motor_MIT_SetPD(DM_Motor_t *const p_motor, float kp, float kd)
{
  if (kp > DM_KP_MAX)
  {
    kp = DM_KP_MAX;
  }
  else if (kp < DM_KP_MIN)
  {
    kp = DM_KP_MIN;
  }
  p_motor->mit_mode_data.ctrl_data.kp = kp;

  if (kd > DM_KD_MAX)
  {
    kd = DM_KD_MAX;
  }
  else if (kd < DM_KD_MIN)
  {
    kd = DM_KD_MIN;
  }
  p_motor->mit_mode_data.ctrl_data.kd = kd;
}

void DM_Motor_MIT_SetTorq(DM_Motor_t *const p_motor, float torq)
{
  if (torq > p_motor->TMAX)
  {
    torq = p_motor->TMAX;
  }
  else if (torq < -p_motor->TMAX)
  {
    torq = -p_motor->TMAX;
  }
  p_motor->mit_mode_data.ctrl_data.torq_nm = torq;
}

void DM_Motor_MIT_SetPos(DM_Motor_t *const p_motor, float pos)
{
  if (pos > p_motor->PMAX)
  {
    pos = p_motor->PMAX;
  }
  else if (pos < -p_motor->PMAX)
  {
    pos = -p_motor->PMAX;
  }
  p_motor->mit_mode_data.ctrl_data.pos_rad = pos;
}

void DM_Motor_MIT_SetSpd(DM_Motor_t *const p_motor, float spd)
{
  if (spd > p_motor->VMAX)
  {
    spd = p_motor->VMAX;
  }
  else if (spd < -p_motor->VMAX)
  {
    spd = -p_motor->VMAX;
  }
  p_motor->mit_mode_data.ctrl_data.spd_radps = spd;
}

void DM_Motor_MIT_Send(DM_Motor_t *const p_motor)
{
  // 获取控制数据
  float temp_set_pos_rad = p_motor->mit_mode_data.ctrl_data.pos_rad;
  float temp_set_spd_radps = p_motor->mit_mode_data.ctrl_data.spd_radps;
  float temp_set_torq_nm = p_motor->mit_mode_data.ctrl_data.torq_nm;

  // 方向反转处理
  if (p_motor->dir == DM_MOTOR_DIR_REVERSE)
  {
    temp_set_pos_rad = -temp_set_pos_rad;
    temp_set_spd_radps = -temp_set_spd_radps;
    temp_set_torq_nm = -temp_set_torq_nm;
  }

  // 软零点处理
  temp_set_pos_rad = temp_set_pos_rad + p_motor->zero_offset_rad;
  
  // 处理范围 [-PMAX, PMAX]
  if (temp_set_pos_rad > p_motor->PMAX)
  {
    temp_set_pos_rad -= 2 * p_motor->PMAX;
  }
  else if (temp_set_pos_rad < -p_motor->PMAX)
  {
    temp_set_pos_rad += 2 * p_motor->PMAX;
  }

  uint16_t pos_ecd = float_to_uint16(temp_set_pos_rad, -p_motor->PMAX, p_motor->PMAX, 16),    // 16bit
           spd_ecd = float_to_uint16(temp_set_spd_radps, -p_motor->VMAX, p_motor->VMAX, 12),    // 12bit
           kp_ecd = float_to_uint16(p_motor->mit_mode_data.ctrl_data.kp, DM_KP_MIN, DM_KP_MAX, 12),               // 12bit
           kd_ecd = float_to_uint16(p_motor->mit_mode_data.ctrl_data.kd, DM_KD_MIN, DM_KD_MAX, 12),               // 12bit
           torq_ecd = float_to_uint16(temp_set_torq_nm, -p_motor->TMAX, p_motor->TMAX, 12);  // 12bit

  p_motor->mit_mode_data.txd[0] = (pos_ecd >> 8);
  p_motor->mit_mode_data.txd[1] = pos_ecd;
  p_motor->mit_mode_data.txd[2] = (spd_ecd >> 4);
  p_motor->mit_mode_data.txd[3] = ((spd_ecd & 0x0F) << 4) | (kp_ecd >> 8);
  p_motor->mit_mode_data.txd[4] = kp_ecd;
  p_motor->mit_mode_data.txd[5] = (kd_ecd >> 4);
  p_motor->mit_mode_data.txd[6] = ((kd_ecd & 0x0F) << 4) | (torq_ecd >> 8);
  p_motor->mit_mode_data.txd[7] = torq_ecd;

  BSP_CAN_SetTxBuf(&p_motor->mit_tx_instance, p_motor->mit_mode_data.txd);

  osMessageQueuePut(canTxMsgQueueHandle, &p_motor->mit_tx_instance, 0, 0);
  // BSP_CAN_Transmit(&p_motor->mit_tx_instance);
}
#endif

// --------------------------------- POS_SPD -----------------------------
#if DM_TX_USE_POS_SPD_MODE == 1
void DM_Motor_POS_SPD_SetPos(DM_Motor_t *const p_motor, float pos)
{
  if (pos > p_motor->PMAX)
  {
    pos = p_motor->PMAX;
  }
  else if (pos < -p_motor->PMAX)
  {
    pos = -p_motor->PMAX;
  }
  p_motor->pos_spd_mode_data.ctrl_data.pos_rad = pos;
}

void DM_Motor_POS_SPD_SetSpd(DM_Motor_t *const p_motor, float spd)
{
  if (spd > p_motor->VMAX)
  {
    spd = p_motor->VMAX;
  }
  else if (spd < -p_motor->VMAX)
  {
    spd = -p_motor->VMAX;
  }
  p_motor->pos_spd_mode_data.ctrl_data.spd_radps = spd;
}

void DM_Motor_POS_SPD_Send(DM_Motor_t *const p_motor)
{
  
  // 获取控制数据
  float temp_set_pos_rad = p_motor->pos_spd_mode_data.ctrl_data.pos_rad;
  float temp_set_spd_radps = p_motor->pos_spd_mode_data.ctrl_data.spd_radps;

  // 方向反转处理
  if (p_motor->dir == DM_MOTOR_DIR_REVERSE)
  {
    temp_set_pos_rad = -temp_set_pos_rad;
    temp_set_spd_radps = -temp_set_spd_radps;
  }

  // 软零点处理
  temp_set_pos_rad = temp_set_pos_rad + p_motor->zero_offset_rad;
  // 处理范围 [-PMAX, PMAX]
  if (temp_set_pos_rad > p_motor->PMAX)
  {
    temp_set_pos_rad -= 2 * p_motor->PMAX;
  }
  else if (temp_set_pos_rad < -p_motor->PMAX)
  {
    temp_set_pos_rad += 2 * p_motor->PMAX;
  }

  // 更新发送数据
  memcpy(p_motor->pos_spd_mode_data.txd + 4, &temp_set_pos_rad, 4);
  memcpy(p_motor->pos_spd_mode_data.txd, &temp_set_spd_radps, 4);

  BSP_CAN_SetTxBuf(&p_motor->pos_spd_tx_instance, p_motor->pos_spd_mode_data.txd);\

  osMessageQueuePut(canTxMsgQueueHandle, &p_motor->pos_spd_tx_instance, 0, 0);
  // BSP_CAN_Transmit(&p_motor->pos_spd_tx_instance);
}
#endif

// --------------------------------- SPD ----------------------------------
#if DM_TX_USE_SPD_MODE == 1
void DM_Motor_SPD_SetSpd(DM_Motor_t *const p_motor, float spd)
{
  if (spd > p_motor->VMAX)
  {
    spd = p_motor->VMAX;
  }
  else if (spd < -p_motor->VMAX)
  {
    spd = -p_motor->VMAX;
  }
  p_motor->spd_mode_data.spd_radps = spd;
}

void DM_Motor_SPD_Send(DM_Motor_t *const p_motor)
{
  // 获取控制数据
  float temp_set_spd_radps = p_motor->spd_mode_data.spd_radps;

  // 方向反转处理
  if (p_motor->dir == DM_MOTOR_DIR_REVERSE)
  {
    temp_set_spd_radps = -temp_set_spd_radps;
  }

  // 更新发送数据
  memcpy(p_motor->spd_mode_data.txd, &temp_set_spd_radps, 4);

  BSP_CAN_SetTxBuf(&p_motor->spd_tx_instance, p_motor->spd_mode_data.txd);

  osMessageQueuePut(canTxMsgQueueHandle, &p_motor->spd_tx_instance, 0, 0);
  // BSP_CAN_Transmit(&p_motor->spd_tx_instance);
}
#endif
