/*
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2025-10-29 12:12:28
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2025-12-02 20:36:09
 * @FilePath: \MDK-ARMd:\RoboMaster\code\NE_RTOS_TEST\Components\motor\DM_Motor.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "dm_motor.h"
#include "tools.h"
#include "string.h"

uint8_t DM_MOTOR_CLEAR_ERR_TXD[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};
uint8_t DM_MOTOR_ENABLE_TXD[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
uint8_t DM_MOTOR_DISABLE_TXD[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
uint8_t DM_MOTOR_SAVE_ZERO_POS_TXD[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};

void _DM_Motor_RxCallback(BSP_CAN_RxInstance *p_rx_instance);

void DM_Motor_Init(DM_Motor_t *const p_motor, CAN_HandleTypeDef *hcan, const uint8_t id, const uint32_t mst_id,
                   float PMAX, float VMAX, float TMAX, void (*MotorRxCallback)(struct _DM_Motor_t *motor))
{
  if (p_motor == NULL || hcan == NULL)
  {
    while (1)
    {
    }
  }

  p_motor->id = id;
  p_motor->mst_id = mst_id;
  p_motor->hcan = hcan;
  p_motor->MotorRxCallback = MotorRxCallback;
  p_motor->PMAX = PMAX;
  p_motor->VMAX = VMAX;
  p_motor->TMAX = TMAX;

  p_motor->state = DM_MOTOR_STATE_ENABLE;

  BSP_CAN_RxRegister(&p_motor->rx_instance, hcan, mst_id, p_motor, _DM_Motor_RxCallback);

  memset(&p_motor->measure, 0, sizeof(p_motor->measure));

#if DM_TX_USE_MIT_MODE == 1
  BSP_CAN_Tx_Init(&p_motor->mit_tx_instance, hcan, p_motor->mit_mode_data.txd, id, CAN_ID_STD, 0x8, CAN_RTR_DATA);
  memset(&p_motor->mit_mode_data, 0, sizeof(p_motor->mit_mode_data));
#endif

#if DM_TX_USE_POS_SPD_MODE == 1
  BSP_CAN_Tx_Init(&p_motor->pos_spd_tx_instance, hcan, p_motor->pos_spd_mode_data_u.txd, id + DM_POS_SPD_MODE_ID_BASE, CAN_ID_STD, 0x8, CAN_RTR_DATA);
  memset(&p_motor->pos_spd_mode_data_u, 0, sizeof(p_motor->pos_spd_mode_data_u));
#endif

#if DM_TX_USE_SPD_MODE == 1
  BSP_CAN_Tx_Init(&p_motor->spd_tx_instance, hcan, p_motor->spd_mode_data_u.txd, id + DM_SPD_MODE_ID_BASE, CAN_ID_STD, 0x4, CAN_RTR_DATA);
  memset(&p_motor->spd_mode_data_u, 0, sizeof(p_motor->spd_mode_data_u));
#endif
  memset(&p_motor->ctrl, 0, sizeof(p_motor->ctrl));
}

// ----------------------------- 四个特殊控制帧 -----------------------------

void DM_Motor_ClearErr(DM_Motor_t *const p_motor)
{
  CAN_TxHeaderTypeDef tx_header;
  tx_header.StdId = p_motor->id;
  tx_header.ExtId = 0x00;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.IDE = CAN_ID_STD;
  tx_header.DLC = 8;
  tx_header.TransmitGlobalTime = DISABLE;
  while (HAL_CAN_AddTxMessage(p_motor->hcan, &tx_header, DM_MOTOR_CLEAR_ERR_TXD, NULL) != HAL_OK)
    ;
}

void DM_Motor_Enable(DM_Motor_t *const p_motor)
{
  p_motor->state = DM_MOTOR_STATE_ENABLE;
  CAN_TxHeaderTypeDef tx_header;
  tx_header.StdId = p_motor->id;
  tx_header.ExtId = 0x00;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.IDE = CAN_ID_STD;
  tx_header.DLC = 8;
  tx_header.TransmitGlobalTime = DISABLE;
  while (HAL_CAN_AddTxMessage(p_motor->hcan, &tx_header, DM_MOTOR_ENABLE_TXD, NULL) != HAL_OK)
    ;
}

void DM_Motor_Disable(DM_Motor_t *const p_motor)
{
  p_motor->state = DM_MOTOR_STATE_DISABLE;
  CAN_TxHeaderTypeDef tx_header;
  tx_header.StdId = p_motor->id;
  tx_header.ExtId = 0x00;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.IDE = CAN_ID_STD;
  tx_header.DLC = 8;
  tx_header.TransmitGlobalTime = DISABLE;
  while (HAL_CAN_AddTxMessage(p_motor->hcan, &tx_header, DM_MOTOR_DISABLE_TXD, NULL) != HAL_OK)
    ;
}

void DM_Motor_SaveZeroPos(DM_Motor_t *const p_motor)
{
  CAN_TxHeaderTypeDef tx_header;
  tx_header.StdId = p_motor->id;
  tx_header.ExtId = 0x00;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.IDE = CAN_ID_STD;
  tx_header.DLC = 8;
  tx_header.TransmitGlobalTime = DISABLE;
  while (HAL_CAN_AddTxMessage(p_motor->hcan, &tx_header, DM_MOTOR_SAVE_ZERO_POS_TXD, NULL) != HAL_OK)
    ;
}

// ------------------------------- 接收数据处理 -----------------------------

inline void _DM_Motor_TotalPosCalc(DM_Motor_t *const p_motor)
{
  float delta_pos = p_motor->measure.pos_rad - p_motor->measure.last_pos_rad;
  if (delta_pos > p_motor->PMAX)
  {
    delta_pos -= p_motor->PMAX * 2;
  }
  else if (delta_pos < -p_motor->PMAX)
  {
    delta_pos += p_motor->PMAX * 2;
  }

  p_motor->measure.total_pos_rad += delta_pos;
}

void _DM_Motor_UnpackRXD(DM_Motor_t *const p_motor)
{
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

  _DM_Motor_TotalPosCalc(p_motor);

  p_motor->measure.last_pos_rad = p_motor->measure.pos_rad;
}

void _DM_Motor_RxCallback(BSP_CAN_RxInstance *p_rx_instance)
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
  uint16_t pos_ecd = float_to_uint16(p_motor->mit_mode_data.ctrl_data.pos_rad, -p_motor->PMAX, p_motor->PMAX, 16),    // 16bit
           spd_ecd = float_to_uint16(p_motor->mit_mode_data.ctrl_data.spd_radps, -p_motor->VMAX, p_motor->VMAX, 12),    // 12bit
           kp_ecd = float_to_uint16(p_motor->mit_mode_data.ctrl_data.kp, DM_KP_MIN, DM_KP_MAX, 12),               // 12bit
           kd_ecd = float_to_uint16(p_motor->mit_mode_data.ctrl_data.kd, DM_KD_MIN, DM_KD_MAX, 12),               // 12bit
           torq_ecd = float_to_uint16(p_motor->mit_mode_data.ctrl_data.torq_nm, -p_motor->TMAX, p_motor->TMAX, 12);  // 12bit

  p_motor->mit_mode_data.txd[0] = (pos_ecd >> 8);
  p_motor->mit_mode_data.txd[1] = pos_ecd;
  p_motor->mit_mode_data.txd[2] = (spd_ecd >> 4);
  p_motor->mit_mode_data.txd[3] = (spd_ecd & 0x0F << 4) | (kp_ecd >> 8);
  p_motor->mit_mode_data.txd[4] = kp_ecd;
  p_motor->mit_mode_data.txd[5] = (kd_ecd >> 4);
  p_motor->mit_mode_data.txd[6] = ((kd_ecd & 0x0F) << 4) | (torq_ecd >> 8);
  p_motor->mit_mode_data.txd[7] = torq_ecd;

  BSP_CAN_Transmit(&p_motor->mit_tx_instance);
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
  p_motor->pos_spd_mode_data_u.ctrl_data.pos_rad = pos;
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
  p_motor->pos_spd_mode_data_u.ctrl_data.spd_radps = spd;
}

void DM_Motor_POS_SPD_Send(DM_Motor_t *const p_motor)
{
  BSP_CAN_Transmit(&p_motor->pos_spd_tx_instance);
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
  p_motor->spd_mode_data_u.spd_radps = spd;
}

void DM_Motor_SPD_Send(DM_Motor_t *const p_motor)
{
  BSP_CAN_Transmit(&p_motor->spd_tx_instance);
}
#endif
