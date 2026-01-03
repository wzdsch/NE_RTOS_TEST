/*
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2025-10-29 12:11:38
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2025-12-31 10:48:11
 * @FilePath: \MDK-ARMd:\RoboMaster\code\NE_RTOS_TEST\Components\motor\DJI_Motor.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "DJI_Motor.h"
#include <string.h>

#define DJI_MMOTOR_MAX_I_CMD_OUT 16384 // 大疆电机电流编码控制最大值
#define DJI_MMOTOR_MAX_U_CMD_OUT 25000 // 大疆电机电压编码控制最大值 (6020电压控制模式)

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
uint8_t sg_dji_motor_can1_tx200_buff[8] = {0};
static _DJI_Motor_TxGroup_t sg_dji_motor_can1_tx200_group;

uint8_t sg_dji_motor_can1_tx1ff_buff[8] = {0};
static _DJI_Motor_TxGroup_t sg_dji_motor_can1_tx1ff_group;

uint8_t sg_dji_motor_can1_tx2ff_buff[8] = {0};
static _DJI_Motor_TxGroup_t sg_dji_motor_can1_tx2ff_group;

uint8_t sg_dji_motor_can1_tx1fe_buff[8] = {0};
static _DJI_Motor_TxGroup_t sg_dji_motor_can1_tx1fe_group;

uint8_t sg_dji_motor_can1_tx2fe_buff[8] = {0};
static _DJI_Motor_TxGroup_t sg_dji_motor_can1_tx2fe_group;

uint8_t sg_dji_motor_can2_tx200_buff[8] = {0};
static _DJI_Motor_TxGroup_t sg_dji_motor_can2_tx200_group;

uint8_t sg_dji_motor_can2_tx1ff_buff[8] = {0};
static _DJI_Motor_TxGroup_t sg_dji_motor_can2_tx1ff_group;

uint8_t sg_dji_motor_can2_tx2ff_buff[8] = {0};
static _DJI_Motor_TxGroup_t sg_dji_motor_can2_tx2ff_group;

uint8_t sg_dji_motor_can2_tx1fe_buff[8] = {0};
static _DJI_Motor_TxGroup_t sg_dji_motor_can2_tx1fe_group;

uint8_t sg_dji_motor_can2_tx2fe_buff[8] = {0};
static _DJI_Motor_TxGroup_t sg_dji_motor_can2_tx2fe_group;

void _DJI_Motor_RxCallback(BSP_CAN_RxInstance *p_rx_instance);

/// @brief 初始化所有大疆电机发送端口
/// @param
void DJI_Motor_TxInitAll(void)
{
  memset(sg_dji_motor_can1_tx200_buff, 0, 8);
  memset(sg_dji_motor_can1_tx1ff_buff, 0, 8);
  memset(sg_dji_motor_can1_tx2ff_buff, 0, 8);
  memset(sg_dji_motor_can1_tx1fe_buff, 0, 8);
  memset(sg_dji_motor_can1_tx2fe_buff, 0, 8);

  memset(sg_dji_motor_can2_tx200_buff, 0, 8);
  memset(sg_dji_motor_can2_tx1ff_buff, 0, 8);
  memset(sg_dji_motor_can2_tx2ff_buff, 0, 8);
  memset(sg_dji_motor_can2_tx1fe_buff, 0, 8);
  memset(sg_dji_motor_can2_tx2fe_buff, 0, 8);
  

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
                  sg_dji_motor_can1_tx200_buff, 0x200, CAN_ID_STD, 8, CAN_RTR_DATA);
  BSP_CAN_Tx_Init(&sg_dji_motor_can1_tx1ff_group.p_tx_instance, &hcan1, \
                  sg_dji_motor_can1_tx1ff_buff, 0x1ff, CAN_ID_STD, 8, CAN_RTR_DATA);
  BSP_CAN_Tx_Init(&sg_dji_motor_can1_tx2ff_group.p_tx_instance, &hcan1, \
                  sg_dji_motor_can1_tx2ff_buff, 0x2ff, CAN_ID_STD, 8, CAN_RTR_DATA);
  BSP_CAN_Tx_Init(&sg_dji_motor_can1_tx1fe_group.p_tx_instance, &hcan1, \
                  sg_dji_motor_can1_tx1fe_buff, 0x1fe, CAN_ID_STD, 8, CAN_RTR_DATA);
  BSP_CAN_Tx_Init(&sg_dji_motor_can1_tx2fe_group.p_tx_instance, &hcan1, \
                  sg_dji_motor_can1_tx2fe_buff, 0x2fe, CAN_ID_STD, 8, CAN_RTR_DATA);

  BSP_CAN_Tx_Init(&sg_dji_motor_can2_tx200_group.p_tx_instance, &hcan2, \
                  sg_dji_motor_can2_tx200_buff, 0x200, CAN_ID_STD, 8, CAN_RTR_DATA);
  BSP_CAN_Tx_Init(&sg_dji_motor_can2_tx1ff_group.p_tx_instance, &hcan2, \
                  sg_dji_motor_can2_tx1ff_buff, 0x1ff, CAN_ID_STD, 8, CAN_RTR_DATA);
  BSP_CAN_Tx_Init(&sg_dji_motor_can2_tx2ff_group.p_tx_instance, &hcan2, \
                  sg_dji_motor_can2_tx2ff_buff, 0x2ff, CAN_ID_STD, 8, CAN_RTR_DATA);
  BSP_CAN_Tx_Init(&sg_dji_motor_can2_tx1fe_group.p_tx_instance, &hcan2, \
                  sg_dji_motor_can2_tx1fe_buff, 0x1fe, CAN_ID_STD, 8, CAN_RTR_DATA);
  BSP_CAN_Tx_Init(&sg_dji_motor_can2_tx2fe_group.p_tx_instance, &hcan2, \
                  sg_dji_motor_can2_tx2fe_buff, 0x2fe, CAN_ID_STD, 8, CAN_RTR_DATA);
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
  if (init->hcan == NULL || p_motor == NULL)
  {
    while (1)
    {
    }
  }

  p_motor->type = init->type;
  p_motor->p_hcan = init->hcan;
  p_motor->rx_id = init->rx_id;
  p_motor->MotorRxCallback = init->MotorRxCallback;

  p_motor->p_owner_moudle = init->p_owner_moudle;

  p_motor->set_cmd = 0;
  p_motor->state = DJI_MOTOR_STATE_DISABLE;
  memset(&p_motor->measure, 0, sizeof(p_motor->measure));

  switch (init->tx_id) {
    case DJI_MOTOR_TX_200:
      p_motor->tx_id = 0x200;
      if (init->hcan == &hcan1) {
        p_motor->_p_can_tx_instance = &sg_dji_motor_can1_tx200_group.p_tx_instance;
        sg_dji_motor_can1_tx200_group.en = 1;
        sg_dji_motor_can1_tx200_group.p_motor[(init->tx_id - 0x200) % 4 - 1] = p_motor;
      } else if (init->hcan == &hcan2) {
        p_motor->_p_can_tx_instance = &sg_dji_motor_can2_tx200_group.p_tx_instance;
        sg_dji_motor_can2_tx200_group.en = 1;
        sg_dji_motor_can2_tx200_group.p_motor[(init->tx_id - 0x200) % 4 - 1] = p_motor;
      }
      break;
    case DJI_MOTOR_TX_1FF:
      p_motor->tx_id = 0x1FF;
      if (init->hcan == &hcan1) {
        p_motor->_p_can_tx_instance = &sg_dji_motor_can1_tx1ff_group.p_tx_instance;
        sg_dji_motor_can1_tx1ff_group.en = 1;
        sg_dji_motor_can1_tx1ff_group.p_motor[(init->tx_id - 0x1FF) % 4 - 1] = p_motor;
      } else if (init->hcan == &hcan2) {
        p_motor->_p_can_tx_instance = &sg_dji_motor_can2_tx1ff_group.p_tx_instance;
        sg_dji_motor_can2_tx1ff_group.en = 1;
        sg_dji_motor_can2_tx1ff_group.p_motor[(init->tx_id - 0x1FF) % 4 - 1] = p_motor;
      }
      break;
    case DJI_MOTOR_TX_2FF:
      p_motor->tx_id = 0x2FF;
      if (init->hcan == &hcan1) {
        p_motor->_p_can_tx_instance = &sg_dji_motor_can1_tx2ff_group.p_tx_instance;
        sg_dji_motor_can1_tx2ff_group.en = 1;
        sg_dji_motor_can1_tx2ff_group.p_motor[(init->tx_id - 0x2FF) % 4 - 1] = p_motor;
      } else if (init->hcan == &hcan2) {
        p_motor->_p_can_tx_instance = &sg_dji_motor_can2_tx2ff_group.p_tx_instance;
        sg_dji_motor_can2_tx2ff_group.en = 1;
        sg_dji_motor_can2_tx2ff_group.p_motor[(init->tx_id - 0x2FF) % 4 - 1] = p_motor;
      }
      break;
    case DJI_MOTOR_TX_1FE:
      p_motor->tx_id = 0x1FE;
      if (init->hcan == &hcan1) {
        p_motor->_p_can_tx_instance = &sg_dji_motor_can1_tx1fe_group.p_tx_instance;
        sg_dji_motor_can1_tx1fe_group.en = 1;
        sg_dji_motor_can1_tx1fe_group.p_motor[(init->tx_id - 0x1FE) % 4 - 1] = p_motor;
      } else if (init->hcan == &hcan2) {
        p_motor->_p_can_tx_instance = &sg_dji_motor_can2_tx1fe_group.p_tx_instance;
        sg_dji_motor_can2_tx1fe_group.en = 1;
        sg_dji_motor_can2_tx1fe_group.p_motor[(init->tx_id - 0x1FE) % 4 - 1] = p_motor;
      }
      break;
    case DJI_MOTOR_TX_2FE:
      p_motor->tx_id = 0x2FE;
      if (init->hcan == &hcan1) {
        p_motor->_p_can_tx_instance = &sg_dji_motor_can1_tx2fe_group.p_tx_instance;
        sg_dji_motor_can1_tx2fe_group.en = 1;
        sg_dji_motor_can1_tx2fe_group.p_motor[(init->tx_id - 0x2FE) % 4 - 1] = p_motor;
      } else if (init->hcan == &hcan2) {
        p_motor->_p_can_tx_instance = &sg_dji_motor_can2_tx2fe_group.p_tx_instance;
        sg_dji_motor_can2_tx2fe_group.en = 1;
        sg_dji_motor_can2_tx2fe_group.p_motor[(init->tx_id - 0x2FE) % 4 - 1] = p_motor;
      }
      break;
    default: // 参数错误
      while (1)
      {
      }
  }

  BSP_CAN_RxRegister(&p_motor->can_rx_instance, init->hcan, init->rx_id, p_motor, _DJI_Motor_RxCallback);
}

// inline void _DJI_Motor_GetCtrlCmd(DJI_Motor_t *p_motor)
// {
//   if (p_motor->ctrl.state == MOTOR_CTRL_DISABLE)
//   {
//     p_motor->set_cmd = 0;
//     return;
//   }
//   p_motor->set_cmd = p_motor->ctrl.final_out;
// }

void _DJI_Motor_UpdateTxBuf(DJI_Motor_t *p_motor)
{
  // 由rxID判断发送数据位
  // 比如0x201, temp = ((0x201 - 0x200 - 1) % 4) * 2 = 0
  // 发送数据位就是0和1
  // 0x202, temp = ((0x202 - 0x200 - 1) % 4) * 2 = 2
  // 发送数据位就是2和3
  uint8_t temp = ((p_motor->rx_id - 0x200 - 1) % 4) * 2;
  p_motor->_p_can_tx_instance->p_tx_buf[temp] = p_motor->set_cmd >> 8;
  p_motor->_p_can_tx_instance->p_tx_buf[temp + 1] = p_motor->set_cmd;
}

void DJI_Motor_SetCmd(DJI_Motor_t *p_motor, int16_t cmd) {
  p_motor->set_cmd = cmd;
  _DJI_Motor_UpdateTxBuf(p_motor);
}

void _DJI_Motor_GroupUpdateSend(_DJI_Motor_TxGroup_t *p_group)
{
  if (p_group->en == 0)
  {
    return;
  }
  BSP_CAN_Transmit(&p_group->p_tx_instance);
}

void DJI_Motor_Transmit(CAN_HandleTypeDef *p_hcan, DJI_Motor_TxID_e tx_id)
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
    else if (tx_id == 0x1FF)
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
  int16_t delta_ecd = p_motor->measure.pos_ecd - p_motor->measure.pos_last_ecd;

  // 累计编码过零处理
  if (delta_ecd < -DJI_MOTOR_ECD_CMD_HALF_ROUND)
  {
    delta_ecd += DJI_MOTOR_ECD_CMD_ROUND;
  }
  else if (delta_ecd > DJI_MOTOR_ECD_CMD_HALF_ROUND)
  {
    delta_ecd -= DJI_MOTOR_ECD_CMD_ROUND;
  }

  p_motor->measure.pos_total_ecd += delta_ecd;
}

/// @brief 大疆电机解包, 仅模块内使用
/// @param p_motor 电机结构体指针
void _DJI_Motor_UnpackRXD(DJI_Motor_t *p_motor)
{
  p_motor->measure.pos_ecd = (p_motor->can_rx_instance.rx_buff[0] << 8) | p_motor->can_rx_instance.rx_buff[1];
  p_motor->measure.pos_ecd_f = p_motor->measure.pos_ecd;

  p_motor->measure.spd_rpm = (p_motor->can_rx_instance.rx_buff[2] << 8) | p_motor->can_rx_instance.rx_buff[3];
  p_motor->measure.spd_rpm_f = p_motor->measure.spd_rpm;

  p_motor->measure.tor_crt_cmd = (p_motor->can_rx_instance.rx_buff[4] << 8) | p_motor->can_rx_instance.rx_buff[5];
  p_motor->measure.tor_crt_cmd_f = p_motor->measure.tor_crt_cmd;

  // 2006没有温度反馈
  if (p_motor->type != DJI_MOTOR_TYPE_M2006)
  {
    p_motor->measure.tempreture = p_motor->can_rx_instance.rx_buff[6];
  }

  _DJI_Motor_TotalEcdCalc(p_motor);

  p_motor->measure.pos_last_ecd = p_motor->measure.pos_ecd;
  p_motor->measure.pos_total_ecd_f = p_motor->measure.pos_total_ecd;
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
