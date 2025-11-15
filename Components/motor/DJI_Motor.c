/*
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2025-10-29 12:11:38
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2025-11-15 11:48:27
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

// 全局大疆电机发送实例以及使能标志，包含了所有发送端口
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

void _DJI_Motor_Callback(BSP_CAN_RxInstance *p_rx_instance);

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

  BSP_CAN_Tx_Init(&sg_dji_motor_can1_tx200_group.p_tx_instance, &hcan1, 0x200, CAN_ID_STD, 8, CAN_RTR_DATA);
  BSP_CAN_Tx_Init(&sg_dji_motor_can1_tx1ff_group.p_tx_instance, &hcan1, 0x1ff, CAN_ID_STD, 8, CAN_RTR_DATA);
  BSP_CAN_Tx_Init(&sg_dji_motor_can1_tx2ff_group.p_tx_instance, &hcan1, 0x2ff, CAN_ID_STD, 8, CAN_RTR_DATA);
  BSP_CAN_Tx_Init(&sg_dji_motor_can1_tx1fe_group.p_tx_instance, &hcan1, 0x1fe, CAN_ID_STD, 8, CAN_RTR_DATA);
  BSP_CAN_Tx_Init(&sg_dji_motor_can1_tx2fe_group.p_tx_instance, &hcan1, 0x2fe, CAN_ID_STD, 8, CAN_RTR_DATA);

  BSP_CAN_Tx_Init(&sg_dji_motor_can2_tx200_group.p_tx_instance, &hcan2, 0x200, CAN_ID_STD, 8, CAN_RTR_DATA);
  BSP_CAN_Tx_Init(&sg_dji_motor_can2_tx1ff_group.p_tx_instance, &hcan2, 0x1ff, CAN_ID_STD, 8, CAN_RTR_DATA);
  BSP_CAN_Tx_Init(&sg_dji_motor_can2_tx2ff_group.p_tx_instance, &hcan2, 0x2ff, CAN_ID_STD, 8, CAN_RTR_DATA);
  BSP_CAN_Tx_Init(&sg_dji_motor_can2_tx1fe_group.p_tx_instance, &hcan2, 0x1fe, CAN_ID_STD, 8, CAN_RTR_DATA);
  BSP_CAN_Tx_Init(&sg_dji_motor_can2_tx2fe_group.p_tx_instance, &hcan2, 0x2fe, CAN_ID_STD, 8, CAN_RTR_DATA);
}

/// @brief 大疆电机初始化
/// @param p_motor 电机指针
/// @param p_hcan can句柄
/// @param motor_type 电机类型
/// @param rx_id 接收id
/// @param callback 回调函数
/// @param gm6020_ctrl_mode 6020控制模式 ( 电压 / 电流 ), 仅当电机是6020时有效
void DJI_Motor_Init(DJI_Motor_t *const p_motor, CAN_HandleTypeDef *p_hcan, DJI_Motor_Type_e motor_type,
                    const uint32_t rx_id, void (*const callback)(DJI_Motor_t *), GM6020_CtrlMode_e gm6020_ctrl_mode)
{
  if (p_hcan == NULL || p_motor == NULL)
  {
    while (1)
    {
    }
  }

  // 通过电机类型及接收id, 确定发送id及使能端口
  if (motor_type == DJI_MOTOR_TYPE_M3508 || motor_type == DJI_MOTOR_TYPE_M2006)
  {
    switch (rx_id)
    {
    case 0x201:
    case 0x202:
    case 0x203:
    case 0x204:
      p_motor->tx_id = 0x200;
      if (p_hcan == &hcan1)
      {
        p_motor->_p_can_tx_instance = &sg_dji_motor_can1_tx200_group.p_tx_instance;
        sg_dji_motor_can1_tx200_group.en = 1;
      }
      else if (p_hcan == &hcan2)
      {
        p_motor->_p_can_tx_instance = &sg_dji_motor_can2_tx200_group.p_tx_instance;
        sg_dji_motor_can2_tx200_group.en = 1;
      }
      break;
    case 0x205:
    case 0x206:
    case 0x207:
    case 0x208:
      p_motor->tx_id = 0x1FF;
      if (p_hcan == &hcan1)
      {
        p_motor->_p_can_tx_instance = &sg_dji_motor_can1_tx1ff_group.p_tx_instance;
        sg_dji_motor_can1_tx1ff_group.en = 1;
      }
      else if (p_hcan == &hcan2)
      {
        p_motor->_p_can_tx_instance = &sg_dji_motor_can2_tx1ff_group.p_tx_instance;
        sg_dji_motor_can2_tx1ff_group.en = 1;
      }
      break;
    default:
      while (1)
      {
      }
      // break; // can not reach here
    }
  }
  else if (motor_type == DJI_MOTOR_TYPE_GM6020)
  {
    switch (rx_id)
    {
    case 0x205:
    case 0x206:
    case 0x207:
    case 0x208:
      if (p_hcan == &hcan1)
      {
        if (gm6020_ctrl_mode == GM6020_U_CTRL)
        {
          p_motor->tx_id = 0x1FF;
          p_motor->_p_can_tx_instance = &sg_dji_motor_can1_tx1ff_group.p_tx_instance;
          sg_dji_motor_can1_tx1ff_group.en = 1;
        }
        else
        {
          p_motor->tx_id = 0x1FE;
          p_motor->_p_can_tx_instance = &sg_dji_motor_can1_tx1fe_group.p_tx_instance;
          sg_dji_motor_can1_tx1fe_group.en = 1;
        }
      }
      else if (p_hcan == &hcan2)
      {
        if (gm6020_ctrl_mode == GM6020_U_CTRL)
        {
          p_motor->tx_id = 0x1FF;
          p_motor->_p_can_tx_instance = &sg_dji_motor_can2_tx1ff_group.p_tx_instance;
          sg_dji_motor_can2_tx1ff_group.en = 1;
        }
        else
        {
          p_motor->tx_id = 0x1FE;
          p_motor->_p_can_tx_instance = &sg_dji_motor_can2_tx1fe_group.p_tx_instance;
          sg_dji_motor_can2_tx1fe_group.en = 1;
        }
      }
      break;
    case 0x209:
    case 0x20A:
    case 0x20B:
      if (p_hcan == &hcan1)
      {
        if (gm6020_ctrl_mode == GM6020_U_CTRL)
        {
          p_motor->tx_id = 0x2FF;
          p_motor->_p_can_tx_instance = &sg_dji_motor_can1_tx2ff_group.p_tx_instance;
          sg_dji_motor_can1_tx2ff_group.en = 1;
        }
        else
        {
          p_motor->tx_id = 0x2FE;
          p_motor->_p_can_tx_instance = &sg_dji_motor_can1_tx2fe_group.p_tx_instance;
          sg_dji_motor_can1_tx2fe_group.en = 1;
        }
      }
      else if (p_hcan == &hcan2)
      {
        if (gm6020_ctrl_mode == GM6020_U_CTRL)
        {
          p_motor->tx_id = 0x2FF;
          p_motor->_p_can_tx_instance = &sg_dji_motor_can2_tx2ff_group.p_tx_instance;
          sg_dji_motor_can2_tx2ff_group.en = 1;
        }
        else
        {
          p_motor->tx_id = 0x2FE;
          p_motor->_p_can_tx_instance = &sg_dji_motor_can2_tx2fe_group.p_tx_instance;
          sg_dji_motor_can2_tx2fe_group.en = 1;
        }
      }
      break;
    default:
      while (1)
      {
      }
      // break; // can not reach here
    }
  }
  else
  {
    while (1)
    {
      // 参数错误
    }
  }

  p_motor->type = motor_type;
  p_motor->p_hcan = p_hcan;
  p_motor->rx_id = rx_id;
  p_motor->MotorRxCallback = callback;
  p_motor->GM6020_ctrl_mode = gm6020_ctrl_mode;

  p_motor->set_cmd = 0;
  p_motor->state = DJI_MOTOR_STATE_DISABLE;

  memset(&p_motor->ctrl, 0, sizeof(p_motor->ctrl));
  memset(&p_motor->measure_data, 0, sizeof(p_motor->measure_data));

  BSP_CAN_RxRegister(&p_motor->can_rx_instance, p_hcan, rx_id, p_motor, _DJI_Motor_Callback);
}

inline void _DJI_Motor_GetCtrlCmd(DJI_Motor_t *p_motor)
{
  if (p_motor->ctrl.state == MOTOR_CTRL_DISABLE)
  {
    p_motor->set_cmd = 0;
    return;
  }
  p_motor->set_cmd = p_motor->ctrl.final_out;
}

void _DJI_Motor_UpdateTxBuf(DJI_Motor_t *p_motor)
{
  _DJI_Motor_GetCtrlCmd(p_motor);
  // 由rxID判断发送数据位
  // 比如0x201, temp = ((0x201 - 0x200 - 1) % 4) * 2 = 0
  // 发送数据位就是0和1
  // 0x202, temp = ((0x202 - 0x200 - 1) % 4) * 2 = 2
  // 发送数据位就是2和3
  uint8_t temp = ((p_motor->rx_id - 0x200 - 1) % 4) * 2;
  p_motor->_p_can_tx_instance->tx_buff[temp] = p_motor->set_cmd >> 8;
  p_motor->_p_can_tx_instance->tx_buff[temp + 1] = p_motor->set_cmd;
}

void _DJI_Motor_UpdateGroupTxBuf(_DJI_Motor_TxGroup_t *p_group)
{
  if (p_group->en == 0)
  {
    return;
  }
  for (int i = 0; i < 4; i++)
  {
    if (p_group->p_motor[i] != NULL)
    {
      _DJI_Motor_UpdateTxBuf(p_group->p_motor[i]);
    }
  }
}

void _DJI_Motor_GroupUpdateSend(_DJI_Motor_TxGroup_t *p_group)
{
  if (p_group->en == 0)
  {
    return;
  }
  _DJI_Motor_UpdateGroupTxBuf(p_group);
  BSP_CAN_Transmit(&p_group->p_tx_instance);
}

void DJI_Motor_Transmit(CAN_HandleTypeDef *p_hcan, uint32_t tx_id)
{
  if (p_hcan == NULL)
  {
    return;
  }
  if (tx_id != 0x200 && tx_id != 0x1FF && tx_id != 0x2FF && tx_id != 0x1FE && tx_id != 0x2FE)
  {
    return;
  }

  if (p_hcan == &hcan1)
  {
    if (tx_id == 0x200)
    {
      _DJI_Motor_GroupUpdateSend(&sg_dji_motor_can1_tx200_group);
    }
    else if (tx_id == 0x1FF)
    {
      _DJI_Motor_GroupUpdateSend(&sg_dji_motor_can1_tx1ff_group);
    }
    else if (tx_id == 0x2FF)
    {
      _DJI_Motor_GroupUpdateSend(&sg_dji_motor_can1_tx2ff_group);
    }
    else if (tx_id == 0x1FE)
    {
      _DJI_Motor_GroupUpdateSend(&sg_dji_motor_can1_tx1fe_group);
    }
    else if (tx_id == 0x2FE)
    {
      _DJI_Motor_GroupUpdateSend(&sg_dji_motor_can1_tx2fe_group);
    }
  }
  else if (p_hcan == &hcan2)
  {
    if (tx_id == 0x200)
    {
      _DJI_Motor_GroupUpdateSend(&sg_dji_motor_can2_tx200_group);
    }
    else if (tx_id == 0x1FF)
    {
      _DJI_Motor_GroupUpdateSend(&sg_dji_motor_can2_tx1ff_group);
    }
    else if (tx_id == 0x2FF)
    {
      _DJI_Motor_GroupUpdateSend(&sg_dji_motor_can2_tx2ff_group);
    }
    else if (tx_id == 0x1FE)
    {
      _DJI_Motor_GroupUpdateSend(&sg_dji_motor_can2_tx1fe_group);
    }
    else if (tx_id == 0x2FE)
    {
      _DJI_Motor_GroupUpdateSend(&sg_dji_motor_can2_tx2fe_group);
    }
  }
}

/// @brief 大疆电机计算累计编码值, 仅模块内使用
/// @param p_motor 电机结构体指针
inline void _DJI_Motor_TotalEcdCalc(DJI_Motor_t *const p_motor)
{
  int16_t delta_ecd = p_motor->measure_data.pos_ecd - p_motor->measure_data.pos_last_ecd;

  // 累计编码过零处理
  if (delta_ecd < -DJI_MOTOR_ECD_CMD_HALF_ROUND)
  {
    delta_ecd += DJI_MOTOR_ECD_CMD_ROUND;
  }
  else if (delta_ecd > DJI_MOTOR_ECD_CMD_HALF_ROUND)
  {
    delta_ecd -= DJI_MOTOR_ECD_CMD_ROUND;
  }

  p_motor->measure_data.pos_total_ecd += delta_ecd;

  p_motor->measure_data.pos_last_ecd = p_motor->measure_data.pos_ecd;
  p_motor->measure_data.pos_total_ecd_f = p_motor->measure_data.pos_total_ecd;
}

/// @brief 大疆电机解包, 仅模块内使用
/// @param p_motor 电机结构体指针
void _DJI_Motor_UnpackRXD(DJI_Motor_t *p_motor)
{
  p_motor->measure_data.pos_ecd = (p_motor->can_rx_instance.rx_buff[0] << 8) | p_motor->can_rx_instance.rx_buff[1];
  p_motor->measure_data.pos_ecd_f = p_motor->measure_data.pos_ecd;

  p_motor->measure_data.spd_rpm = (p_motor->can_rx_instance.rx_buff[2] << 8) | p_motor->can_rx_instance.rx_buff[3];
  p_motor->measure_data.spd_rpm_f = p_motor->measure_data.spd_rpm;

  p_motor->measure_data.tor_crt_cmd = (p_motor->can_rx_instance.rx_buff[4] << 8) | p_motor->can_rx_instance.rx_buff[5];
  p_motor->measure_data.tor_crt_cmd_f = p_motor->measure_data.tor_crt_cmd;

  // 2006没有温度反馈
  if (p_motor->type != DJI_MOTOR_TYPE_M2006)
  {
    p_motor->measure_data.tempreture = p_motor->can_rx_instance.rx_buff[6];
  }

  _DJI_Motor_TotalEcdCalc(p_motor);
}

/// @brief 大疆电机回调函数, 仅模块内使用
/// @param p_rx_instance
void _DJI_Motor_Callback(BSP_CAN_RxInstance *p_rx_instance)
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
  p_motor->ctrl.state = MOTOR_CTRL_DISABLE;
}

void DJI_Motor_Enable(DJI_Motor_t *const p_motor)
{
  p_motor->state = DJI_MOTOR_STATE_ENABLE;
  p_motor->ctrl.state = MOTOR_CTRL_ENABLE;
}
