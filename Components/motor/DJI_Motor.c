/*
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2025-10-29 12:11:38
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2025-11-05 21:54:19
 * @FilePath: \MDK-ARMd:\RoboMaster\code\NE_RTOS_TEST\Components\motor\DJI_Motor.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "DJI_Motor.h"

#define K_M3508_CRT_CMD_TO_CRT_A 0.001220703125f                        // (20 / 16384)
#define K_M3508_CRT_CMD_TO_TOR_NM 1.9070299446532999164578111946533e-5f // (20 / 16384) * 0.3 * 187 / 3591
#define K_M3508_CRT_NM_TO_CRT_CMD 52437.56149732620320855614973262f     // 1 / K_M3508_CRT_CMD_TO_TOR_NM

// 全局大疆电机发送实例以及使能标志，包含了所有发送端口
static uint8_t sg_can1_tx200_flg = 0;
static uint8_t sg_can1_tx1ff_flg = 0;
static uint8_t sg_can1_tx2ff_flg = 0;
static uint8_t sg_can1_tx1fe_flg = 0;
static uint8_t sg_can1_tx2fe_flg = 0;
static BSP_CAN_TxInstance sg_DJI_motor_can1_tx200_instance = {0};
static BSP_CAN_TxInstance sg_DJI_motor_can1_tx1ff_instance = {0};
static BSP_CAN_TxInstance sg_DJI_motor_can1_tx2ff_instance = {0};
static BSP_CAN_TxInstance sg_DJI_motor_can1_tx1fe_instance = {0};
static BSP_CAN_TxInstance sg_DJI_motor_can1_tx2fe_instance = {0};

static uint8_t sg_can2_tx200_flg = 0;
static uint8_t sg_can2_tx1ff_flg = 0;
static uint8_t sg_can2_tx2ff_flg = 0;
static uint8_t sg_can2_tx1fe_flg = 0;
static uint8_t sg_can2_tx2fe_flg = 0;
static BSP_CAN_TxInstance sg_DJI_motor_can2_tx200_instance = {0};
static BSP_CAN_TxInstance sg_DJI_motor_can2_tx1ff_instance = {0};
static BSP_CAN_TxInstance sg_DJI_motor_can2_tx2ff_instance = {0};
static BSP_CAN_TxInstance sg_DJI_motor_can2_tx1fe_instance = {0};
static BSP_CAN_TxInstance sg_DJI_motor_can2_tx2fe_instance = {0};

void _DJI_Motor_Callback(BSP_CAN_RxInstance *p_rx_instance);

/// @brief 初始化所有大疆电机发送端口
/// @param
void DJI_Motor_TxInitAll(void)
{
  BSP_CAN_Tx_Init(&sg_DJI_motor_can1_tx200_instance, &hcan1, 0x200, CAN_ID_STD, 8, CAN_RTR_DATA);
  BSP_CAN_Tx_Init(&sg_DJI_motor_can1_tx1ff_instance, &hcan1, 0x1ff, CAN_ID_STD, 8, CAN_RTR_DATA);
  BSP_CAN_Tx_Init(&sg_DJI_motor_can1_tx2ff_instance, &hcan1, 0x2ff, CAN_ID_STD, 8, CAN_RTR_DATA);
  BSP_CAN_Tx_Init(&sg_DJI_motor_can1_tx1fe_instance, &hcan1, 0x1fe, CAN_ID_STD, 8, CAN_RTR_DATA);
  BSP_CAN_Tx_Init(&sg_DJI_motor_can1_tx2fe_instance, &hcan1, 0x2fe, CAN_ID_STD, 8, CAN_RTR_DATA);

  BSP_CAN_Tx_Init(&sg_DJI_motor_can2_tx200_instance, &hcan2, 0x200, CAN_ID_STD, 8, CAN_RTR_DATA);
  BSP_CAN_Tx_Init(&sg_DJI_motor_can2_tx1ff_instance, &hcan2, 0x1ff, CAN_ID_STD, 8, CAN_RTR_DATA);
  BSP_CAN_Tx_Init(&sg_DJI_motor_can2_tx2ff_instance, &hcan2, 0x2ff, CAN_ID_STD, 8, CAN_RTR_DATA);
  BSP_CAN_Tx_Init(&sg_DJI_motor_can2_tx1fe_instance, &hcan2, 0x1fe, CAN_ID_STD, 8, CAN_RTR_DATA);
  BSP_CAN_Tx_Init(&sg_DJI_motor_can2_tx2fe_instance, &hcan2, 0x2fe, CAN_ID_STD, 8, CAN_RTR_DATA);
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
        p_motor->_p_can_tx_instance = &sg_DJI_motor_can1_tx200_instance;
        sg_can1_tx200_flg = 1;
      }
      else if (p_hcan == &hcan2)
      {
        p_motor->_p_can_tx_instance = &sg_DJI_motor_can2_tx200_instance;
        sg_can2_tx200_flg = 1;
      }
      break;
    case 0x205:
    case 0x206:
    case 0x207:
    case 0x208:
      p_motor->tx_id = 0x1FF;
      if (p_hcan == &hcan1)
      {
        p_motor->_p_can_tx_instance = &sg_DJI_motor_can1_tx1ff_instance;
        sg_can1_tx1ff_flg = 1;
      }
      else if (p_hcan == &hcan2)
      {
        p_motor->_p_can_tx_instance = &sg_DJI_motor_can2_tx1ff_instance;
        sg_can2_tx1ff_flg = 1;
      }
      break;
    default:
      while (1)
      {
      }
      break;
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
        if (gm6020_ctrl_mode == GM6020_U_CTRL) {
          p_motor->tx_id = 0x1FF;
          p_motor->_p_can_tx_instance = &sg_DJI_motor_can1_tx1ff_instance;
          sg_can1_tx1ff_flg = 1;
        } else {
          p_motor->tx_id = 0x1FE;
          p_motor->_p_can_tx_instance = &sg_DJI_motor_can1_tx1fe_instance;
          sg_can1_tx1fe_flg = 1;
        }
      }
      else if (p_hcan == &hcan2)
      {
        if (gm6020_ctrl_mode == GM6020_U_CTRL) {
          p_motor->tx_id = 0x1FF;
          p_motor->_p_can_tx_instance = &sg_DJI_motor_can2_tx1ff_instance;
          sg_can2_tx1ff_flg = 1;
        } else {
          p_motor->tx_id = 0x1FE;
          p_motor->_p_can_tx_instance = &sg_DJI_motor_can2_tx1fe_instance;
          sg_can2_tx1fe_flg = 1;
        }
      }
      break;
    case 0x209:
    case 0x20A:
    case 0x20B:
      if (p_hcan == &hcan1)
      {
        if (gm6020_ctrl_mode == GM6020_U_CTRL) {
          p_motor->tx_id = 0x2FF;
          p_motor->_p_can_tx_instance = &sg_DJI_motor_can1_tx2ff_instance;
          sg_can1_tx2ff_flg = 1;
        } else {
          p_motor->tx_id = 0x2FE;
          p_motor->_p_can_tx_instance = &sg_DJI_motor_can1_tx2fe_instance;
          sg_can1_tx2fe_flg = 1;
        }
      }
      else if (p_hcan == &hcan2)
      {
        if (gm6020_ctrl_mode == GM6020_U_CTRL) {
          p_motor->tx_id = 0x2FF;
          p_motor->_p_can_tx_instance = &sg_DJI_motor_can2_tx2ff_instance;
          sg_can2_tx2ff_flg = 1;
        } else {
          p_motor->tx_id = 0x2FE;
          p_motor->_p_can_tx_instance = &sg_DJI_motor_can2_tx2fe_instance;
          sg_can2_tx2fe_flg = 1;
        }
      }
      break;
    default:
      while (1)
      {
      }
      break;
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

  BSP_CAN_RxRegister(&p_motor->can_rx_instance, p_hcan, rx_id, p_motor, _DJI_Motor_Callback);
}

int16_t _DJI_Motor_GetCtrlCmd(DJI_Motor_t *p_motor)
{
  if (p_motor == NULL)
  {
    return 0;
  }

  if (p_motor->common_data.ctrl_data.set_state == MOTOR_CTRL_DISABLE) {
    return 0;
  }

  int16_t temp = p_motor->common_data.final_out;
  if (p_motor->type == DJI_MOTOR_TYPE_M3508 || p_motor->type == DJI_MOTOR_TYPE_M2006 ||
      (p_motor->type == DJI_MOTOR_TYPE_GM6020 && p_motor->GM6020_ctrl_mode == GM6020_I_CTRL))
  {
    if (temp > 16384)
    {
      temp = 16384;
    }
    else if (temp < -16384)
    {
      temp = -16384;
    }
  }
  else if (p_motor->type == DJI_MOTOR_TYPE_GM6020 && p_motor->GM6020_ctrl_mode == GM6020_U_CTRL)
  {
    if (temp > 25000)
    {
      temp = 25000;
    }
    else if (temp < -25000)
    {
      temp = -25000;
    }
  }

  return temp;
}

void DJI_Motor_UpdateTxBuf(DJI_Motor_t *p_motor)
{
  if (p_motor == NULL)
  {
    return;
  }

  // 由rxID判断发送数据位
  // 比如0x201, temp = ((0x201 - 0x200 - 1) % 4) * 2 = 0
  // 发送数据位就是0和1
  // 0x202, temp = ((0x202 - 0x200 - 1) % 4) * 2 = 2
  // 发送数据位就是2和3
  uint8_t temp = ((p_motor->rx_id - 0x200 - 1) % 4) * 2;
  int16_t ctrl_cmd = _DJI_Motor_GetCtrlCmd(p_motor);
  p_motor->_p_can_tx_instance->tx_buff[temp] = ctrl_cmd >> 8;
  p_motor->_p_can_tx_instance->tx_buff[temp + 1] = ctrl_cmd;
}

void DJI_Motor_SendGrouping(CAN_HandleTypeDef *p_hcan, uint32_t tx_id)
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
    if (tx_id == 0x200 && sg_can1_tx200_flg == 1)
    {
      BSP_CAN_Transmit(&sg_DJI_motor_can1_tx200_instance);
    }
    else if (tx_id == 0x1FF && sg_can1_tx1ff_flg == 1)
    {
      BSP_CAN_Transmit(&sg_DJI_motor_can1_tx1ff_instance);
    }
    else if (tx_id == 0x2FF && sg_can1_tx2ff_flg == 1)
    {
      BSP_CAN_Transmit(&sg_DJI_motor_can1_tx2ff_instance);
    }
    else if (tx_id == 0x1FE && sg_can1_tx1fe_flg == 1)
    {
      BSP_CAN_Transmit(&sg_DJI_motor_can1_tx1fe_instance);
    }
    else if (tx_id == 0x2FE && sg_can1_tx2fe_flg == 1)
    {
      BSP_CAN_Transmit(&sg_DJI_motor_can1_tx2fe_instance);
    }
  }
  else if (p_hcan == &hcan2)
  {
    if (tx_id == 0x200 && sg_can2_tx200_flg == 1)
    {
      BSP_CAN_Transmit(&sg_DJI_motor_can2_tx200_instance);
    }
    else if (tx_id == 0x1FF && sg_can2_tx1ff_flg == 1)
    {
      BSP_CAN_Transmit(&sg_DJI_motor_can2_tx1ff_instance);
    }
    else if (tx_id == 0x2FF && sg_can2_tx2ff_flg == 1)
    {
      BSP_CAN_Transmit(&sg_DJI_motor_can2_tx2ff_instance);
    }
    else if (tx_id == 0x1FE && sg_can2_tx1fe_flg == 1)
    {
      BSP_CAN_Transmit(&sg_DJI_motor_can2_tx1fe_instance);
    }
    else if (tx_id == 0x2FE && sg_can2_tx2fe_flg == 1)
    {
      BSP_CAN_Transmit(&sg_DJI_motor_can2_tx2fe_instance);
    }
  }
}

/// @brief 大疆电机计算累计编码值, 仅模块内使用
/// @param p_motor 电机结构体指针
inline void _DJI_Motor_TotalEcdCalc(DJI_Motor_t *const p_motor)
{
  int16_t delta_ecd = p_motor->measure_data.pos_ecd - p_motor->measure_data.pos_last_ecd;

  // 累计编码过零处理
  if (delta_ecd < -4096)
  {
    delta_ecd += 8192;
  }
  else if (delta_ecd > 4096)
  {
    delta_ecd -= 8192;
  }

  p_motor->measure_data.pos_total_ecd += delta_ecd;
  
  p_motor->measure_data.pos_last_ecd = p_motor->measure_data.pos_ecd;
}

/// @brief 大疆电机解包, 仅模块内使用
/// @param p_motor 电机结构体指针
void _DJI_Motor_UnpackRXD(DJI_Motor_t *p_motor)
{
  p_motor->measure_data.pos_ecd = (p_motor->can_rx_instance.rx_buff[0] << 8) | p_motor->can_rx_instance.rx_buff[1];
  p_motor->measure_data.spd_rpm = (p_motor->can_rx_instance.rx_buff[2] << 8) | p_motor->can_rx_instance.rx_buff[3];
  p_motor->measure_data.tor_crt_cmd = (p_motor->can_rx_instance.rx_buff[4] << 8) | p_motor->can_rx_instance.rx_buff[5];

  // 2006没有温度反馈
  if (p_motor->type != DJI_MOTOR_TYPE_M2006)
  {
    p_motor->measure_data.tempreture = p_motor->can_rx_instance.rx_buff[6];
  }

  _DJI_Motor_TotalEcdCalc(p_motor);
}

/// @brief 将电机原始数据转换成Common数据, 仅模块内使用
/// @param p_motor 电机指针
void _DJI_Motor_Data2Common(DJI_Motor_t *const p_motor)
{
  p_motor->common_data.measure_data.total_pos_ecd = p_motor->measure_data.pos_total_ecd;
  p_motor->common_data.measure_data.pos_ecd = p_motor->measure_data.pos_ecd;
  p_motor->common_data.measure_data.spd_rpm = p_motor->measure_data.spd_rpm;
  p_motor->common_data.measure_data.tor = p_motor->measure_data.tor_crt_cmd; // 在大疆电机中，tor为电流编码值
  p_motor->common_data.measure_data.tempreture = p_motor->measure_data.tempreture;
}

/// @brief 大疆电机回调函数, 仅模块内使用
/// @param p_rx_instance
void _DJI_Motor_Callback(BSP_CAN_RxInstance *p_rx_instance)
{
  _DJI_Motor_UnpackRXD((DJI_Motor_t *)p_rx_instance->p_owner_moudle);
  _DJI_Motor_Data2Common((DJI_Motor_t *)p_rx_instance->p_owner_moudle);

  if (((DJI_Motor_t *)p_rx_instance->p_owner_moudle)->MotorRxCallback != NULL)
  {
    ((DJI_Motor_t *)p_rx_instance->p_owner_moudle)->MotorRxCallback((DJI_Motor_t *)p_rx_instance->p_owner_moudle);
  }
}
