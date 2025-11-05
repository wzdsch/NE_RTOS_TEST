/*
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2025-10-29 12:12:02
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2025-11-05 19:45:06
 * @FilePath: \MDK-ARMd:\RoboMaster\code\NE_RTOS_TEST\Components\motor\DJI_Motor.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#include "motor_common.h"
#include "bsp_can.h"

/// @brief 大疆电机类型枚举
typedef enum
{
  DJI_MOTOR_TYPE_M3508,
  DJI_MOTOR_TYPE_M2006,
  DJI_MOTOR_TYPE_GM6020
} DJI_Motor_Type_e;

/// @brief 6020电机控制模式枚举
typedef enum
{
  GM6020_I_CTRL, // 电流控制, 发送值范围: -16384 ~ 16384, 发送端口: 0x1FE/0x2FE
  GM6020_U_CTRL  // 电压控制, 发送值范围: -25000 ~ 25000, 发送端口: 0x1FF/0x2FF
} GM6020_CtrlMode_e;

typedef struct {
  int64_t pos_total_ecd; // 累计编码值, 若要使用此值，需要保证电机反馈频率足够
  uint16_t pos_ecd; // 电机编码器位置 0 ~ 8191
  uint16_t pos_last_ecd; // 上一个编码值，用于计算累计编码值
  int16_t spd_rpm;  // 电机转速
  int16_t tor_crt_cmd; // 转矩电流: 编码值
  uint8_t tempreture; // 温度 单位: °C
} DJI_Motor_Measure_t;

typedef struct _DJI_Motor_t {
  DJI_Motor_Type_e type; // 电机类型

  CAN_HandleTypeDef *p_hcan; // CAN handle
  uint32_t tx_id; // 发送ID
  uint32_t rx_id; // 接收ID

  BSP_CAN_TxInstance *_p_can_tx_instance; // can发送实例, 模块内使用
  BSP_CAN_RxInstance can_rx_instance; // can接收实例

  GM6020_CtrlMode_e GM6020_ctrl_mode; // 6020控制模式, 仅当type为GM6020时有效

  DJI_Motor_Measure_t measure_data; // 电机测量数据

  MotorCommon_t common_data; // 电机通用数据
  void (*MotorRxCallback)(struct _DJI_Motor_t *);
} DJI_Motor_t;

void DJI_Motor_TxInitAll(void);
void DJI_Motor_Init(DJI_Motor_t *const p_motor, CAN_HandleTypeDef *p_hcan, DJI_Motor_Type_e motor_type,
                    const uint32_t rx_id, void (*const callback)(DJI_Motor_t *), GM6020_CtrlMode_e gm6020_ctrl_mode);
void DJI_Motor_UpdateTxBuf(DJI_Motor_t *p_motor);
void DJI_Motor_SendGrouping(CAN_HandleTypeDef *p_hcan, uint32_t tx_id);
#endif
