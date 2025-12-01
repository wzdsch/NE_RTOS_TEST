/*
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2025-10-29 12:12:02
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2025-11-15 11:55:54
 * @FilePath: \MDK-ARMd:\RoboMaster\code\NE_RTOS_TEST\Components\motor\DJI_Motor.h
 * @Description: 大疆电机驱动模块, 初始化参数相关问题, 会卡死在while(1)中, 此模块只负责数据交互和失能/失能逻辑
 */
#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#include "motor_ctrl.h"
#include "bsp_can.h"

typedef enum
{
  DJI_MOTOR_STATE_DISABLE,
  DJI_MOTOR_STATE_ENABLE
} DJI_Motor_State_e;

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
  float pos_total_ecd_f; // 累计编码值的浮点数形式
  
  uint16_t pos_ecd; // 电机编码器位置 0 ~ 8191
  float pos_ecd_f;  // 电机编码器位置的浮点数形式

  uint16_t pos_last_ecd; // 上一个编码值，用于计算累计编码值
  float pos_last_ecd_f; // 上一个编码值的浮点数形式

  int16_t spd_rpm;  // 电机转速
  float spd_rpm_f;  // 电机转速的浮点数形式

  int16_t tor_crt_cmd; // 转矩电流: 编码值
  float tor_crt_cmd_f;  // 转矩电流的浮点数形式

  uint8_t tempreture; // 温度 单位: °C
} DJI_Motor_Measure_t;

typedef struct _DJI_Motor_t {
  DJI_Motor_State_e state; // 这个参数决定电机的 使能 / 失能
  DJI_Motor_Type_e type; // 电机类型

  CAN_HandleTypeDef *p_hcan; // CAN handle
  uint32_t tx_id; // 发送ID
  uint32_t rx_id; // 接收ID

  BSP_CAN_TxInstance *_p_can_tx_instance; // can发送实例, 模块内使用
  BSP_CAN_RxInstance can_rx_instance; // can接收实例

  GM6020_CtrlMode_e GM6020_ctrl_mode; // 6020控制模式, 仅当type为GM6020时有效

  DJI_Motor_Measure_t measure; // 电机测量数据

  MotorCtrl_t ctrl; // 电机控制结构体

  int16_t set_cmd; // 电机设定编码值: 电流(-16384 ~ 16384) / 电压(-25000 ~ 25000, 6020电压控制模式)

  void (*MotorRxCallback)(struct _DJI_Motor_t *);
} DJI_Motor_t;

/// @brief 初始化所有电机发送端口
/// @param  
void DJI_Motor_TxInitAll(void);

/// @brief 大疆电机初始化
/// @param p_motor 电机指针
/// @param p_hcan can句柄
/// @param motor_type 电机类型
/// @param rx_id 接收id
/// @param callback 回调函数指针, 可以为NULL
/// @param gm6020_ctrl_mode 6020电压/电流控制, 此参数只针对6020电机有效
void DJI_Motor_Init(DJI_Motor_t *const p_motor, CAN_HandleTypeDef *p_hcan, DJI_Motor_Type_e motor_type,
                    const uint32_t rx_id, void (*const callback)(DJI_Motor_t *), GM6020_CtrlMode_e gm6020_ctrl_mode);

/// @brief 大疆电机发送端口 更新并发送数据
/// @param p_hcan can句柄
/// @param tx_id 发送端口id
void DJI_Motor_Transmit(CAN_HandleTypeDef *p_hcan, uint32_t tx_id);

/// @brief 大疆电机失能
/// @param p_motor 电机指针
void DJI_Motor_Disable(DJI_Motor_t *const p_motor);

/// @brief 大疆电机使能
/// @param p_motor 电机指针
void DJI_Motor_Enable(DJI_Motor_t *const p_motor);
#endif
