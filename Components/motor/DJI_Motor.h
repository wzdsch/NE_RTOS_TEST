/*
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2025-10-29 12:12:02
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2025-12-30 16:14:46
 * @FilePath: \MDK-ARMd:\RoboMaster\code\NE_RTOS_TEST\Components\motor\DJI_Motor.h
 * @Description: 大疆电机驱动模块, 初始化参数相关问题, 会卡死在while(1)中, 此模块只负责数据交互和失能/失能逻辑
 */
#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#include "bsp_can.h"

typedef enum
{
  DJI_MOTOR_STATE_DISABLE,
  DJI_MOTOR_STATE_ENABLE
} DJI_Motor_State_e;

typedef enum
{
  DJI_MOTOR_TYPE_M3508,
  DJI_MOTOR_TYPE_M2006,
  DJI_MOTOR_TYPE_GM6020
} DJI_Motor_Type_e;

typedef enum
{
  DJI_MOTOR_TX_200,
  DJI_MOTOR_TX_1FF,
  DJI_MOTOR_TX_2FF,
  DJI_MOTOR_TX_1FE,
  DJI_MOTOR_TX_2FE
} DJI_Motor_TxID_e;

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

  DJI_Motor_Measure_t measure; // 电机测量数据

  int16_t set_cmd; // 电机设定编码值: 电流(-16384 ~ 16384) / 电压(-25000 ~ 25000, 6020电压控制模式)

  void (*MotorRxCallback)(struct _DJI_Motor_t *); // 电机接收回调函数

  void *p_owner_moudle; // 电机所属模块(云台、底盘等), 用于前馈等函数
} DJI_Motor_t;

typedef struct {
  CAN_HandleTypeDef *hcan;
  DJI_Motor_TxID_e tx_id;
  uint32_t rx_id;
  DJI_Motor_Type_e type;
  void (*const MotorRxCallback)(DJI_Motor_t *); // 电机接收回调函数
  void *p_owner_moudle; // 电机所属模块(云台、底盘等), 用于前馈等函数
} DJI_Motor_Init_t;

/// @brief 初始化所有电机发送端口
/// @param  
void DJI_Motor_TxInitAll(void);

/// @brief 大疆电机初始化
/// @param p_motor 电机指针
/// @param init 初始化结构体
void DJI_Motor_Init(DJI_Motor_t *const p_motor, const DJI_Motor_Init_t *const init);

/// @brief 大疆电机发送端口 更新并发送数据
/// @param p_hcan can句柄
/// @param tx_id 发送端口id
void DJI_Motor_Transmit(CAN_HandleTypeDef *p_hcan, DJI_Motor_TxID_e tx_id);

/// @brief 设定大疆电机发送控制值
/// @param p_motor 
/// @param cmd 
void DJI_Motor_SetCmd(DJI_Motor_t *p_motor, int16_t cmd);

/// @brief 大疆电机失能
/// @param p_motor 电机指针
void DJI_Motor_Disable(DJI_Motor_t *const p_motor);

/// @brief 大疆电机使能
/// @param p_motor 电机指针
void DJI_Motor_Enable(DJI_Motor_t *const p_motor);
#endif
