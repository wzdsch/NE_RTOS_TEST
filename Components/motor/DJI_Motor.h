/*
 * @beforeAnnotation: 
 * Copyright (c) 2026 by 
 * """ The Robomaster team : NEXT-E from Xi'an University of Technology """
 * All Rights Reserved. 
 * 
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2025-10-29 12:12:02
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-01-28 22:55:23
 * @FilePath: \NE_RTOS_TEST\Components\motor\DJI_Motor.h
 * @Description: 
 */
#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#include "bsp_can.h"

/// @brief 大疆电机状态枚举
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

/// @brief 大疆电机发送ID枚举
typedef enum
{
  DJI_MOTOR_TX_200 = 0x200,
  DJI_MOTOR_TX_1FF = 0x1FF,
  DJI_MOTOR_TX_2FF = 0x2FF,
  DJI_MOTOR_TX_1FE = 0x1FE,
  DJI_MOTOR_TX_2FE = 0x2FE
} DJI_Motor_TxID_e;

/// @brief 大疆电机方向枚举
typedef enum {
  DJI_MOTOR_DIR_NORMAL = 0, // 默认方向
  DJI_MOTOR_DIR_REVERSE         // 反向
} DJI_Motor_Dir_e;

/// @brief 大疆电机原始接收数据结构体
typedef struct {
  int16_t pos_ecd;      // 电机原始编码器位置 0 ~ 8191
  int16_t spd_rpm;      // 电机原始转速
  int16_t tor_crt_ecd;  // 原始转矩电流: 编码值
  uint8_t temperature;   // 温度 单位: °C
} DJI_Motor_Measure_t;

/// @brief 大疆电机处理后接收数据结构体
typedef struct {
  int16_t pos_last_ecd; // 上一个编码值，用于计算累计编码值

  int16_t pos_ecd;  // 电机编码: 0 ~ 8191 (处理后)
  float pos_ecd_f;  // 电机编码: 0 ~ 8191 (处理后)

  int64_t pos_total_ecd; // 累计编码值 (处理后) 若要使用此值，需要保证电机反馈频率足够
  float pos_total_ecd_f; // 累计编码值 (处理后) 若要使用此值，需要保证电机反馈频率足够

  int16_t spd_rpm; // 电机转速 单位: rpm (处理后)
  float spd_rpm_f; // 电机转速 单位: rpm (处理后)

  int16_t tor_crt_ecd; // 转矩电流: 编码值 (处理后)
  float tor_crt_ecd_f; // 转矩电流: 编码值 (处理后)
} DJI_Motor_ProcessedMeasure_t;

/// @brief 大疆电机结构体
typedef struct _DJI_Motor_t {
  DJI_Motor_State_e state;  // 这个参数决定电机的 使能 / 失能
  DJI_Motor_Type_e type;    // 电机类型

  CAN_HandleTypeDef *p_hcan; // CAN handle
  uint32_t tx_id; // 发送ID
  uint32_t rx_id; // 接收ID

  DJI_Motor_Dir_e dir; // 电机方向
  int16_t zero_offset; // 电机编码器零点偏移

  BSP_CAN_TxInstance *_p_can_tx_instance; // can发送实例, 模块内使用
  BSP_CAN_RxInstance can_rx_instance;     // can接收实例

  DJI_Motor_Measure_t measure; // 电机原始接收数据
  DJI_Motor_ProcessedMeasure_t processed_measure; // 电机处理后接收数据

  int16_t set_cmd; // 电机设定控制量编码值: 电流(-16384 ~ 16384) / 电压(-25000 ~ 25000, 6020电压控制模式)

  void (*MotorRxCallback)(struct _DJI_Motor_t *); // 电机接收回调函数

  void *p_owner_moudle; // 电机所属模块(云台、底盘等), 用于前馈等函数
} DJI_Motor_t;

/// @brief 大疆电机初始化结构体
typedef struct {
  CAN_HandleTypeDef *hcan;  // can句柄
  DJI_Motor_Type_e type;    // 电机类型
  DJI_Motor_TxID_e tx_id;   // 发送ID
  uint32_t rx_id;           // 接收ID
  DJI_Motor_Dir_e dir;      // 电机方向
  int16_t zero_offset;      // 电机编码器零点偏移
  void (* MotorRxCallback)(DJI_Motor_t *); // 电机接收回调函数
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
void DJI_Motor_GroupTransmit(CAN_HandleTypeDef *p_hcan, DJI_Motor_TxID_e tx_id);

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
