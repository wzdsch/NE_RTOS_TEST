/*
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2025-10-29 12:12:13
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-01-02 17:13:37
 * @FilePath: \MDK-ARMd:\RoboMaster\code\NE_RTOS_TEST\Components\motor\DM_Motor.h
 * @Description: 达妙电机不同发送模式相关定义
 */
#ifndef DM_MOTOR_H
#define DM_MOTOR_H

#include "DM_Motor_typedef.h"
#include "BSP_CAN.h"
#include "MotorCtrl.h"

#define DM_KP_MAX 500.0f
#define DM_KP_MIN 0.0f

#define DM_KD_MAX 5.0f
#define DM_KD_MIN 0.0f

typedef enum {
  DM_MOTOR_STATE_DISABLE,
  DM_MOTOR_STATE_ENABLE
} DM_Motor_State_e;

typedef enum
{
  DM_MOTOR_CTRL_MODE_MIT,     // MIT模式
  DM_MOTOR_CTRL_MODE_POS_SPD, // 位置速度模式
  DM_MOTOR_CTRL_MODE_SPD      // 速度模式
} DM_Motor_CtrlMode_e;

/// @brief 达妙电机错误码 4bit
typedef enum {
  DM_MOTOR_ERR_NONE_DISABLE = 0x0, // 失能
  DM_MOTOR_ERR_NONE_ENABLE = 0x1, // 使能
  DM_MOTOR_ERR_OVER_VOL = 0x8, // 超压
  DM_MOTOR_ERR_UNDER_VOL = 0x9, // 欠压
  DM_MOTOR_ERR_OVER_CRT = 0xA, // 过流
  DM_MOTOR_ERR_MOS_OVER_TEMP = 0xB, // MOS过温
  DM_MOTOR_ERR_COIL_OVER_TEMP = 0xC, // 绕组过温
  DM_MOTOR_ERR_LOST_COMMUNICATION = 0xD, // 通信丢失
  DM_MOTOR_ERR_OVER_LOAD = 0xE // 负载过载
} DM_Motor_Err_e;

typedef struct {
  DM_Motor_Err_e err_code; // 错误码 4bit
  uint8_t id; // 上位机设置的id 4bit
  float pos_rad; // 位置 单位: 弧度
  float spd_radps; // 速度 单位: 弧度每秒
  float torq_nm; // 力矩 单位: 牛米
  float t_mos; // MOS温度 单位: ℃
  float t_rotor; // 转子温度 单位: ℃
  
  float last_pos_rad; // 上一次位置, 用于计算累计位置
  float total_pos_rad; // 累计位置
} DM_Motor_Measure_t;

typedef struct _DM_Motor_t {
  DM_Motor_State_e state;

  uint8_t id; // 上位机中设置的发送id 4bit
  uint32_t mst_id; // 上位机中设置的反馈id

  CAN_HandleTypeDef *hcan; // CAN句柄
  BSP_CAN_RxInstance rx_instance; // 接收实例

  DM_Motor_Measure_t measure; // 电机测量数据

  // DM_Motor_CtrlMode_e ctrl_mode; // 控制模式

  // 上位机设定数据
  float PMAX; // 建议在上位机中设为 12.566370614359172953850573533118
  float VMAX; // 建议在上位机中设为电机对应的速度(radps)上限
  float TMAX; // 建议在上位机中设为电机对应的扭矩(NM)上限

#if DM_TX_USE_MIT_MODE == 1
  BSP_CAN_TxInstance mit_tx_instance; // MIT模式发送实例
  DM_Motor_MIT_Data_t mit_mode_data; // MIT模式数据
#endif

#if DM_TX_USE_POS_SPD_MODE == 1
  BSP_CAN_TxInstance pos_spd_tx_instance; // 位置速度模式发送实例
  DM_Motor_POS_SPD_Data_u pos_spd_mode_data_u; // 位置速度模式数据
#endif

#if DM_TX_USE_SPD_MODE == 1
  BSP_CAN_TxInstance spd_tx_instance; // 速度模式发送实例
  DM_Motor_SPD_Data_u spd_mode_data_u; // 速度模式数据
#endif

  void (*MotorRxCallback)(struct _DM_Motor_t *motor); // 电机接收回调函数
  void *p_owner_moudle; // 电机所属模块(底盘/云台/机械臂等)
} DM_Motor_t;

typedef struct {
  CAN_HandleTypeDef *hcan;
  uint8_t id; // 上位机中设置的发送id 4bit
  uint32_t mst_id; // 上位机中设置的反馈id
  float PMAX;
  float VMAX;
  float TMAX;
  void (*MotorRxCallback)(struct _DM_Motor_t *);
  void *p_owner_moudle; // 电机所属模块(底盘/云台/机械臂等)
} DM_Motor_Init_t;

void DM_Motor_Init(DM_Motor_t *const p_motor, DM_Motor_Init_t* init);

void DM_Motor_ClearErr(DM_Motor_t *const p_motor);

void DM_Motor_Disable(DM_Motor_t *const p_motor);

void DM_Motor_Enable(DM_Motor_t *const p_motor);

void DM_Motor_SaveZeroPos(DM_Motor_t *const p_motor);

#if DM_TX_USE_MIT_MODE == 1

void DM_Motor_MIT_SetPD(DM_Motor_t *const p_motor, float kp, float kd);

void DM_Motor_MIT_SetTorq(DM_Motor_t *const p_motor, float torq);

void DM_Motor_MIT_SetPos(DM_Motor_t *const p_motor, float pos);

void DM_Motor_MIT_SetSpd(DM_Motor_t *const p_motor, float spd);

void DM_Motor_MIT_Send(DM_Motor_t *const p_motor);

#endif

#if DM_TX_USE_POS_SPD_MODE == 1

void DM_Motor_POS_SPD_SetPos(DM_Motor_t *const p_motor, float pos);

void DM_Motor_POS_SPD_SetSpd(DM_Motor_t *const p_motor, float spd);

void DM_Motor_POS_SPD_Send(DM_Motor_t *const p_motor);

#endif

#if DM_TX_USE_SPD_MODE == 1

void DM_Motor_SPD_SetSpd(DM_Motor_t *const p_motor, float spd);

void DM_Motor_SPD_Send(DM_Motor_t *const p_motor);

#endif

#endif
