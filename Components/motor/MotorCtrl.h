/*
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2025-10-29 12:11:24
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-01-03 14:04:27
 * @FilePath: \MDK-ARMd:\RoboMaster\code\NE_RTOS_TEST\Components\motor\motor_common.h
 * @Description: 此模块为电机控制模块, 只负责数据的处理, 可用作处理电机数据的统一接口
 */
#ifndef MOTOR_CTRL_H
#define MOTOR_CTRL_H

#include <stdint.h>
#include "pid.h"

// 电机输出, 使用按位或链接, 表示总输出把这些量加起来
#define MOTOR_CTRL_OUT_PID          (0x0001U << 0) // PID输出
#define MOTOR_CTRL_OUT_CUSTOM       (0x0001U << 1) // 自定义控制函数输出
#define MOTOR_CTRL_OUT_FEEDFWD      (0x0001U << 2) // 前馈输出
#define MOTOR_CTRL_OUT_PREPROCESS   (0x0001U << 3) // 预处理输出
#define MOTOR_CTRL_OUT_POSTPROCESS  (0x0001U << 4) // 后处理输出

// 电机控制状态, 是否计算
typedef enum
{
  MOTOR_CTRL_DISABLE, // 失能
  MOTOR_CTRL_ENABLE   // 使能
} MotorCtrl_State_e;

typedef enum
{
  MOTOR_CTRL_PID_NONE, // 无
  MOTOR_CTRL_PID_INTERNAL, // 单闭环 (内环)
  MOTOR_CTRL_PID_DOUBLE // 双闭环
} MotorCtrl_PidMode_e;

typedef struct _MotorCtrl_t
{
  MotorCtrl_State_e state; // 使能 / 失能, 这个参数表示是否计算
  MotorCtrl_PidMode_e ctrl_mode; // 控制模式 (单环/双环/自定义)

  // pid结构体
  Pid_t pid_external; // 外环
  Pid_t pid_internal; // 内环 (单闭环使用此环)

  float set_target; // 设定目标

  // 反馈数据指针
  float *p_pid_ext_fdb; // pid外环反馈数据指针
  float *p_pid_int_fdb;

  float (*pCustomCtrlAlgorithm)(struct _MotorCtrl_t *p_motor_ctrl); // 自定义控制算法
  float (*pFeedForward)(struct _MotorCtrl_t *p_motor_ctrl);         // 前馈控制
  float (*pPreProcess)(struct _MotorCtrl_t *p_motor_ctrl);          // 控制算法预处理
  float (*pPostProcess)(struct _MotorCtrl_t *p_motor_ctrl);         // 控制算法后处理

  float pid_out;                   // pid最终输出
  float custom_ctrl_algorithm_out; // 控制算法输出
  float feed_forward_out;          // 前馈控制输出
  float pre_process_out;           // 控制算法预处理输出
  float post_process_out;          // 控制算法后处理输出

  // 电机设定输出, 使用按位或连接, 表示总输出把这些量加起来
  // 具体输出值, 用按位或连接以下值:
  // MOTOR_COMMON_OUT_PID, MOTOR_COMMON_OUT_CUSTOM, MOTOR_COMMON_OUT_FEEDFWD, MOTOR_COMMON_OUT_PREPROCESS, MOTOR_COMMON_OUT_POSTPROCESS
  uint32_t out_values;

  float max_out; // 最终总输出限幅(力矩)
  float final_out; // 最终输出

  void *p_owner_moudle; // 所属模块指针(一般为电机)
} MotorCtrl_t;

/// @brief 电机控制初始化
/// @param p_motor_ctrl 电机控制结构体指针
/// @param ctrl_mode 控制模式 (单环/双环/自定义)
/// @param out_values 输出增益, 按位或连接 MOTOR_CTRL_OUT_PID, MOTOR_CTRL_OUT_CUSTOM, MOTOR_CTRL_OUT_FEEDFWD, MOTOR_CTRL_OUT_PREPROCESS, MOTOR_CTRL_OUT_POSTPROCESS
/// @param max_out 
/// @param p_owner_moudle 所属模块指针(一般为电机)
void MotorCtrl_Init(MotorCtrl_t *p_motor_ctrl, MotorCtrl_PidMode_e ctrl_mode, uint32_t out_values, float max_out, void *p_owner_moudle);

/// @brief 双环pid外环初始化
/// @param p_motor_ctrl 电机控制结构体指针
/// @param pid_mode pid模式 (位置式 / 增量式)
/// @param p_ref 反馈值指针
/// @param p_pid_data pid数据 数组 {Kp, Ki, Kd, out_limit, integral_limit}
void MotorCtrl_ExternalPid_Init(MotorCtrl_t *p_motor_ctrl, PID_MODE_e pid_mode, float *p_ref, float p_pid_data[5]);

/// @brief 双环pid内环初始化 / 单闭环pid初始化
/// @param p_motor_ctrl 电机控制结构体指针
/// @param pid_mode pid模式 (位置式 / 增量式)
/// @param p_ref 反馈值指针
/// @param p_pid_data pid数据 数组 {Kp, Ki, Kd, out_limit, integral_limit}
void MotorCtrl_InternalPid_Init(MotorCtrl_t *p_motor_ctrl, PID_MODE_e pid_mode, float *p_ref, float p_pid_data[5]);

/// @brief 设置自定义控制算法
/// @param p_motor_ctrl 电机控制结构体指针
/// @param pCustomCtrlAlgorithm 自定义控制函数指针
void MotorCtrl_SetCustomCtrlAlgorithm(MotorCtrl_t *p_motor_ctrl, float (*pCustomCtrlAlgorithm)(struct _MotorCtrl_t *p_motor_ctrl));

/// @brief 设置预处理函数
/// @param p_motor_ctrl 电机控制结构体指针
/// @param pPreProcess 预处理函数指针
void MotorCtrl_SetPreProcess(MotorCtrl_t *p_motor_ctrl, float (*pPreProcess)(struct _MotorCtrl_t *p_motor_ctrl));

/// @brief 设置后处理函数
/// @param p_motor_ctrl 电机控制结构体指针
/// @param pPostProcess 后处理函数指针
void MotorCtrl_SetPostProcess(MotorCtrl_t *p_motor_ctrl, float (*pPostProcess)(struct _MotorCtrl_t *p_motor_ctrl));

/// @brief 设置前馈函数
/// @param p_motor_ctrl 电机控制结构体指针
/// @param pFeedForward 前馈函数指针
void MotorCtrl_SetFeedForward(MotorCtrl_t *p_motor_ctrl, float (*pFeedForward)(struct _MotorCtrl_t *p_motor_ctrl));

/// @brief 电机控制计算
/// @param p_motor_ctrl 电机控制结构体
void MotorCtrl_Calc(MotorCtrl_t *p_motor_ctrl);

/// @brief 设定电机控制目标值
/// @param p_motor_ctrl 电机控制结构体指针
/// @param target 目标值
void MotorCtrl_SetTarget(MotorCtrl_t *p_motor_ctrl, float target);

/// @brief 使能电机控制计算
/// @param p_motor_ctrl 
void MotorCtrl_Enable(MotorCtrl_t *p_motor_ctrl);

/// @brief 失能电机控制计算
/// @param p_motor_ctrl
void MotorCtrl_Disable(MotorCtrl_t *p_motor_ctrl);

#endif
