/*
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2025-10-29 12:11:24
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2025-11-01 15:57:15
 * @FilePath: \MDK-ARMd:\RoboMaster\code\NE_RTOS_TEST\Components\motor\motor_common.h
 * @Description: 此模块为电机通用控制/反馈数据模块，所有电机继承包含此模块，实现所有电机统一的接口
 */
#ifndef MOTOR_COMMON_H
#define MOTOR_COMMON_H

#include <stdint.h>
#include "pid.h"

// 电机输出, 使用按位或链接, 表示总输出把这些量加起来
#define MOTOR_COMMON_OUT_PID          (0x0001U << 0) // PID输出
#define MOTOR_COMMON_OUT_CUSTOM       (0x0001U << 1) // 自定义控制函数输出
#define MOTOR_COMMON_OUT_FEEDFWD      (0x0001U << 2) // 前馈输出
#define MOTOR_COMMON_OUT_PREPROCESS   (0x0001U << 3) // 预处理输出
#define MOTOR_COMMON_OUT_POSTPROCESS  (0x0001U << 4) // 后处理输出

// 电机控制状态
typedef enum
{
  MOTOR_CTRL_DISABLE, // 失能
  MOTOR_CTRL_ENABLE   // 使能
} MotorCtrlState_e;

typedef enum
{
  // 内部传感器闭环（使用电机自身编码器/传感器）
  MOTOR_CTRL_LOOP_POS_SPD,      // 位置环串速度环（目标：set_pos_ecd；反馈：内部位置+内部速度）
  MOTOR_CTRL_LOOP_POS,          // 单位置环（目标：set_pos_ecd；反馈：内部位置）
  MOTOR_CTRL_LOOP_TOTALPOS_SPD, // 累计位置环串速度环（目标：set_pos_total_ecd；反馈：内部累计位置+内部速度）
  MOTOR_CTRL_LOOP_TOTALPOS,     // 累计位置环（目标：set_pos_total_ecd；反馈：内部累计位置）
  MOTOR_CTRL_LOOP_SPD,          // 速度环（目标：set_spd_rpm；反馈：内部速度）
  MOTOR_CTRL_LOOP_TOR,          // 力矩环（目标：set_tor_nm；反馈：内部力矩）
  MOTOR_CTRL_TOR,               // 力矩直接控制（无闭环，目标：set_tor_nm）

  // 外部传感器闭环（使用外部传感器反馈，如IMU、外部编码器）
  MOTOR_CTRL_EXTERNAL_LOOP_POS_SPD, // 外部位置环串速度环（目标：set_external_pos；反馈：外部位置+外部速度）
  MOTOR_CTRL_EXTERNAL_LOOP_POS,     // 外部单位置环（目标：set_external_pos；反馈：外部位置）
  MOTOR_CTRL_EXTERNAL_LOOP_SPD,     // 外部速度环（目标：set_external_spd；反馈：外部速度）
  MOTOR_CTRL_EXTERNAL_LOOP_TOR      // 外部力矩环（目标：set_external_tor；反馈：外部力矩）
} MotorCtrlMode_e;

#pragma pack(1)

// 电机控制目标联合体，由控制方式自动判断使用哪个成员
typedef union
{
  int64_t set_total_pos_ecd; // 累计编码器位置(单位: 电角度) 0 ~ 8191
  uint16_t set_pos_ecd;      // 单圈编码值(单位: 电角度) 0 ~ 8191
  int16_t set_spd_rpm;       // 转速(单位: 转每分)
  float set_tor_nm;          // 力矩(单位: 牛米)

  float set_external_pos; // 外部位置(IMU等传感器), 单位不确定
  float set_external_spd; // 外部速度(IMU等传感器), 单位不确定
  float set_external_tor; // 外部力矩(外部传感器),  单位不确定
} MotorCommon_Target_u;

// 电机控制结构体
typedef struct
{
  MotorCtrlMode_e ctrl_mode;       // 控制模式
  MotorCtrlState_e set_state;      // 使能/失能
  MotorCommon_Target_u set_target; // 控制目标
} MotorCommon_Ctrl_t;

// 电机反测量据结构体, 在具体电机类中赋值
typedef struct
{
  int64_t total_pos_ecd; // 累计编码器位置(单位: 电角度)
  uint16_t pos_ecd;      // 单圈编码值(单位: 电角度) 0 ~ 8191
  int16_t spd_rpm;       // 转速(单位: 转每分)
  float tor_nm;          // 力矩(单位: 牛米)
  uint8_t tempreture;    // 温度(单位: 摄氏度)
} MotorCommon_Measure_t;

typedef struct _MotorCommon_t
{
  MotorCommon_Ctrl_t ctrl_data;
  MotorCommon_Measure_t measure_data;

  // pid结构体
  Pid_t pid_pos; // 位置环
  Pid_t pid_spd; // 速度环
  Pid_t pid_tor; // 力矩环

  float external_pos; // 外部位置(IMU等传感器), 单位不确定
  float external_spd; // 外部速度(IMU等传感器), 单位不确定
  float external_tor; // 外部力矩(外部传感器),  单位不确定

  float (*pCustomCtrlAlgorithm)(struct _MotorCommon_t *p_motor_common); // 自定义控制算法
  float (*pPreProcess)(struct _MotorCommon_t *p_motor_common);          // 控制算法预处理
  float (*pPostProcess)(struct _MotorCommon_t *p_motor_common);         // 控制算法后处理
  float (*pFeedForward)(struct _MotorCommon_t *p_motor_common);         // 前馈控制

  float pid_out;                   // pid最终输出
  float custom_ctrl_algorithm_out; // 控制算法输出
  float feed_forward_out;          // 前馈控制输出
  float pre_process_out;           // 控制算法预处理输出
  float post_process_out;          // 控制算法后处理输出
  
  // 电机设定输出, 使用按位或链接, 表示总输出把这些量加起来
  // 具体输出值, 用按位或连接以下值:
  // MOTOR_COMMON_OUT_PID, MOTOR_COMMON_OUT_CUSTOM, MOTOR_COMMON_OUT_FEEDFWD, MOTOR_COMMON_OUT_PREPROCESS, MOTOR_COMMON_OUT_POSTPROCESS
  uint32_t out_values;

  float max_out; // 最终总输出限幅(力矩)
  float final_out; // 最终输出
} MotorCommon_t;

#pragma pack()

void MotorCommon_Init(MotorCommon_t *p_motor_common, const MotorCtrlMode_e ctrl_mode, uint32_t out_values, float max_out);
void MotorCommon_Pid_Init(MotorCommon_t *p_motor_common, float p_pos_pid_data[5], float p_vel_pid_data[5],
                          float p_tor_pid_data[5]);
void MotorCommon_Ctrl_Disable(MotorCommon_Ctrl_t *p_motor_common_ctrl);
void MotorCommon_Ctrl_Enable(MotorCommon_Ctrl_t *p_motor_common_ctrl);
void MotorCommon_Calc(MotorCommon_t *p_motor_common);


#define MotorCommon_Ctrl_SetTarget(p_motor_common, target)                   \
  do                                                                         \
  {                                                                          \
    if (p_motor_common != NULL)                                              \
    {                                                                        \
      switch ((p_motor_common)->ctrl_data.ctrl_mode)                         \
      {                                                                      \
      case MOTOR_CTRL_LOOP_POS_SPD:                                          \
      case MOTOR_CTRL_LOOP_POS:                                              \
        (p_motor_common)->ctrl_data.set_target.set_pos_ecd = (target);       \
        break;                                                               \
      case MOTOR_CTRL_LOOP_TOTALPOS_SPD:                                     \
      case MOTOR_CTRL_LOOP_TOTALPOS:                                         \
        (p_motor_common)->ctrl_data.set_target.set_total_pos_ecd = (target); \
        break;                                                               \
      case MOTOR_CTRL_LOOP_SPD:                                              \
        (p_motor_common)->ctrl_data.set_target.set_spd_rpm = (target);       \
        break;                                                               \
      case MOTOR_CTRL_LOOP_TOR:                                              \
      case MOTOR_CTRL_TOR:                                                   \
        (p_motor_common)->ctrl_data.set_target.set_tor_nm = (target);        \
        break;                                                               \
      case MOTOR_CTRL_EXTERNAL_LOOP_POS_SPD:                                 \
      case MOTOR_CTRL_EXTERNAL_LOOP_POS:                                     \
        (p_motor_common)->ctrl_data.set_target.set_external_pos = (target);  \
        break;                                                               \
      case MOTOR_CTRL_EXTERNAL_LOOP_SPD:                                     \
        (p_motor_common)->ctrl_data.set_target.set_external_spd = (target);  \
        break;                                                               \
      case MOTOR_CTRL_EXTERNAL_LOOP_TOR:                                     \
        (p_motor_common)->ctrl_data.set_target.set_external_tor = (target);  \
        break;                                                               \
      default:                                                               \
        break;                                                               \
      }                                                                      \
    }                                                                        \
  } while (0)

#endif
