#ifndef MECNUM_CHASSIS_H
#define MECNUM_CHASSIS_H

#include "DJI_Motor.h"
#include "MotorCtrl.h"

/// @brief 底盘状态枚举
typedef enum {
  CHASSIS_STATE_DISABLE,
  CHASSIS_STATE_ENABLE
} Chassis_State_e;

typedef struct _ {
  // 底盘状态 默认失能
  Chassis_State_e state;

  // 左前轮
  DJI_Motor_t motor_lf;
  MotorCtrl_t motor_lf_ctrl;

  // 右前轮
  DJI_Motor_t motor_rf;
  MotorCtrl_t motor_rf_ctrl;

  // 左后轮
  DJI_Motor_t motor_lb;
  MotorCtrl_t motor_lb_ctrl;

  // 右后轮
  DJI_Motor_t motor_rb;
  MotorCtrl_t motor_rb_ctrl;
} Chassis_t;

typedef struct {
  DJI_Motor_Init_t motor_lf;
  DJI_Motor_Init_t motor_rf;
  DJI_Motor_Init_t motor_lb;
  DJI_Motor_Init_t motor_rb;
  PID_Init_t pid_motor_lf;
  PID_Init_t pid_motor_rf;
  PID_Init_t pid_motor_lb;
  PID_Init_t pid_motor_rb;
} Chassis_Init_t;

/// @brief 底盘初始化
/// @param p_chassis 底盘指针
/// @param p_init 初始化指针
void Chassis_Init(Chassis_t* p_chassis, Chassis_Init_t* p_init);

/// @brief 底盘使能
/// @param p_chassis 底盘指针
void Chassis_Enable(Chassis_t* p_chassis);

/// @brief 底盘失能
/// @param p_chassis 底盘指针
void Chassis_Disable(Chassis_t* p_chassis);

/// @brief 底盘计算
/// @param p_chassis 底盘指针
/// @param spd_x 设定速度x (右为正方向)
/// @param spd_y 设定速度y (前为正方向)
/// @param spd_z 设定速度z (顺时针为正方向)
void Chassis_Calc(Chassis_t* p_chassis, float spd_x, float spd_y, float spd_z);

#endif
