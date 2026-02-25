/*
 * @beforeAnnotation: 
 * Copyright (c) 2026 by 
 * """ The Robomaster team : NEXT-E from Xi'an University of Technology """
 * All Rights Reserved.
 * 
 * @Description: 双M3508电机推杆机构控制模块实现
 */

#include "push_rod.h"
#include "cmsis_os.h"
#include <string.h>
#include <math.h>

// 内部回调函数声明
static void _PushRod_MotorRxCallback(DJI_Motor_t *p_motor);

// 掉电检测逻辑
static void _PushRod_OfflineDetect(PushRod_t *const p_rod)
{
  uint32_t tick = osKernelGetTickCount();

  // 检测电机1
  if ((tick - p_rod->last_update_tick_1) > p_rod->offline_timeout)
  {
    if (p_rod->is_calibrated) // 只有之前校准过的才算掉电
    {
      p_rod->need_calib_1 = 1;
      p_rod->is_calibrated = 0;
      // 失能保护
      PushRod_Disable(p_rod);
    }
  }

  // 检测电机2
  if ((tick - p_rod->last_update_tick_2) > p_rod->offline_timeout)
  {
    if (p_rod->is_calibrated)
    {
      p_rod->need_calib_2 = 1;
      p_rod->is_calibrated = 0;
      PushRod_Disable(p_rod);
    }
  }

  // 如果需要校准且当前是使能状态，强制进入校准
  if ((p_rod->need_calib_1 || p_rod->need_calib_2) && 
       p_rod->state == PUSH_ROD_STATE_ENABLE)
  {
    p_rod->state = PUSH_ROD_STATE_CALIBRATING;
  }
}

void PushRod_Init(PushRod_t *const p_rod, const PushRod_Init_t *const init)
{
  // 检查指针
  if (p_rod == NULL || init == NULL)
  {
    while(1); // 错误死循环
  }

  memset(p_rod, 0, sizeof(PushRod_t));

  // 1. 初始化基础参数
  p_rod->state = PUSH_ROD_STATE_DISABLE;
  p_rod->max_allowed_err = init->max_allowed_err;
  p_rod->gravity_feedforward = init->gravity_feedforward;
  p_rod->offline_timeout = init->offline_timeout;
  
  // 标记需要初始校准
  p_rod->need_calib_1 = 1;
  p_rod->need_calib_2 = 1;
  p_rod->is_calibrated = 0;

  // 2. 初始化电机1
  DJI_Motor_Init_t motor_init_1;
  motor_init_1.hcan = init->hcan_1;
  motor_init_1.type = DJI_MOTOR_TYPE_M3508;
  motor_init_1.tx_id = init->tx_id_1;
  motor_init_1.rx_id = init->rx_id_1;
  motor_init_1.dir = init->dir_1;
  motor_init_1.zero_offset = init->zero_offset_1;
  motor_init_1.MotorRxCallback = _PushRod_MotorRxCallback;
  motor_init_1.p_owner_moudle = p_rod;
  DJI_Motor_Init(&p_rod->motor_1, &motor_init_1);

  // 3. 初始化电机2
  DJI_Motor_Init_t motor_init_2;
  motor_init_2.hcan = init->hcan_2;
  motor_init_2.type = DJI_MOTOR_TYPE_M3508;
  motor_init_2.tx_id = init->tx_id_2;
  motor_init_2.rx_id = init->rx_id_2;
  motor_init_2.dir = init->dir_2;
  motor_init_2.zero_offset = init->zero_offset_2;
  motor_init_2.MotorRxCallback = _PushRod_MotorRxCallback;
  motor_init_2.p_owner_moudle = p_rod;
  DJI_Motor_Init(&p_rod->motor_2, &motor_init_2);

  // 4. 初始化电机1控制器 (单环PID)
  // 注意：这里我们只启用PID，前馈将在PushRod_Calc中手动叠加
  uint32_t out_mask = MOTOR_CTRL_OUT_PID;
  
  MotorCtrl_Init(&p_rod->ctrl_1, MOTOR_CTRL_PID_INTERNAL, out_mask, init->max_out, &p_rod->motor_1);
  
  float pid_data_1[5] = {init->pos_kp, init->pos_ki, init->pos_kd, init->pos_out_limit, init->pos_int_limit};
  MotorCtrl_InternalPid_Init(&p_rod->ctrl_1, PID_POSITION, 
                              &p_rod->motor_1.processed_measure.pos_total_ecd_f, pid_data_1);

  // 5. 初始化电机2控制器
  MotorCtrl_Init(&p_rod->ctrl_2, MOTOR_CTRL_PID_INTERNAL, out_mask, init->max_out, &p_rod->motor_2);
  
  float pid_data_2[5] = {init->pos_kp, init->pos_ki, init->pos_kd, init->pos_out_limit, init->pos_int_limit};
  MotorCtrl_InternalPid_Init(&p_rod->ctrl_2, PID_POSITION, 
                              &p_rod->motor_2.processed_measure.pos_total_ecd_f, pid_data_2);

  // 6. 初始化误差PID
  PID_Init(&p_rod->err_pid, PID_POSITION, init->err_kp, init->err_ki, init->err_kd, init->err_out_limit, init->err_int_limit);
}

void PushRod_Enable(PushRod_t *const p_rod)
{
  if (p_rod == NULL) return;

  // 如果未校准，先进入校准模式
  if (!p_rod->is_calibrated || p_rod->need_calib_1 || p_rod->need_calib_2)
  {
    p_rod->state = PUSH_ROD_STATE_CALIBRATING;
  }
  else
  {
    p_rod->state = PUSH_ROD_STATE_ENABLE;
  }

  DJI_Motor_Enable(&p_rod->motor_1);
  DJI_Motor_Enable(&p_rod->motor_2);
  MotorCtrl_Enable(&p_rod->ctrl_1);
  MotorCtrl_Enable(&p_rod->ctrl_2);
}

void PushRod_Disable(PushRod_t *const p_rod)
{
  if (p_rod == NULL) return;

  p_rod->state = PUSH_ROD_STATE_DISABLE;
  
  DJI_Motor_Disable(&p_rod->motor_1);
  DJI_Motor_Disable(&p_rod->motor_2);
  MotorCtrl_Disable(&p_rod->ctrl_1);
  MotorCtrl_Disable(&p_rod->ctrl_2);
  
  DJI_Motor_SetCmd(&p_rod->motor_1, 0);
  DJI_Motor_SetCmd(&p_rod->motor_2, 0);
}

void PushRod_SetTarget(PushRod_t *const p_rod, float target)
{
  if (p_rod == NULL) return;
  p_rod->target_total_ecd = target;
}

void PushRod_Calc(PushRod_t *const p_rod)
{
  if (p_rod == NULL) return;

  // 掉电检测
  _PushRod_OfflineDetect(p_rod);

  // 状态机处理
  if (p_rod->state == PUSH_ROD_STATE_DISABLE)
  {
    // 确保电机停止
    DJI_Motor_SetCmd(&p_rod->motor_1, 0);
    DJI_Motor_SetCmd(&p_rod->motor_2, 0);
    return;
  }
  else if (p_rod->state == PUSH_ROD_STATE_CALIBRATING)
  {
    // 校准逻辑实现
    // 给电机发一个不大的电流持续一定时间，再将当前累计编码归0
    // 注意：在此状态下，需要手动调用 DJI_Motor_SetCmd 和 DJI_Motor_GroupTransmit 来发送电流
    // 校准完成后，记得清除 need_calib_1/2 标志，设置 is_calibrated = 1，并将 state 切回 ENABLE
    return;
  }

  // --- 以下为正常使能控制逻辑 ---

  // 计算双电机误差 (电机1 - 电机2)
  float err = p_rod->motor_1.processed_measure.pos_total_ecd_f - 
              p_rod->motor_2.processed_measure.pos_total_ecd_f;

  // 计算误差PID (目标是让误差为0)
  PID_Calc(&p_rod->err_pid, err, 0.0f);
  p_rod->err_pid_out = p_rod->err_pid.out;

  // 获取重力前馈
  // 重力前馈完善
  // 这里可以根据当前位置、速度等动态计算 gravity_ff
  // 例如：float gravity_ff = p_rod->gravity_feedforward * (1.0f + p_rod->motor_1.processed_measure.pos_total_ecd_f / 10000.0f);
  float gravity_ff = p_rod->gravity_feedforward; // 当前默认使用固定值

  // 设置主控制目标
  MotorCtrl_SetTarget(&p_rod->ctrl_1, p_rod->target_total_ecd);
  MotorCtrl_SetTarget(&p_rod->ctrl_2, p_rod->target_total_ecd);

  // 计算位置环PID
  MotorCtrl_Calc(&p_rod->ctrl_1);
  MotorCtrl_Calc(&p_rod->ctrl_2);

  // 构建总前馈
  // 策略：
  // 电机1前馈 = 重力前馈 - 误差PID输出 (如果电机1超前，减小出力)
  // 电机2前馈 = 重力前馈 + 误差PID输出 (如果电机1超前，增加电机2出力)
  float ff_1 = gravity_ff - p_rod->err_pid_out;
  float ff_2 = gravity_ff + p_rod->err_pid_out;

  // 手动叠加前馈并限幅
  float final_out_1 = p_rod->ctrl_1.final_out + ff_1;
  float final_out_2 = p_rod->ctrl_2.final_out + ff_2;

  // 限幅
  if (final_out_1 > p_rod->ctrl_1.max_out) final_out_1 = p_rod->ctrl_1.max_out;
  if (final_out_1 < -p_rod->ctrl_1.max_out) final_out_1 = -p_rod->ctrl_1.max_out;
  
  if (final_out_2 > p_rod->ctrl_2.max_out) final_out_2 = p_rod->ctrl_2.max_out;
  if (final_out_2 < -p_rod->ctrl_2.max_out) final_out_2 = -p_rod->ctrl_2.max_out;

  // 发送指令
  DJI_Motor_SetCmd(&p_rod->motor_1, (int16_t)final_out_1);
  DJI_Motor_SetCmd(&p_rod->motor_2, (int16_t)final_out_2);
}

void PushRod_StartCalibration(PushRod_t *const p_rod)
{
  if (p_rod == NULL) return;
  p_rod->need_calib_1 = 1;
  p_rod->need_calib_2 = 1;
  p_rod->is_calibrated = 0;
  p_rod->state = PUSH_ROD_STATE_CALIBRATING;
}

// --- 内部回调函数实现 ---

static void _PushRod_MotorRxCallback(DJI_Motor_t *p_motor)
{
  if (p_motor == NULL || p_motor->p_owner_moudle == NULL) return;
  
  PushRod_t *p_rod = (PushRod_t *)p_motor->p_owner_moudle;
  uint32_t tick = osKernelGetTickCount();

  // 判断是哪个电机的回调
  if (p_motor == &p_rod->motor_1)
  {
    p_rod->last_update_tick_1 = tick;
  }
  else if (p_motor == &p_rod->motor_2)
  {
    p_rod->last_update_tick_2 = tick;
  }
}
