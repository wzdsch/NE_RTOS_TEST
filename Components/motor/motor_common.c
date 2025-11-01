/*
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2025-10-29 12:09:26
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2025-11-01 15:31:09
 * @FilePath: \MDK-ARMd:\RoboMaster\code\NE_RTOS_TEST\Components\motor\motor_common.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "motor_common.h"
#include "string.h"

inline void _MotorCommon_Ctrl_Init(MotorCommon_Ctrl_t* p_ctrl, const MotorCtrlMode_e ctrl_mode) {
  p_ctrl->ctrl_mode = ctrl_mode;
  p_ctrl->set_state = MOTOR_CTRL_DISABLE;
  memset(&p_ctrl->set_target, 0, sizeof(p_ctrl->set_target));
}

inline void _MotorCommon_Measure_Init(MotorCommon_Measure_t* p_measure) {
  p_measure->pos_ecd = 0;
  p_measure->total_pos_ecd = 0;
  p_measure->spd_rpm = 0;
  p_measure->tor_nm = 0;
  p_measure->tempreture = 0;
}

void MotorCommon_Init(MotorCommon_t* p_motor_common, const MotorCtrlMode_e ctrl_mode, uint32_t out_values, float max_out) {
  if (p_motor_common == NULL) {
    return;
  }
  
  _MotorCommon_Ctrl_Init(&p_motor_common->ctrl_data, ctrl_mode);
  _MotorCommon_Measure_Init(&p_motor_common->measure_data);

  p_motor_common->external_pos = 0;
  p_motor_common->external_spd = 0;
  p_motor_common->external_tor = 0;

  p_motor_common->pCustomCtrlAlgorithm = NULL;
  p_motor_common->pPreProcess = NULL;
  p_motor_common->pPostProcess = NULL;
  p_motor_common->pFeedForward = NULL;

  // 清空pid, 初始化pid需要另外的函数
  memset(&(p_motor_common->pid_pos), 0, sizeof(p_motor_common->pid_pos));
  memset(&(p_motor_common->pid_spd), 0, sizeof(p_motor_common->pid_spd));
  memset(&(p_motor_common->pid_tor), 0, sizeof(p_motor_common->pid_tor));

  p_motor_common->custom_ctrl_algorithm_out = 0;
  p_motor_common->pre_process_out = 0;
  p_motor_common->post_process_out = 0;
  p_motor_common->feed_forward_out = 0;

  p_motor_common->out_values = out_values;
  p_motor_common->max_out = max_out;
  p_motor_common->final_out = 0;
}

void MotorCommon_Pid_Init(MotorCommon_t* p_motor_common, float p_pos_pid_data[5], float p_vel_pid_data[5], \
                          float p_tor_pid_data[5]) {
  if (p_motor_common == NULL) {
    return;
  }

  if (p_pos_pid_data != NULL) {
    PID_Init(&p_motor_common->pid_pos, PID_POSITION, p_pos_pid_data[0], p_pos_pid_data[1], \
             p_pos_pid_data[2], p_pos_pid_data[3], p_pos_pid_data[4]);
  }
  if (p_vel_pid_data != NULL) {
    PID_Init(&p_motor_common->pid_spd, PID_POSITION, p_vel_pid_data[0], p_vel_pid_data[1], \
             p_vel_pid_data[2], p_vel_pid_data[3], p_vel_pid_data[4]);
  }
  if (p_tor_pid_data != NULL) {
    PID_Init(&p_motor_common->pid_tor, PID_POSITION, p_tor_pid_data[0], p_tor_pid_data[1], \
             p_tor_pid_data[2], p_tor_pid_data[3], p_tor_pid_data[4]);
  }
}

void MotorCommon_Ctrl_Disable(MotorCommon_Ctrl_t* p_motor_common_ctrl) {
  p_motor_common_ctrl->set_state = MOTOR_CTRL_DISABLE;
}

void MotorCommon_Ctrl_Enable(MotorCommon_Ctrl_t* p_motor_common_ctrl) {
  p_motor_common_ctrl->set_state = MOTOR_CTRL_ENABLE;
}

void _MotorCommon_PID_Calc(MotorCommon_t* p_motor_common) {
  if (p_motor_common == NULL) {
    return;
  }

  switch (p_motor_common->ctrl_data.ctrl_mode) {
    case MOTOR_CTRL_LOOP_POS_SPD:
      PID_Calc(&p_motor_common->pid_pos, p_motor_common->measure_data.pos_ecd, p_motor_common->ctrl_data.set_target.set_pos_ecd);
      p_motor_common->pid_out = PID_Calc(&p_motor_common->pid_spd, p_motor_common->measure_data.spd_rpm, p_motor_common->pid_pos.out);
      break;
    case MOTOR_CTRL_LOOP_POS:
      p_motor_common->pid_out = PID_Calc(&p_motor_common->pid_pos, p_motor_common->measure_data.pos_ecd, p_motor_common->ctrl_data.set_target.set_pos_ecd);
      break;
    case MOTOR_CTRL_LOOP_TOTALPOS_SPD:
      PID_Calc(&p_motor_common->pid_pos, p_motor_common->measure_data.total_pos_ecd, p_motor_common->ctrl_data.set_target.set_total_pos_ecd);
      p_motor_common->pid_out = PID_Calc(&p_motor_common->pid_spd, p_motor_common->measure_data.spd_rpm, p_motor_common->pid_pos.out);
      break;
    case MOTOR_CTRL_LOOP_TOTALPOS:
      p_motor_common->pid_out = PID_Calc(&p_motor_common->pid_pos, p_motor_common->measure_data.total_pos_ecd, p_motor_common->ctrl_data.set_target.set_total_pos_ecd);
      break;
    case MOTOR_CTRL_LOOP_SPD:
      p_motor_common->pid_out = PID_Calc(&p_motor_common->pid_spd, p_motor_common->measure_data.spd_rpm, p_motor_common->ctrl_data.set_target.set_spd_rpm);
      break;
    case MOTOR_CTRL_LOOP_TOR:
      p_motor_common->pid_out = PID_Calc(&p_motor_common->pid_tor, p_motor_common->measure_data.tor_nm, p_motor_common->ctrl_data.set_target.set_tor_nm);
      break;
    case MOTOR_CTRL_TOR:
      p_motor_common->pid_out = p_motor_common->ctrl_data.set_target.set_tor_nm;
      break;
    case MOTOR_CTRL_EXTERNAL_LOOP_POS_SPD:
      PID_Calc(&p_motor_common->pid_pos, p_motor_common->external_pos, p_motor_common->ctrl_data.set_target.set_external_pos);
      p_motor_common->pid_out = PID_Calc(&p_motor_common->pid_spd, p_motor_common->external_spd, p_motor_common->pid_pos.out);
      break;
    case MOTOR_CTRL_EXTERNAL_LOOP_POS:
      p_motor_common->pid_out = PID_Calc(&p_motor_common->pid_pos, p_motor_common->external_pos, p_motor_common->ctrl_data.set_target.set_external_pos);
      break;
    case MOTOR_CTRL_EXTERNAL_LOOP_SPD:
      p_motor_common->pid_out = PID_Calc(&p_motor_common->pid_spd, p_motor_common->external_spd, p_motor_common->ctrl_data.set_target.set_external_spd);
      break;
    case MOTOR_CTRL_EXTERNAL_LOOP_TOR:
      p_motor_common->pid_out = PID_Calc(&p_motor_common->pid_tor, p_motor_common->external_tor, p_motor_common->ctrl_data.set_target.set_external_tor);
      break;
    default:
      break;
  }
}

void MotorCommon_Calc(MotorCommon_t* p_motor_common) {
  if (p_motor_common == NULL) {
    return;
  }

  if (p_motor_common->ctrl_data.set_state == MOTOR_CTRL_DISABLE) {
    // 注意: 电机为失能模式，并不会清零final_out, 而是在具体的电机类中发送失能数据
    return;
  } else if (p_motor_common->ctrl_data.set_state == MOTOR_CTRL_ENABLE) {
    // 预处理函数
    if (p_motor_common->pPreProcess != NULL) {
      p_motor_common->pre_process_out = p_motor_common->pPreProcess(p_motor_common);
    }

    // 根据控制模式计算pid
    _MotorCommon_PID_Calc(p_motor_common);

    // 自定义控制算法
    if (p_motor_common->pCustomCtrlAlgorithm != NULL) {
      p_motor_common->custom_ctrl_algorithm_out = p_motor_common->pCustomCtrlAlgorithm(p_motor_common);
    }

    // 前馈算法
    if (p_motor_common->pFeedForward != NULL) {
      p_motor_common->feed_forward_out = p_motor_common->pFeedForward(p_motor_common);
    }

    // 后处理函数
    if (p_motor_common->pPostProcess != NULL) {
      p_motor_common->post_process_out = p_motor_common->pPostProcess(p_motor_common);
    }

    // 通过设定输出来计算输出值
    float temp_out = 0;
    if (p_motor_common->out_values & MOTOR_COMMON_OUT_FEEDFWD) {
      temp_out += p_motor_common->feed_forward_out;
    }
    if (p_motor_common->out_values & MOTOR_COMMON_OUT_PID) {
      temp_out += p_motor_common->pid_out;
    }
    if (p_motor_common->out_values & MOTOR_COMMON_OUT_PREPROCESS) {
      temp_out += p_motor_common->pre_process_out;
    }
    if (p_motor_common->out_values & MOTOR_COMMON_OUT_CUSTOM) {
      temp_out += p_motor_common->custom_ctrl_algorithm_out;
    }
    if (p_motor_common->out_values & MOTOR_COMMON_OUT_POSTPROCESS) {
      temp_out += p_motor_common->post_process_out;
    }

    if (temp_out > p_motor_common->max_out) {
      temp_out = p_motor_common->max_out;
    } else if (temp_out < -p_motor_common->max_out) {
      temp_out = -p_motor_common->max_out;
    }
    p_motor_common->final_out = temp_out; // 最终输出
  }
}
