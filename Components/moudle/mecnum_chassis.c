/*
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2026-02-17 19:36:22
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-03-10 20:13:01
 * @FilePath: \NE_RTOS_TEST\Components\moudle\mecnum_chassis.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "mecnum_chassis.h"
#include "refereeData_v1.6.h"

extern robot_state_t robot_state;

// sin/cos四分之PI
#define SIN_PI_OVER_4 0.7071f

static float Chassis_PowerLimit(MotorCtrl_t* p_ctrl);

void Chassis_Init(Chassis_t* p_chassis, Chassis_Init_t* p_init) {
  if (p_chassis == NULL || p_init == NULL) {
    while (1) {
      // param error
    }
  }

  p_init->motor_lf.p_owner_moudle = p_chassis;
  p_init->motor_rf.p_owner_moudle = p_chassis;
  p_init->motor_lb.p_owner_moudle = p_chassis;
  p_init->motor_rb.p_owner_moudle = p_chassis;

  p_chassis->state = CHASSIS_STATE_DISABLE;
  DJI_Motor_Init(&p_chassis->motor_lf, &(p_init->motor_lf));
  DJI_Motor_Init(&p_chassis->motor_rf, &(p_init->motor_rf));
  DJI_Motor_Init(&p_chassis->motor_lb, &(p_init->motor_lb));
  DJI_Motor_Init(&p_chassis->motor_rb, &(p_init->motor_rb));

  MotorCtrl_Init(&p_chassis->motor_lf_ctrl, MOTOR_CTRL_PID_INTERNAL, MOTOR_CTRL_OUT_POSTPROCESS, 16384.f, p_chassis);
  MotorCtrl_Init(&p_chassis->motor_rf_ctrl, MOTOR_CTRL_PID_INTERNAL, MOTOR_CTRL_OUT_POSTPROCESS, 16384.f, p_chassis);
  MotorCtrl_Init(&p_chassis->motor_lb_ctrl, MOTOR_CTRL_PID_INTERNAL, MOTOR_CTRL_OUT_POSTPROCESS, 16384.f, p_chassis);
  MotorCtrl_Init(&p_chassis->motor_rb_ctrl, MOTOR_CTRL_PID_INTERNAL, MOTOR_CTRL_OUT_POSTPROCESS, 16384.f, p_chassis);

  MotorCtrl_InternalPid_Init(&p_chassis->motor_lf_ctrl, &p_chassis->motor_lf.processed_measure.spd_rpm_f, &p_init->pid_motor_lf);
  MotorCtrl_InternalPid_Init(&p_chassis->motor_rf_ctrl, &p_chassis->motor_rf.processed_measure.spd_rpm_f, &p_init->pid_motor_rf);
  MotorCtrl_InternalPid_Init(&p_chassis->motor_lb_ctrl, &p_chassis->motor_lb.processed_measure.spd_rpm_f, &p_init->pid_motor_lb);
  MotorCtrl_InternalPid_Init(&p_chassis->motor_rb_ctrl, &p_chassis->motor_rb.processed_measure.spd_rpm_f, &p_init->pid_motor_rb);

  MotorCtrl_SetPostProcess(&p_chassis->motor_lf_ctrl, Chassis_PowerLimit);
  MotorCtrl_SetPostProcess(&p_chassis->motor_rf_ctrl, Chassis_PowerLimit);
  MotorCtrl_SetPostProcess(&p_chassis->motor_lb_ctrl, Chassis_PowerLimit);
  MotorCtrl_SetPostProcess(&p_chassis->motor_rb_ctrl, Chassis_PowerLimit);
}

void Chassis_Enable(Chassis_t* p_chassis) {
  p_chassis->state = CHASSIS_STATE_ENABLE;
  DJI_Motor_Enable(&p_chassis->motor_lf);
  DJI_Motor_Enable(&p_chassis->motor_rf);
  DJI_Motor_Enable(&p_chassis->motor_lb);
  DJI_Motor_Enable(&p_chassis->motor_rb);
  MotorCtrl_Enable(&p_chassis->motor_lf_ctrl);
  MotorCtrl_Enable(&p_chassis->motor_rf_ctrl);
  MotorCtrl_Enable(&p_chassis->motor_lb_ctrl);
  MotorCtrl_Enable(&p_chassis->motor_rb_ctrl);
}

void Chassis_Disable(Chassis_t* p_chassis) {
  p_chassis->state = CHASSIS_STATE_DISABLE;
  DJI_Motor_Disable(&p_chassis->motor_lf);
  DJI_Motor_Disable(&p_chassis->motor_rf);
  DJI_Motor_Disable(&p_chassis->motor_lb);
  DJI_Motor_Disable(&p_chassis->motor_rb);
  MotorCtrl_Disable(&p_chassis->motor_lf_ctrl);
  MotorCtrl_Disable(&p_chassis->motor_rf_ctrl);
  MotorCtrl_Disable(&p_chassis->motor_lb_ctrl);
  MotorCtrl_Disable(&p_chassis->motor_rb_ctrl);
}

void Chassis_Calc(Chassis_t* p_chassis, float spd_x, float spd_y, float spd_z) {
  if (p_chassis->state == CHASSIS_STATE_DISABLE) {
    return;
  }

  float chassis_spd_x = spd_x * SIN_PI_OVER_4 - spd_y * SIN_PI_OVER_4;
  float chassis_spd_y = spd_x * SIN_PI_OVER_4 + spd_y * SIN_PI_OVER_4;

  MotorCtrl_SetTarget(&p_chassis->motor_lf_ctrl, chassis_spd_y + spd_z);
  MotorCtrl_SetTarget(&p_chassis->motor_rf_ctrl, chassis_spd_x + spd_z);
  MotorCtrl_SetTarget(&p_chassis->motor_lb_ctrl, -chassis_spd_x + spd_z);
  MotorCtrl_SetTarget(&p_chassis->motor_rb_ctrl, -chassis_spd_y + spd_z);

  MotorCtrl_Calc(&p_chassis->motor_lf_ctrl);
  MotorCtrl_Calc(&p_chassis->motor_rf_ctrl);
  MotorCtrl_Calc(&p_chassis->motor_lb_ctrl);
  MotorCtrl_Calc(&p_chassis->motor_rb_ctrl);

  DJI_Motor_SetCmd(&p_chassis->motor_lf, p_chassis->motor_lf_ctrl.final_out);
  DJI_Motor_SetCmd(&p_chassis->motor_rf, p_chassis->motor_rf_ctrl.final_out);
  DJI_Motor_SetCmd(&p_chassis->motor_lb, p_chassis->motor_lb_ctrl.final_out);
  DJI_Motor_SetCmd(&p_chassis->motor_rb, p_chassis->motor_rb_ctrl.final_out);
}

/**
 * @brief  西交利物浦大学+香港科技大学功率限制移植
 * @param  motor: 电机结构体
 * @return float: 功率限制后的电流值
 */
float Chassis_PowerLimit(MotorCtrl_t *p_ctrl) {
  // uint16_t max_power_limit = robot_state.chassis_power_limit;
  // float initial_give_power = 0;  // initial power from PID calculation
  // static float initial_total_power = 0;
  // static float initial_total_power_last = 0;
  // static float scaled_give_power = 0;

  // static float spd_total_err = 0;
  // static float spd_total_err_last = 0;
  // float spd_err =usr_fabsf(p_ctrl->target - p_ctrl->realSpeedF);

  // float toque_coefficient = 1.99688994e-6f;  // (20/16384)*(0.3)*(187/3591)/9.55
  // float a = 1.23e-07;                        // k1
  // float k2 = 1.453e-07;                      // k2

  // static uint8_t count = 0;
  // initial_give_power = motor->pidOutput0 * toque_coefficient * motor->realSpeed
  //                      + k2 * motor->realSpeed * motor->realSpeed
  //                      + a * motor->pidOutput0 * motor->pidOutput0 + constant;
  // if (initial_give_power >= 0) {  // negative power not included (transitory)
  //   initial_total_power += initial_give_power;
  //   spd_total_err += spd_err;
  // }
  // count++;

  // if (count == 4) {
  //   count = 0;

  //   initial_total_power_last = initial_total_power;
  //   initial_total_power = 0;

  //   spd_total_err_last = spd_total_err;
  //   spd_total_err = 0;
  // }

  // if (initial_total_power_last > max_power_limit)  // determine if larger than max power
  // {

  //   float errorConfidence = 0.5; // 误差权重，误差越大，修正误差越快
  //   if (spd_total_err_last > ERROR_POWER_DISTRIBUTION_SET)
  //   {
  //       errorConfidence = 1.0f;
  //   }
  //   else if (spd_total_err_last > PROP_POWER_DISTRIBUTION_SET)
  //   {
  //     errorConfidence = (spd_total_err_last - PROP_POWER_DISTRIBUTION_SET) / (ERROR_POWER_DISTRIBUTION_SET - PROP_POWER_DISTRIBUTION_SET);
  //     if (errorConfidence < 0.0f) {
  //       errorConfidence = 0.0f;
  //     }
  //       if (errorConfidence > 1.0f) {
  //           errorConfidence = 1.0f;
  //       }
  //   }
  //   else
  //   {
  //       errorConfidence = 0.0f;
  //   }
    
  //    // new：误差/方差，相对误差
  //   float powerWeight_Error = spd_err / spd_total_err_last;                  // 误差权重（该电机速度误差/总速度误差）
  //   float powerWeight_Prop = initial_give_power / initial_total_power_last;  // 功率权重
  //   float powerWeight =
  //     errorConfidence * powerWeight_Error + (1.0f - errorConfidence) * powerWeight_Prop;

  //   scaled_give_power = max_power_limit * powerWeight;  // get scaled power
  //   if (scaled_give_power < 0) {
  //     return motor->pidOutput0;
  //   }

  //   fp32 b = toque_coefficient * motor->realSpeed;
  //   fp32 c = k2 * motor->realSpeed * motor->realSpeed - scaled_give_power + constant;
  //   float inside = b * b - 4 * a * c;
  //   if (motor->pidOutput0 > 0)  // Selection of the calculation formula according
  //                               // to the direction of the original moment
  //   {
  //     if (inside < 0) {
  //       // inside = 0;
  //     }
  //     fp32 temp = (-b + sqrt(inside))
  //                 / (2 * a);  ////////////////////////////////////////////////////
  //     if (temp > 16000) {
  //       return 16000;
  //     }
  //     else
  //       return temp;
  //   }
  //   else {
  //     fp32 temp = (-b - sqrt(inside))
  //                 / (2 * a);  ////////////////////////////////////////////////////
  //     if (temp < -16000) {
  //       return -16000;
  //     }
  //     else
  //       return temp;
  //   }
  // }
  // else {
  //   return motor->pidOutput0;
  // }
  return p_ctrl->pid_out;
}
