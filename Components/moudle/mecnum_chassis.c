#include "mecnum_chassis.h"

// sin/cos四分之PI
#define SIN_PI_OVER_4 0.7071f

static float Chassis_PowerLimit(MotorCtrl_t* p_ctrl);

void Chassis_Init(Chassis_t* p_chassis, Chassis_Init_t* p_init) {
  if (p_chassis == NULL || p_init == NULL) {
    while (1) {
      // param error
    }
  }

  p_chassis->state = CHASSIS_STATE_DISABLE;
  DJI_Motor_Init(&p_chassis->motor_lf, &(p_init->motor_lf));
  DJI_Motor_Init(&p_chassis->motor_rf, &(p_init->motor_rf));
  DJI_Motor_Init(&p_chassis->motor_lb, &(p_init->motor_lb));
  DJI_Motor_Init(&p_chassis->motor_rb, &(p_init->motor_rb));

  MotorCtrl_Init(&p_chassis->motor_lf_ctrl, MOTOR_CTRL_PID_INTERNAL, MOTOR_CTRL_OUT_POSTPROCESS, 16384.f, p_chassis);
  MotorCtrl_Init(&p_chassis->motor_rf_ctrl, MOTOR_CTRL_PID_INTERNAL, MOTOR_CTRL_OUT_POSTPROCESS, 16384.f, p_chassis);
  MotorCtrl_Init(&p_chassis->motor_lb_ctrl, MOTOR_CTRL_PID_INTERNAL, MOTOR_CTRL_OUT_POSTPROCESS, 16384.f, p_chassis);
  MotorCtrl_Init(&p_chassis->motor_rb_ctrl, MOTOR_CTRL_PID_INTERNAL, MOTOR_CTRL_OUT_POSTPROCESS, 16384.f, p_chassis);

  MotorCtrl_InternalPid_Init(&p_chassis->motor_lf_ctrl, PID_POSITION, &p_chassis->motor_lf.processed_measure.spd_rpm_f, p_init->pid_motor_lf);
  MotorCtrl_InternalPid_Init(&p_chassis->motor_rf_ctrl, PID_POSITION, &p_chassis->motor_rf.processed_measure.spd_rpm_f, p_init->pid_motor_rf);
  MotorCtrl_InternalPid_Init(&p_chassis->motor_lb_ctrl, PID_POSITION, &p_chassis->motor_lb.processed_measure.spd_rpm_f, p_init->pid_motor_lb);
  MotorCtrl_InternalPid_Init(&p_chassis->motor_rb_ctrl, PID_POSITION, &p_chassis->motor_rb.processed_measure.spd_rpm_f, p_init->pid_motor_rb);

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
  MotorCtrl_SetTarget(&p_chassis->motor_lb_ctrl, -chassis_spd_y + spd_z);
  MotorCtrl_SetTarget(&p_chassis->motor_rb_ctrl, -chassis_spd_x + spd_z);

  MotorCtrl_Calc(&p_chassis->motor_lf_ctrl);
  MotorCtrl_Calc(&p_chassis->motor_rf_ctrl);
  MotorCtrl_Calc(&p_chassis->motor_lb_ctrl);
  MotorCtrl_Calc(&p_chassis->motor_rb_ctrl);

  DJI_Motor_SetCmd(&p_chassis->motor_lf, p_chassis->motor_lf_ctrl.final_out);
  DJI_Motor_SetCmd(&p_chassis->motor_rf, p_chassis->motor_rf_ctrl.final_out);
  DJI_Motor_SetCmd(&p_chassis->motor_lb, p_chassis->motor_lb_ctrl.final_out);
  DJI_Motor_SetCmd(&p_chassis->motor_rb, p_chassis->motor_rb_ctrl.final_out);
}

static float Chassis_PowerLimit(MotorCtrl_t* p_ctrl) {
  return p_ctrl->pid_out;
}
