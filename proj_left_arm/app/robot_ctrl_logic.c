/*
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2026-03-12 16:44:09
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-03-17 19:04:29
 * @FilePath: \proj_left_arm\app\robot_ctrl_logic.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "robot_ctrl_logic.h"
#include "remote_receive.h"
#include "arm.h"

#define RC_JOY_DEADBAND 10

extern Arm_t arm;

extern CAN_Custom_ArmCtrlData_t arm1_ctrl_data;
extern CAN_Custom_ArmCtrlData_t arm2_ctrl_data;
extern CAN_Custom_ChassisCtrlData_t chassis_ctrl_data;
extern CAN_Custom_PushRodsCtrlData_t push_rods_ctrl_data;

extern ArmTarget_t arm2_fdb_target;

float yaw1_step = 0.000005f, pitch1_step = 0.000005f, pitch2_step = 0.000005f * K_RAD_TO_DEG, yaw2_step = 0.000005f, \
      end_pitch_step = 0.000005f, end_yaw_step = 0.000005f;

/// @brief 处理FS-i6遥杆死区
/// @param val 原始摇杆值
/// @param deadband 死区 (一般不超过100)
/// @return 以FSI6_CHANNEL_MID为中心的值
static inline int16_t RC_Joy_Process(int16_t val, uint16_t deadband) {
  int16_t offset = val - FSI6_CHANNEL_MID;
  if (offset > deadband) {
    return val - deadband;
  } else if (offset < -deadband) {
    return val + deadband;
  } else {
    return FSI6_CHANNEL_MID;
  }
}

// swa 上 控制底盘: 
//       |     swb      |    swc    |     swd
//  up   |  push_f_back |  disable  |  push_b_back
// mid   |      /       |  enable   |      /
// down  |  push_f_out  |  pump_on  |  push_b_out

// swa 下 控制机械臂:
//       |      swb     |    swc    |     swd
//  up   |   left_arm   |  disable  |     arm
// mid   |       /      |  enable   |      /
// down  |   right_arm  |  pump_on  |     end

void CtrlLogic_RC() {
  if (fsi6_data.swc == FSI6_CHANNEL_MIN) {
    arm1_ctrl_data.state = ARM_STATE_DISABLE;
    arm2_ctrl_data.state = ARM_STATE_DISABLE;
    chassis_ctrl_data.state = CHASSIS_STATE_DISABLE;
    push_rods_ctrl_data.state_f = PUSH_ROD_STATE_DISABLE;
    push_rods_ctrl_data.state_b = PUSH_ROD_STATE_DISABLE;
  }
  else if (fsi6_data.swc == FSI6_CHANNEL_MID || fsi6_data.swc == FSI6_CHANNEL_MAX) {
    arm1_ctrl_data.state = ARM_STATE_ENABLE;
    arm2_ctrl_data.state = ARM_STATE_ENABLE;
    chassis_ctrl_data.state = CHASSIS_STATE_ENABLE;
    push_rods_ctrl_data.state_f = PUSH_ROD_STATE_ENABLE;
    push_rods_ctrl_data.state_b = PUSH_ROD_STATE_ENABLE;
  }

  if (fsi6_data.swa == FSI6_CHANNEL_MIN) {
    // swa拨至上册控制底盘
    chassis_ctrl_data.spd_x = (RC_Joy_Process(fsi6_data.left_x, RC_JOY_DEADBAND) - FSI6_CHANNEL_MID) * 10;
    chassis_ctrl_data.spd_y = (RC_Joy_Process(fsi6_data.left_y, RC_JOY_DEADBAND) - FSI6_CHANNEL_MID) * 10;
    chassis_ctrl_data.spd_z = (RC_Joy_Process(fsi6_data.right_x, RC_JOY_DEADBAND) - FSI6_CHANNEL_MID) * 10;
    if (fsi6_data.swb == FSI6_CHANNEL_MIN) {
      push_rods_ctrl_data.pos_f = 0;
    }
    else if (fsi6_data.swb == FSI6_CHANNEL_MAX) {
      push_rods_ctrl_data.pos_f = 1;
    }
    if (fsi6_data.swd == FSI6_CHANNEL_MIN) {
      push_rods_ctrl_data.pos_b = 0;
    }
    else if (fsi6_data.swd == FSI6_CHANNEL_MAX) {
      push_rods_ctrl_data.pos_b = 1;
    }
    // 控制底盘时，保持机械臂target
    arm1_ctrl_data.yaw1_tar_rad = arm.target.yaw1_rad;
    arm1_ctrl_data.pitch1_tar_rad = arm.target.pitch1_rad;
    arm1_ctrl_data.pitch2_tar_deg = arm.target.pitch2_deg;
    arm1_ctrl_data.yaw2_tar_rad = arm.target.yaw2_rad;
    arm1_ctrl_data.end_pitch_tar_rad = arm.target.end_pitch_rad;

    arm2_ctrl_data.yaw1_tar_rad = arm2_fdb_target.yaw1_rad;
    arm2_ctrl_data.pitch1_tar_rad = arm2_fdb_target.pitch1_rad;
    arm2_ctrl_data.pitch2_tar_deg = arm2_fdb_target.pitch2_deg;
    arm2_ctrl_data.yaw2_tar_rad = arm2_fdb_target.yaw2_rad;
    arm2_ctrl_data.end_pitch_tar_rad = arm2_fdb_target.end_pitch_rad;
  }
  else if (fsi6_data.swa == FSI6_CHANNEL_MAX) {
    if (fsi6_data.swd == FSI6_CHANNEL_MIN) {
      arm1_ctrl_data.yaw1_tar_rad = arm.target.yaw1_rad + (RC_Joy_Process(fsi6_data.left_x, RC_JOY_DEADBAND) - FSI6_CHANNEL_MID) * yaw1_step;
      arm1_ctrl_data.pitch1_tar_rad = arm.target.pitch1_rad + (RC_Joy_Process(fsi6_data.left_y, RC_JOY_DEADBAND) - FSI6_CHANNEL_MID) * pitch1_step;
      arm1_ctrl_data.pitch2_tar_deg = arm.target.pitch2_deg + (RC_Joy_Process(fsi6_data.right_y, RC_JOY_DEADBAND) - FSI6_CHANNEL_MID) * pitch2_step;
      arm1_ctrl_data.yaw2_tar_rad = arm.target.yaw2_rad - (RC_Joy_Process(fsi6_data.right_x, RC_JOY_DEADBAND) - FSI6_CHANNEL_MID) * yaw2_step;
    }
    else if (fsi6_data.swd == FSI6_CHANNEL_MAX) {
      arm1_ctrl_data.end_pitch_tar_rad = arm.target.end_pitch_rad + (RC_Joy_Process(fsi6_data.right_y, RC_JOY_DEADBAND) - FSI6_CHANNEL_MID) * end_pitch_step;
      arm1_ctrl_data.end_yaw_tar_rad = arm.target.end_yaw_rad + (RC_Joy_Process(fsi6_data.right_x, RC_JOY_DEADBAND) - FSI6_CHANNEL_MID) * end_yaw_step;
    }
    if (fsi6_data.swb == FSI6_CHANNEL_MAX) {
      arm1_ctrl_data.end_pitch_tar_rad = arm.target.end_pitch_rad - 0.005f;
    }
    // 控制机械臂时，底盘target清零
    chassis_ctrl_data.spd_x = 0;
    chassis_ctrl_data.spd_y = 0;
    chassis_ctrl_data.spd_z = 0;
  }
}
