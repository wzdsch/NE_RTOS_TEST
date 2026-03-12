/*
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2026-02-22 19:49:50
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-03-12 16:33:54
 * @FilePath: \NE_RTOS_TEST\Components\app\robot_logic.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "moudle_logic.h"
#include "remote_receive.h"
#include "arm.h"
#include "JY_ME01.h"

#define RC_JOY_DEADBAND 10

float yaw1_step = 0.000005f, pitch1_step = 0.000005f, pitch2_step = 0.0002f, yaw2_step = 0.000005f, \
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

void ArmLogic_RC() {
  static ArmState_e last_state = ARM_DISABLE;
  if (fsi6_data.right_ch2 == FSI6_CHANNEL_MIN) {
    Arm_Disable(&arm);
  } else if (fsi6_data.right_ch2 == FSI6_CHANNEL_MID || fsi6_data.right_ch2 == FSI6_CHANNEL_MAX) {
    if (last_state == ARM_DISABLE) {
      Arm_Enable(&arm);
    }

    // ------ enable logical ------
    // set target & calculate
    if (fsi6_data.right_ch1 == FSI6_CHANNEL_MIN) {
      arm.target.yaw1 = arm.target.yaw1 + (RC_Joy_Process(fsi6_data.left_x, RC_JOY_DEADBAND) - FSI6_CHANNEL_MID) * yaw1_step;
      arm.target.pitch1 = arm.target.pitch1 + (RC_Joy_Process(fsi6_data.left_y, RC_JOY_DEADBAND) - FSI6_CHANNEL_MID) * pitch1_step;
      arm.target.pitch2 = arm.target.pitch2 + (RC_Joy_Process(fsi6_data.right_y, RC_JOY_DEADBAND) - FSI6_CHANNEL_MID) * pitch2_step;
      arm.target.yaw2 = arm.target.yaw2 - (RC_Joy_Process(fsi6_data.right_x, RC_JOY_DEADBAND) - FSI6_CHANNEL_MID) * yaw2_step;
    } else if (fsi6_data.right_ch1 == FSI6_CHANNEL_MAX) {
      arm.target.end_pitch = arm.target.end_pitch + (RC_Joy_Process(fsi6_data.right_y, RC_JOY_DEADBAND) - FSI6_CHANNEL_MID) * end_pitch_step;
      arm.target.end_yaw = arm.target.end_yaw + (RC_Joy_Process(fsi6_data.right_x, RC_JOY_DEADBAND) - FSI6_CHANNEL_MID) * end_yaw_step;
    }
  }
  last_state = arm.state;
}
