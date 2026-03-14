/*
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2026-02-22 19:49:50
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-03-13 17:33:24
 * @FilePath: \NE_RTOS_TEST\Components\app\robot_logic.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "moudle_logic.h"
#include "remote_receive.h"
#include "arm.h"
#include "JY_ME01.h"
#include "push_rod.h"
#include "mecnum_chassis.h"
#include "can_custom_comm.h"

extern Arm_t arm;

extern CAN_Custom_ArmCtrlData_t arm_ctrl_data;

#define RC_JOY_DEADBAND 10

void ArmLogic() {
  static ArmState_e last_state = ARM_STATE_DISABLE;
  if (arm_ctrl_data.state == ARM_STATE_DISABLE) {
    Arm_Disable(&arm);
  }
  else if (arm_ctrl_data.state == ARM_STATE_ENABLE) {
    if (last_state == ARM_STATE_DISABLE) {
      Arm_Enable(&arm);
    }
    Arm_SetTarget(&arm, arm_ctrl_data.yaw1_tar_rad, arm_ctrl_data.pitch1_tar_rad, \
                  arm_ctrl_data.pitch2_tar_deg, arm_ctrl_data.yaw2_tar_rad, \
                  arm_ctrl_data.end_pitch_tar_rad, arm_ctrl_data.end_yaw_tar_rad);
  }
  last_state = arm.state;
}
