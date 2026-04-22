/*
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2026-03-11 21:35:10
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-03-25 10:44:31
 * @FilePath: \proj_left_arm\app\moudle_logic.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "moudle_logic.h"
#include "remote_receive.h"
#include "arm.h"
#include "JY_ME01.h"
#include "usr_main.h"

void ArmLogic() {
  static ArmState_e last_state = ARM_STATE_DISABLE;
  if (arm_l_ctrl_data.state == ARM_STATE_DISABLE) {
    Arm_Disable(&arm);
  }
  else if (arm_l_ctrl_data.state == ARM_STATE_ENABLE) {
    if (last_state == ARM_STATE_DISABLE) {
      Arm_Enable(&arm);
    }
  }
  Arm_SetTarget(&arm, arm_l_ctrl_data.yaw1_tar_rad, arm_l_ctrl_data.pitch1_tar_rad, \
                arm_l_ctrl_data.pitch2_tar_deg, arm_l_ctrl_data.yaw2_tar_rad, \
                arm_l_ctrl_data.end_pitch_tar_rad, arm_l_ctrl_data.end_yaw_tar_rad);
  last_state = arm.state;
}
