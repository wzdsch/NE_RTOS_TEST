#include "robot_logic.h"
#include "remote_receive.h"
#include "arm.h"
#include "JY_ME01.h"

#define LIMIT(val, min, max) ((val) < (min) ? (min) : ((val) > (max) ? (max) : (val)))

#define RC_JOY_DEADBAND 10

float yaw1 = -0.36f, pitch1 = -0.8f, yaw2 = 1.13f, pitch2 = .0f, end_pitch = 0.f, end_yaw = 0.f;
float yaw1_step = 0.000005f, pitch1_step = 0.000005f, pitch2_step = 0.0002f, yaw2_step = 0.000005f, \
      end_pitch_step = 0.4f, end_yaw_step = 0.8f;

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
      yaw1 = arm.yaw1_8009p.processed_measure.pos_rad;
      pitch1 = arm.pitch1_8009p.processed_measure.pos_rad;
      pitch2 = JY_ME01.angle;
      yaw2 = arm.yaw2_4310.processed_measure.pos_rad;
      end_pitch = (arm.end1_2006.processed_measure.pos_total_ecd_f - arm.end2_2006.processed_measure.pos_total_ecd_f) / 2.f;
      end_yaw = (arm.end1_2006.processed_measure.pos_total_ecd_f + arm.end2_2006.processed_measure.pos_total_ecd_f) / 2.f;
    } else if (fsi6_data.right_ch2 == FSI6_CHANNEL_MID || fsi6_data.right_ch2 == FSI6_CHANNEL_MAX) {
      if (last_state == ARM_DISABLE) {
        Arm_Enable(&arm);
      }

      // ------ enable logical ------
      // set target & calculate
      if (fsi6_data.right_ch1 == FSI6_CHANNEL_MIN) {
        yaw1 -= (RC_Joy_Process(fsi6_data.left_x, RC_JOY_DEADBAND) - FSI6_CHANNEL_MID) * yaw1_step;
        pitch1 += (RC_Joy_Process(fsi6_data.left_y, RC_JOY_DEADBAND) - FSI6_CHANNEL_MID) * pitch1_step;
        pitch2 -= (RC_Joy_Process(fsi6_data.right_y, RC_JOY_DEADBAND) - FSI6_CHANNEL_MID) * pitch2_step;
        yaw2 -= (RC_Joy_Process(fsi6_data.right_x, RC_JOY_DEADBAND) - FSI6_CHANNEL_MID) * yaw2_step;
      } else if (fsi6_data.right_ch1 == FSI6_CHANNEL_MAX) {
        // end_1 += (RC_Joy_Process(fsi6_data.right_x, RC_JOY_DEADBAND) - FSI6_CHANNEL_MID) * end_yaw_step;
        // end_2 += (RC_Joy_Process(fsi6_data.right_x, RC_JOY_DEADBAND) - FSI6_CHANNEL_MID) * end_yaw_step;
        // end_1 += (RC_Joy_Process(fsi6_data.right_y, RC_JOY_DEADBAND) - FSI6_CHANNEL_MID) * end_pitch_step;
        // end_2 -= (RC_Joy_Process(fsi6_data.right_y, RC_JOY_DEADBAND) - FSI6_CHANNEL_MID) * end_pitch_step;
        end_pitch += (RC_Joy_Process(fsi6_data.right_y, RC_JOY_DEADBAND) - FSI6_CHANNEL_MID) * end_pitch_step;
        end_yaw += (RC_Joy_Process(fsi6_data.right_x, RC_JOY_DEADBAND) - FSI6_CHANNEL_MID) * end_yaw_step;
      }
      yaw1 = LIMIT(yaw1, -3.f, 3.f);
      pitch1 = LIMIT(pitch1, 0.f, 1.5f);
      pitch2 = LIMIT(pitch2, 0.f, 90.f);
      yaw2 = LIMIT(yaw2, -6.f, 6.f);
      end_pitch = LIMIT(end_pitch, -100000.f, 100000.f);
      end_yaw = LIMIT(end_yaw, -100000.f, 100000.f);

      Arm_SetTarget(&arm, ARM_LOAD_NONE, yaw1, pitch1, pitch2, yaw2, end_pitch, end_yaw);
      Arm_Calc(&arm);

      if (fsi6_data.right_ch2 == FSI6_CHANNEL_MAX) {
        // 发送
        DJI_Motor_GroupTransmit(&hcan1, DJI_MOTOR_TX_200);
        DM_Motor_MIT_Send(&arm.yaw1_8009p);
        DM_Motor_MIT_Send(&arm.pitch1_8009p);
        DM_Motor_MIT_Send(&arm.yaw2_4310);
      }
    }
    last_state = arm.state;
}
