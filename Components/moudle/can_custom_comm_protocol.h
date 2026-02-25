#ifndef CAN_CUSTOM_PROTOCOL_H
#define CAN_CUSTOM_PROTOCOL_H

#pragma pack(1)

typedef struct {
  float yaw1_tar;
  float pitch1_tar;
  float pitch2_tar;
  float yaw2_tar;
  float end_pitch_tar;
  float end_yaw_tar;
} CAN_Custom_ArmData_t;

typedef struct {
  float left_x;
  float left_y;
  float right_x;
  float right_y;
} CAN_Custom_RCData_t;

#pragma pack()

extern CAN_Custom_RCData_t can_custom_rc_data;

void CAN_Custom_RCData_Pack(void* p_buf);
void CAN_Custom_RCData_Unpack(void* p_buf);

#endif
