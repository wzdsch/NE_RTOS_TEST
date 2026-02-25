#include "can_custom_comm_protocol.h"
#include "remote_receive.h"

CAN_Custom_RCData_t can_custom_rc_data = {
  .left_x = FSI6_CHANNEL_MID,
  .left_y = FSI6_CHANNEL_MID,
  .right_x = FSI6_CHANNEL_MID,
  .right_y = FSI6_CHANNEL_MID
};

void CAN_Custom_RCData_Pack(void* p_buf) {
  CAN_Custom_RCData_t* p_data = (CAN_Custom_RCData_t*)p_buf;
  p_data->left_x = fsi6_data.left_x;
  p_data->left_y = fsi6_data.left_y;
  p_data->right_x = fsi6_data.right_x;
  p_data->right_y = fsi6_data.right_y;
}

void CAN_Custom_RCData_Unpack(void* p_buf) {
  CAN_Custom_RCData_t* p_data = (CAN_Custom_RCData_t*)p_buf;
  can_custom_rc_data.left_x = p_data->left_x;
  can_custom_rc_data.left_y = p_data->left_y;
  can_custom_rc_data.right_x = p_data->right_x;
  can_custom_rc_data.right_y = p_data->right_y;
}
