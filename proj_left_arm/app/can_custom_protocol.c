/*
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2026-03-25 12:39:34
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-03-25 20:08:16
 * @FilePath: \proj_left_arm\app\can_custom_protocol.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "can_custom_protocol.h"
#include "usr_main.h"

void CAN_Custom_ArmFdbTarget_Pack(void* p_buf) {
    CAN_Custom_ArmFdbTarget_t* p_data = (CAN_Custom_ArmFdbTarget_t*)p_buf;
    p_data->yaw1_rad = arm.target.yaw1_rad;
    p_data->pitch1_rad = arm.target.pitch1_rad;
    p_data->pitch2_deg = arm.target.pitch2_deg;
    p_data->yaw2_rad = arm.target.yaw2_rad;
    p_data->end_yaw_rad = arm.target.end_yaw_rad;
    p_data->end_pitch_rad = arm.target.end_pitch_rad;
}
