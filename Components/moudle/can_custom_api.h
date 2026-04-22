/*
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2026-03-13 16:56:13
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-03-25 10:07:19
 * @FilePath: \proj_right_armd:\RoboMaster\code\Enginner\Components\moudle\can_custom_comm_api.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef CAN_CUSTOM_COMM_API_H
#define CAN_CUSTOM_COMM_API_H

#define CAN_CUSTOM_COMM_START_ID_ARM_L_CTRL 0x600       // 左臂控制can通信起始ID
#define CAN_CUSTOM_COMM_START_ID_ARM_L_FDB_TARGET 0x610 // 左臂反馈目标can通信起始ID

#define CAN_CUSTOM_COMM_START_ID_ARM_R_CTRL 0x620       // 右臂控制can通信起始ID
#define CAN_CUSTOM_COMM_START_ID_ARM_R_FDB_TARGET 0x630 // 右臂反馈目标can通信起始ID

#endif
