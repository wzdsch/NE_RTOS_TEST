/*
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2026-03-12 16:45:03
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-03-17 19:01:36
 * @FilePath: \proj_left_arm\app\robot_ctrl_logic.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef ROBOT_CTRL_LOGIC_H
#define ROBOT_CTRL_LOGIC_H

#define K_RAD_TO_DEG 57.29577951308232f
#define K_DEG_TO_RAD 0.017453292519943295769236907684886f

#include "can_custom_comm.h"

void CtrlLogic_RC(void);

#endif
