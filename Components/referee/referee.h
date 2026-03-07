/*
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2026-02-28 16:14:10
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-03-01 14:32:55
 * @FilePath: \NE_RTOS_TEST\Components\referee\referee.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef REFEREE_H
#define REFEREE_H

#include "main.h"

#define USART_RX_BUF_LENGHT     512
#define REFEREE_FIFO_BUF_LENGTH 1024
extern void refereeINIT(void);
extern void referee_unpack_fifo_data(void);
extern void refereeReceiveHandler(void);

#endif
