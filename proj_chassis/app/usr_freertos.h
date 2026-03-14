/*
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2026-02-24 11:51:28
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-03-13 22:24:46
 * @FilePath: \proj_chassis\app\usr_freertos.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef USR_FREERTOS_H
#define USR_FREERTOS_H

#include "cmsis_os.h"

extern osThreadId_t bspCan1TaskHandle;
extern osThreadId_t bspCan2TaskHandle;
extern osThreadId_t pushRodTaskHandle;
extern osThreadId_t chassisTaskHandle;
extern osThreadId_t canCustomCommHandle;

extern osMessageQueueId_t bspCan1TxMsgQueueHandle;
extern osMessageQueueId_t bspCan2TxMsgQueueHandle;

void Usr_FreeRTOS_Init(void);

#endif
