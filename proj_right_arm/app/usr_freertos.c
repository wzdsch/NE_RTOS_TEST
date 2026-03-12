/*
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2026-02-24 11:50:47
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-03-12 20:52:47
 * @FilePath: \NE_RTOS_TEST\Components\app\usr_freertos.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "usr_freertos.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "usr_tasks.h"
#include "bsp_can.h"

osThreadId_t armTaskHandle;
osThreadAttr_t armTask_attributes = {
  .name = "armTask",
  .stack_size = 1024,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t canTaskHandle;
osThreadAttr_t canTask_attributes = {
  .name = "canTask",
  .stack_size = 1024,
  .priority = (osPriority_t) osPriorityNormal,
};

osMessageQueueId_t canTxMsgQueueHandle;

void Usr_FreeRTOS_Init(void) {
  canTxMsgQueueHandle = osMessageQueueNew(20, sizeof(BSP_CAN_TxInstance), NULL);
  canTaskHandle = osThreadNew(CanTask, NULL, &canTask_attributes);
  armTaskHandle = osThreadNew(ArmTask, NULL, &armTask_attributes);
}
