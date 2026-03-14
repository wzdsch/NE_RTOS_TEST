/*
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2026-02-24 11:50:35
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-03-13 22:21:01
 * @FilePath: \NE_RTOS_TEST\Components\app\usr_tasks.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef USE_TASKS_H
#define USE_TASKS_H

void BspCan1Task(void *argument);
void BspCan2Task(void *argument);
void PushRodTask(void *argument);
void ChassisTask(void *argument);
void CanCustomCommTask(void *argument);

#endif
