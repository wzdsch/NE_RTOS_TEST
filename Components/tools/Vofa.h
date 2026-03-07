/*
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2026-02-22 17:39:22
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-02-28 14:27:11
 * @FilePath: \NE_RTOS_TEST\Components\tools\Vofa.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef VOFA_H
#define VOFA_H
#include "usart.h"
#include "struct_typedef.h"
// 使用联合体，避免对数据的重复搬运
typedef struct VofaDatas {
  fp32 set;
  fp32 feedback;
  fp32 TargetSpeed1;
  fp32 Speed1;
  uint8_t tail1;
  uint8_t tail2;
  uint8_t tail3;
  uint8_t tail4;
} VofaData;

union VofaDATA {
  VofaData VofaD;
  uint8_t ch[20];  // 4*4+4=20
};
extern union VofaDATA Vofa;
extern void JustFloat(fp32 set, fp32 feedback, fp32 target, fp32 speed, UART_HandleTypeDef *huart);
#endif
