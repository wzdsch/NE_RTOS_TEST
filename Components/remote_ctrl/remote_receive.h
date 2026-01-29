/*
 * @beforeAnnotation: 
 * Copyright (c) 2026 by 
 * """ The Robomaster team : NEXT-E from Xi'an University of Technology """
 * All Rights Reserved. 
 * 
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2026-01-07 11:36:00
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-01-29 07:40:20
 * @FilePath: \NE_RTOS_TEST\Components\remote_ctrl\remote_receive.h
 * @Description: This file is by Guo Hongting
 */
#ifndef REMOTE_RECEIVE_H
#define REMOTE_RECEIVE_H
#include "stdint.h"
#include "FreeRTOS.h"

/* 宏定义 */
/* 一帧的字节 */
#define RC_FRAME_NUM     25u

#define FSI6_CHANNEL_MIN 240
#define FSI6_CHANNEL_MID 1024
#define FSI6_CHANNEL_MAX 1807

/* 遥控数据 */
typedef struct {

  uint8_t fsi6_start; // 0x0F

  int16_t left_x;
  int16_t left_y;
  int16_t right_x;
  int16_t right_y;

  int16_t left_ch1;    // 左1
  int16_t left_ch2;    // 左2 
  int16_t right_ch2;   // 右2
  int16_t right_ch1;   // 右1

  int16_t left_knob;   // 左旋钮
  int16_t right_knob;  // 右旋钮

  uint8_t fsi6_flag; // 0x00
  uint8_t fsi6_end;  // 0x00

}FSI6Data_t;

extern FSI6Data_t FSI6Data;                      // 拆包后的数据

/* DMA和串口空闲中断初始化 */
extern void FSI6_BUS_IDLEHandler_Init(void);
/* DMA和串口空闲中断启用 */
extern void FSI6_BUS_IDLEHandler(void);

#endif
