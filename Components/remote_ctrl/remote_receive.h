/*
 * @beforeAnnotation: 
 * Copyright (c) 2026 by 
 * """ The Robomaster team : NEXT-E from Xi'an University of Technology """
 * All Rights Reserved. 
 * 
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2026-01-07 11:36:00
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-03-13 16:22:23
 * @FilePath: \NE_RTOS_TEST\Components\remote_ctrl\remote_receive.h
 * @Description: This file is by Guo Hongting
 */
#ifndef REMOTE_RECEIVE_H
#define REMOTE_RECEIVE_H
#include "stdint.h"
#include "FreeRTOS.h"

/* 宏定义 */
/* 一帧的字节 */
#define RC_FRAME_NUM     25

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

  int16_t swa;    // 拨杆A
  int16_t swb;    // 拨杆B
  int16_t swc;   // 拨杆C
  int16_t swd;   // 拨杆D

  int16_t vra;   // 旋钮A
  int16_t vrb;   // 旋钮B

  uint8_t fsi6_flag; // 0x00
  uint8_t fsi6_end;  // 0x00

} FSI6Data_t;

extern FSI6Data_t fsi6_data;                      // 拆包后的数据

/* DMA和串口空闲中断初始化 */
extern void FSI6_BUS_IDLEHandler_Init(void);
/* DMA和串口空闲中断启用 */
extern void FSI6_BUS_IDLEHandler(void);

#endif
