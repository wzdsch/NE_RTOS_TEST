/*
 * @beforeAnnotation: 
 * Copyright (c) 2026 by 
 * """ The Robomaster team : NEXT-E from Xi'an University of Technology """
 * All Rights Reserved. 
 * 
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2026-01-07 11:36:00
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-01-23 23:49:40
 * @FilePath: \NE_RTOS_TEST\Components\remote_ctrl\remote_receive.h
 * @Description: This file is by Guo Hongting
 */
#ifndef REMOTE_RECEIVE_H
#define REMOTE_RECEIVE_H
#include "stdint.h"
#include "FreeRTOS.h"
#include "semphr.h"

/* 宏定义 */
/* 一帧的字节 */
#define RC_FRAME_NUM     25u

/* 遥控数据 */
typedef struct FSI6Data_t{

  uint8_t fsi6_start; // 0x0F

  int16_t right_x;     
  int16_t right_y;         
  int16_t left_x;    
  int16_t left_y;		 
	
  int16_t left_ch1;    // 左1
  int16_t left_ch2;    // 左2 
  int16_t right_ch2;   // 右2
  int16_t right_ch1;   // 右1

  int16_t left_knob;   // 左旋钮
  int16_t right_knob;  // 右旋钮
	
  uint8_t fsi6_flag; // 0x00
  uint8_t fsi6_end;  // 0x00

}FSI6Data;

extern uint8_t W_BusRxBuffer[2][RC_FRAME_NUM]; // DMA 双缓冲区
extern uint8_t* fsi6_completed_buf;            // 指向本次接收完成的数据缓冲区
extern FSI6Data fsi6Data;                      // 拆包后的数据
extern SemaphoreHandle_t fsi6_rx_sem;    			 // 信号量 用于通知任务

/* 获取遥控数据 */
extern void GetFSI6Data(FSI6Data* fsi6Data, uint8_t* rxBuffer);
/* DMA和串口空闲中断初始化 */
extern void FSI6_BUS_IDLEHandler_Init(uint8_t *rx0_buf, uint8_t *rx1_buf, uint16_t dma_buf_num);
/* DMA和串口空闲中断启用 */
extern void FSI6_BUS_IDLEHandler(void);

#endif
