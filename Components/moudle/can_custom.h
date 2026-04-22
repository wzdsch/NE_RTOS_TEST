/*
 * @beforeAnnotation: 
 * Copyright (c) 2026 by 
 * """ The Robomaster team : NEXT-E from Xi'an University of Technology """
 * All Rights Reserved. 
 * 
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2026-01-05 21:29:50
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-03-25 12:07:21
 * @FilePath: \NE_RTOS_TEST\Components\moudle\can_custom_comm.h
 * @Description: CAN自定义通信头文件，自动拆包，会占用 startID ~ startID + 拆包数 - 1 的连续ID
 */
#ifndef CAN_CUSTOM_H
#define CAN_CUSTOM_H

#include "can_custom_api.h"
#include "can_custom_structs.h"
#include "bsp_can.h"

typedef struct {
  BSP_CAN_TxInstance tx_instance;
  uint32_t start_tx_id;
  uint8_t pack_cnt;
  void* p_buf;
  uint8_t size;
  void (*pPackFunc)(void* p_buf);
} CAN_Custom_Tx_t;

typedef struct {
  CAN_HandleTypeDef* hcan;
  uint32_t start_tx_id;
  uint32_t IDE;
  void* p_buf;
  uint8_t size;
  void (*pPackFunc)(void* p_buf);
} CAN_Custom_Tx_Init_t;

typedef struct {
  uint32_t start_rx_id;
  uint32_t last_rx_id; // 上次接收的ID
  uint32_t err_cnt; // 错误计数器
  uint8_t pack_cnt;
  BSP_CAN_RxInstance* p_rx_instances; // 接收实例数组指针
  void* p_buf;
  uint8_t size;
  void (*pUnpackFunc)(void* p_buf);
} CAN_Custom_Rx_t;

typedef struct {
  CAN_HandleTypeDef* hcan;
  uint32_t start_rx_id;
  uint32_t IDE;
  void* p_buf;
  uint8_t size;
  void (*pUnpackFunc)(void* p_buf); // 解包函数，可以为NULL
} CAN_Custom_Rx_Init_t;

void CAN_Custom_Tx_Init(CAN_Custom_Tx_t* p_tx, CAN_Custom_Tx_Init_t* init);
void CAN_Custom_Tx_PackSend(CAN_Custom_Tx_t* p_tx);

void CAN_Custom_Rx_Init(CAN_Custom_Rx_t* p_rx, CAN_Custom_Rx_Init_t* init);

#endif
