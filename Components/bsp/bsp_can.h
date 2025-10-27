/*
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2025-10-26 16:48:17
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2025-10-27 23:51:31
 * @FilePath: \MDK-ARMd:\RoboMaster\code\NE_RTOS_TEST\Components\bsp\bsp_can.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef BSP_CAN_H
#define BSP_CAN_H

#include <stdint.h>
#include "can.h"

#define BSP_CAN_MAX_FILTER_CNT 28 // 最多可以使用的CAN过滤器数量
#define BSP_CAN_MAX_REGISTER_CNT 28  // 一条CAN总线最大的接收设备数量，值不应该超过BSP_CAN_MAX_FILTER_CNT

#pragma pack(1) ////////////////////////////////////////////////////////////////////////
typedef struct _TxInstance
{
    CAN_HandleTypeDef *can_handle; // can句柄
    CAN_TxHeaderTypeDef tx_header; // 发送报头
    uint32_t tx_id;                // 发送id
    uint8_t tx_buff[8];            // 发送缓存, 最大为8个字节
    uint32_t tx_mailbox;           // 发送邮箱
} BSP_CAN_TxInstance;

typedef struct _RxInstance
{
  CAN_HandleTypeDef *can_handle; // can句柄
  CAN_RxHeaderTypeDef rx_header; // 接收报头
  uint32_t rx_id;                // 接收id
  uint8_t rx_buff[8];            // 接收缓存, 最大为8个字节
  uint8_t rx_len;                // 接收长度(字节), 可能为0-8

  // 接收的回调函数,用于解析接收到的数据, 参数为接收实例指针
  void (*can_rx_callback)(struct _RxInstance *);

  void *owner_moudle; // 此接收实例所属的模块(回调中可能使用)
} BSP_CAN_RxInstance;
#pragma pack() /////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////// API ////////////////////////////////////////////////////////

/// @brief 初始化所有CAN设备, 包括使能中断
void BSP_CAN_InitAll();

/// @brief 初始化CAN发送实例
/// @param tx_instance CAN发送实例指针
/// @param hcan can句柄
/// @param tx_id 发送ID, 暂时没啥作用，真正的ID在tx_header中设置
/// @param tx_header 发送报头
void BSP_CAN_Tx_Init(BSP_CAN_TxInstance *tx_instance, CAN_HandleTypeDef *hcan, uint32_t tx_id, CAN_TxHeaderTypeDef tx_header);

/// @brief 注册can接收实例, 注册后会自动接收数据和调用回调(目前只实现了StdID), 注册失败会卡死!
/// @param g_can_rx_instance 接收实例指针(注意: 此指针指向的结构体必须是全局变量, 否则可能会非法访问内存!)
/// @param hcan can句柄
/// @param rx_id 接收ID
/// @param owner_moudle 所属模块指针
/// @param can_rx_callback 回调函数
void BSP_CAN_RxRegister(BSP_CAN_RxInstance *g_can_rx_instance, CAN_HandleTypeDef *const hcan,
                        const uint32_t rx_id, void *const owner_moudle,
                        void (*can_rx_callback)(struct _RxInstance *));

/// @brief 设置CAN发送实例的发送数据长度
/// @param tx_instance CAN发送实例指针,CAN发送实例指针, 注意确保该参数有效性
/// @param length 发送数据长度(字节)
void BSP_CAN_SetTxDLC(BSP_CAN_TxInstance *tx_instance, uint8_t length);

/// @brief 将CAN发送实例的发送缓存填入发送邮箱, 邮箱满或发送失败立即返回，不会阻塞
/// @param tx_instance CAN发送实例指针, 注意确保该参数有效性
/// @return 成功返回1，失败返回0
uint8_t BSP_CAN_Transmit(BSP_CAN_TxInstance *tx_instance);

#endif
