#ifndef CAN_CUSTOM_COMM_H
#define CAN_CUSTOM_COMM_H

#include "bsp_can.h"
#include <stdint.h>

// CAN数据包最大有效载荷（每帧8字节，去掉协议头）
#define CAN_CUSTOM_COMM_MAX_PAYLOAD 7

// 协议头结构（1字节）
typedef struct {
    uint8_t index : 5;      // 分包索引（5位，支持最多32个分包）
    uint8_t last : 1;       // 是否为最后一个包（1位）
    uint8_t reserved : 2;   // 保留位
} CAN_PacketHeader_t;

// 发送实例结构体
typedef struct {
    BSP_CAN_TxInstance can_tx;           // BSP CAN发送实例
    uint8_t *p_data;                     // 待发送数据指针
    uint16_t data_size;                  // 待发送数据总大小
    uint16_t sent_size;                  // 已发送数据大小

    // 打包函数指针：将结构体数据打包到发送缓冲区
    void (*pack_func)(void *p_struct, uint8_t *p_buf, uint16_t buf_size);

    uint8_t packet_index;                // 当前分包索引
    uint8_t is_sending;                  // 发送状态标志
} CAN_CustomTx_t;

// 接收实例结构体
typedef struct {
    BSP_CAN_RxInstance can_rx;           // BSP CAN接收实例
    uint8_t *p_recv_buf;                 // 接收缓冲区
    uint16_t recv_buf_size;              // 接收缓冲区大小
    uint16_t recv_size;                  // 已接收数据大小

    // 解包函数指针：将接收缓冲区数据解包到结构体
    void (*unpack_func)(uint8_t *p_buf, uint16_t buf_size, void *p_struct);

    uint8_t packet_index;                // 当前分包索引
    uint8_t is_receiving;                // 接收状态标志

    // 接收完成回调函数
    void (*recv_complete_callback)(void *p_struct);
} CAN_CustomRx_t;

// 发送相关API

/**
 * @brief 初始化自定义CAN发送实例
 * @param p_tx 发送实例指针
 * @param hcan CAN句柄
 * @param tx_id 发送ID
 * @param IDE ID类型（CAN_ID_STD或CAN_ID_EXT）
 * @param p_struct 结构体指针
 * @param struct_size 结构体大小
 * @param pack_func 打包函数指针
 */
void CAN_CustomTx_Init(CAN_CustomTx_t *p_tx, CAN_HandleTypeDef *hcan, 
                       uint32_t tx_id, uint32_t IDE,
                       void *p_struct, uint16_t struct_size,
                       void (*pack_func)(void *, uint8_t *, uint16_t));

/**
 * @brief 更新发送数据并开始发送
 * @param p_tx 发送实例指针
 * @return 0-发送成功，1-发送中，2-参数错误
 */
uint8_t CAN_CustomTx_UpdateAndSend(CAN_CustomTx_t *p_tx);

/**
 * @brief 发送处理函数（在主循环中调用）
 * @param p_tx 发送实例指针
 * @return 0-发送完成，1-发送中
 */
uint8_t CAN_CustomTx_Process(CAN_CustomTx_t *p_tx);

// 接收相关API

/**
 * @brief 初始化自定义CAN接收实例
 * @param p_rx 接收实例指针
 * @param hcan CAN句柄
 * @param rx_id 接收ID
 * @param IDE ID类型（CAN_ID_STD或CAN_ID_EXT）
 * @param p_struct 结构体指针
 * @param struct_size 结构体大小
 * @param unpack_func 解包函数指针
 * @param recv_complete_callback 接收完成回调函数
 */
void CAN_CustomRx_Init(CAN_CustomRx_t *p_rx, CAN_HandleTypeDef *hcan,
                       uint32_t rx_id, uint32_t IDE,
                       void *p_struct, uint16_t struct_size,
                       void (*unpack_func)(uint8_t *, uint16_t, void *),
                       void (*recv_complete_callback)(void *));

/**
 * @brief CAN接收回调函数（内部使用）
 * @param p_rx_instance BSP CAN接收实例指针
 */
void CAN_CustomRx_Callback(BSP_CAN_RxInstance *p_rx_instance);

/**
 * @brief 获取接收到的数据大小
 * @param p_rx 接收实例指针
 * @return 已接收的数据大小
 */
uint16_t CAN_CustomRx_GetRecvSize(CAN_CustomRx_t *p_rx);

#endif
