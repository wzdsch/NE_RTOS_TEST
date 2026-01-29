#include "can_custom_comm.h"
#include <string.h>

// 发送缓冲区（用于存储打包后的数据和协议头）
static uint8_t tx_buffer[CAN_CUSTOM_COMM_MAX_PAYLOAD + 1];

/**
 * @brief 将协议头打包到字节中
 * @param header 协议头结构体指针
 * @return 打包后的字节
 */
static uint8_t _PackHeader(CAN_PacketHeader_t *header)
{
    return ((header->index & 0x1F) << 1) | (header->last & 0x01);
}

/**
 * @brief 从字节解包协议头
 * @param byte 包含协议头的字节
 * @param header 协议头结构体指针
 */
static void _UnpackHeader(uint8_t byte, CAN_PacketHeader_t *header)
{
    header->index = (byte >> 1) & 0x1F;
    header->last = byte & 0x01;
}

// ==================== 发送功能实现 ====================

void CAN_CustomTx_Init(CAN_CustomTx_t *p_tx, CAN_HandleTypeDef *hcan,
                       uint32_t tx_id, uint32_t IDE,
                       void *p_struct, uint16_t struct_size,
                       void (*pack_func)(void *, uint8_t *, uint16_t))
{
    // 参数校验
    if (p_tx == NULL || p_struct == NULL || pack_func == NULL)
    {
        return;
    }

    // 分配发送缓冲区
    p_tx->p_data = (uint8_t *)p_struct;
    p_tx->data_size = struct_size;
    p_tx->pack_func = pack_func;

    // 初始化发送状态
    p_tx->sent_size = 0;
    p_tx->packet_index = 0;
    p_tx->is_sending = 0;

    // 初始化BSP CAN发送实例
    // 发送长度为8字节（1字节协议头 + 7字节数据）
    BSP_CAN_Tx_Init(&p_tx->can_tx, hcan, tx_buffer, tx_id, IDE, 8, CAN_RTR_DATA);
}

uint8_t CAN_CustomTx_UpdateAndSend(CAN_CustomTx_t *p_tx)
{
    // 参数校验
    if (p_tx == NULL)
    {
        return 2;  // 参数错误
    }

    // 如果正在发送，返回1
    if (p_tx->is_sending)
    {
        return 1;  // 发送中
    }

    // 调用打包函数将结构体数据打包到发送缓冲区
    p_tx->pack_func(p_tx->p_data, p_tx->p_data, p_tx->data_size);

    // 重置发送状态
    p_tx->sent_size = 0;
    p_tx->packet_index = 0;
    p_tx->is_sending = 1;

    return 0;  // 开始发送
}

uint8_t CAN_CustomTx_Process(CAN_CustomTx_t *p_tx)
{
    // 参数校验
    if (p_tx == NULL || !p_tx->is_sending)
    {
        return 0;  // 发送完成或未发送
    }

    // 计算剩余数据大小
    uint16_t remaining_size = p_tx->data_size - p_tx->sent_size;

    // 如果没有剩余数据，发送完成
    if (remaining_size == 0)
    {
        p_tx->is_sending = 0;
        return 0;  // 发送完成
    }

    // 准备协议头
    CAN_PacketHeader_t header;
    header.index = p_tx->packet_index;

    // 计算当前包的数据大小
    uint8_t payload_size = (remaining_size > CAN_CUSTOM_COMM_MAX_PAYLOAD) ? 
                           CAN_CUSTOM_COMM_MAX_PAYLOAD : remaining_size;

    // 设置最后包标志
    header.last = (remaining_size <= CAN_CUSTOM_COMM_MAX_PAYLOAD) ? 1 : 0;

    // 填充发送缓冲区
    tx_buffer[0] = _PackHeader(&header);
    memcpy(&tx_buffer[1], &p_tx->p_data[p_tx->sent_size], payload_size);

    // 如果不是最后一个包，填充剩余字节为0
    if (payload_size < CAN_CUSTOM_COMM_MAX_PAYLOAD)
    {
        memset(&tx_buffer[1 + payload_size], 0, CAN_CUSTOM_COMM_MAX_PAYLOAD - payload_size);
    }

    // 发送数据包
    if (BSP_CAN_Transmit(&p_tx->can_tx))
    {
        // 发送成功，更新状态
        p_tx->sent_size += payload_size;
        p_tx->packet_index++;

        // 如果是最后一个包，发送完成
        if (header.last)
        {
            p_tx->is_sending = 0;
            return 0;  // 发送完成
        }

        return 1;  // 发送中
    }

    // 发送失败，继续尝试
    return 1;  // 发送中
}

// ==================== 接收功能实现 ====================

void CAN_CustomRx_Init(CAN_CustomRx_t *p_rx, CAN_HandleTypeDef *hcan,
                       uint32_t rx_id, uint32_t IDE,
                       void *p_struct, uint16_t struct_size,
                       void (*unpack_func)(uint8_t *, uint16_t, void *),
                       void (*recv_complete_callback)(void *))
{
    // 参数校验
    if (p_rx == NULL || p_struct == NULL || unpack_func == NULL)
    {
        return;
    }

    // 保存参数
    p_rx->p_recv_buf = (uint8_t *)p_struct;
    p_rx->recv_buf_size = struct_size;
    p_rx->unpack_func = unpack_func;
    p_rx->recv_complete_callback = recv_complete_callback;

    // 初始化接收状态
    p_rx->recv_size = 0;
    p_rx->packet_index = 0;
    p_rx->is_receiving = 0;

    // 注册BSP CAN接收实例
    BSP_CAN_RxRegister(&p_rx->can_rx, hcan, rx_id, IDE, p_rx, CAN_CustomRx_Callback);
}

void CAN_CustomRx_Callback(BSP_CAN_RxInstance *p_rx_instance)
{
    // 从接收实例中获取自定义接收实例指针
    CAN_CustomRx_t *p_rx = (CAN_CustomRx_t *)p_rx_instance->p_owner_moudle;

    // 参数校验
    if (p_rx == NULL)
    {
        return;
    }

    // 解包协议头
    CAN_PacketHeader_t header;
    _UnpackHeader(p_rx_instance->rx_buff[0], &header);

    // 如果是新的数据包（索引为0），重置接收状态
    if (header.index == 0)
    {
        p_rx->recv_size = 0;
        p_rx->packet_index = 0;
        p_rx->is_receiving = 1;
    }

    // 检查索引是否匹配
    if (header.index != p_rx->packet_index)
    {
        // 索引不匹配，可能是丢包或乱序，重置接收状态
        p_rx->recv_size = 0;
        p_rx->packet_index = 0;
        p_rx->is_receiving = 0;
        return;
    }

    // 计算当前包的数据大小
    uint8_t payload_size = (header.last) ? 
                           (p_rx_instance->rx_len - 1) : CAN_CUSTOM_COMM_MAX_PAYLOAD;

    // 检查接收缓冲区是否足够
    if (p_rx->recv_size + payload_size > p_rx->recv_buf_size)
    {
        // 缓冲区不足，重置接收状态
        p_rx->recv_size = 0;
        p_rx->packet_index = 0;
        p_rx->is_receiving = 0;
        return;
    }

    // 将数据复制到接收缓冲区
    memcpy(&p_rx->p_recv_buf[p_rx->recv_size], &p_rx_instance->rx_buff[1], payload_size);
    p_rx->recv_size += payload_size;
    p_rx->packet_index++;

    // 如果是最后一个包，调用解包函数和完成回调
    if (header.last)
    {
        p_rx->is_receiving = 0;

        // 调用解包函数将接收缓冲区数据解包到结构体
        if (p_rx->unpack_func != NULL)
        {
            p_rx->unpack_func(p_rx->p_recv_buf, p_rx->recv_size, p_rx->p_recv_buf);
        }

        // 调用接收完成回调
        if (p_rx->recv_complete_callback != NULL)
        {
            p_rx->recv_complete_callback(p_rx->p_recv_buf);
        }
    }
}

uint16_t CAN_CustomRx_GetRecvSize(CAN_CustomRx_t *p_rx)
{
    if (p_rx == NULL)
    {
        return 0;
    }
    return p_rx->recv_size;
}

