/*
 * @Author       : Notch-FGJ mail.fgj.com@gmail.com
 * @Date         : 2025-04-04 16:07:45
 * @LastEditors  : Notch-FGJ mail.fgj.com@gmail.com
 * @LastEditTime : 2025-04-04 16:08:04
 * @FilePath     : \FramworkA\bsp\uart\IUart.hpp
 * @Description  : 通用异步收发传输（UART）基类
 */
#pragma once
#include "bsp/base_def.hpp"
#include "bsp/structure/queue_buffer.hpp"
#include <memory>
#include <vector>

/**
 * @class IUart
 * @brief 通用异步收发传输（UART）基类
 */
class IUart
{
public:
    virtual ~IUart() = default;
    virtual ErrorCode transmit(uint8_t *buf, uint16_t len) = 0;
    /**
     * @brief 尝试从接收队列中获取数据, 如果没有数据则阻塞等待
     * @param timeout 超时时间 (ms)
     * @return 指向数据的智能指针
     * @retval `nullptr` 如果没有数据
     */
    virtual std::shared_ptr<std::vector<uint8_t>> receive(uint32_t timeout) = 0;
    virtual void rx_callback_handle(uint8_t *buf, uint32_t len) = 0;
protected:
    /// @brief 接收数据的队列缓冲区
    std::unique_ptr<QueueBuffer<uint8_t, USART_BUFFER_SIZE, USART_QUEUE_SIZE>> uart_rx_queue = nullptr;
};