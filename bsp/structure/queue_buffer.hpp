/*
 * @Author       : Notch-FGJ mail.fgj.com@gmail.com
 * @Date         : 2025-04-02 21:59:49
 * @LastEditors  : Notch-FGJ mail.fgj.com@gmail.com
 * @LastEditTime : 2025-04-07 00:14:17
 * @FilePath     : \FrameworkA_FGJ\bsp\structure\queue_buffer.hpp
 * @Description  : 基于FreeRTOS队列的, 用于中断回调的缓冲区
 */
#pragma once
#include "FreeRTOS.h"
#include "queue.h"
#include <cstdint>
#include <cstring>
#include <memory>
#include <vector>

/**
 * @class QueueBuffer
 * @brief 基于FreeRTOS队列的缓冲区类
 * @tparam T 数据类型
 * @tparam BufferSize 缓冲区大小
 * @tparam QueueSize 队列大小
 * @note 该类用于在中断回调中发送数据到队列, 并在任务中接收数据
 */
template <typename T, size_t BufferSize, size_t QueueSize>
class QueueBuffer
{
public:
    /**
     * @struct Buffer
     * @brief 缓冲区结构体
     */
    struct Buffer
    {
        T      data[BufferSize];  ///< 数据缓冲区
        size_t length;            ///< 数据长度
    };
    /**
     * @brief 构造函数
     * @note 创建FreeRTOS队列
     */
    QueueBuffer() { queue = xQueueCreate(QueueSize, sizeof(Buffer)); }
    /**
     * @brief 析构函数
     * @note 删除FreeRTOS队列
     */
    ~QueueBuffer()
    {
        if (queue != nullptr)
        {
            vQueueDelete(queue);
            queue = nullptr;
        }
    }
    /**
     * @brief 发送数据到队列
     * @param data 数据指针
     * @param length 数据长度
     * @return true 发送成功
     * @return false 发送失败
     * @warning 大于BufferSize的数据将被截断
     */
    bool sendFromISR(const T* data, size_t length)
    {
        if (length > BufferSize)
        {
            length = BufferSize;  // 限制长度
        }

        Buffer& buffer = bufferPool[bufferIndex % QueueSize];
        bufferIndex++;

        std::memcpy(buffer.data, data, length);
        buffer.length = length;

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        bool       result                   = xQueueSendFromISR(queue, &buffer, &xHigherPriorityTaskWoken) == pdTRUE;
        if (xHigherPriorityTaskWoken != pdFALSE)
        {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
        return result;
    }
    /**
     * @brief 阻塞接收数据
     * @param timeout 超时时间
     * @return std::shared_ptr<std::vector<T>> 接收到的数据
     * @note 超时返回nullptr
     */
    std::shared_ptr<std::vector<T>> receive(uint32_t timeout = portMAX_DELAY)
    {
        Buffer buffer;
        if (xQueueReceive(queue, &buffer, timeout) == pdTRUE)
        {
            return std::make_shared<std::vector<T>>(buffer.data, buffer.data + buffer.length);
        }
        return nullptr;
    }

private:
    QueueHandle_t queue       = nullptr;
    size_t        bufferIndex = 0;
    Buffer        bufferPool[QueueSize];
};