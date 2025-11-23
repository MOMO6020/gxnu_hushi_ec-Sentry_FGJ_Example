/*
 * @Author       : Notch-FGJ mail.fgj.com@gmail.com
 * @Date         : 2025-03-30 21:26:02
 * @LastEditors  : Notch-FGJ mail.fgj.com@gmail.com
 * @LastEditTime : 2025-04-22 14:30:52
 * @FilePath     : \FrameworkA_FGJ\bsp\can\stm32_can.hpp
 * @Description  : CAN总线板级支持包, 仅针对C板
 */
#pragma once
#include "main.h"

#include "bsp/base_def.hpp"
#include <functional>
#include <memory>

class ICAN
{
public:
    /**
     * @enum Type
     * @brief CAN消息类型
     */
    enum class Type : uint8_t
    {
        STANDARD = 0,        ///< 标准帧
        EXTENDED = 1,        ///< 扩展帧
        REMOTE_STANDARD = 2, ///< 远程标准帧
        REMOTE_EXTENDED = 3, ///< 远程扩展帧
    };

    /**
     * @struct ClassicPacket
     * @brief 经典CAN数据包结构体
     * @details 该结构体用于存储经典CAN数据包的ID、类型和数据载荷
     */
    struct ClassicPacket
    {
        uint32_t id = 0;     ///< 消息ID
        Type type = Type::STANDARD;       ///< 消息类型
        uint8_t data[8]; ///< 数据载荷
    };

    virtual ~ICAN() = default;

    /**
     * @brief 将CAN消息添加到发送队列
     * @param packet 要发送的数据包
     * @param timeout 超时时间(ms)
     * @return 操作结果
     */
    virtual ErrorCode transmit(const ClassicPacket &packet, uint32_t timeout = 1000) = 0;

    /**
     * @brief 添加对应RX_ID的回调函数, 并将RX_ID添加到CAN_Filter中
     * @param rx_id 接收ID
     * @param callback 回调函数
     */
    virtual ErrorCode setRxCallback(uint16_t rx_id, std::function<void(const uint8_t *, const uint8_t)> callback) = 0;
};

using CANHandle_t = std::shared_ptr<ICAN>;
/**
 * @brief STM32CAN初始化函数, 若已存在与map中, 则返回该对象
 * @param _hcan CAN句柄
 * @param _rx_topic 接收主题
 * @param _queue_size 队列大小
 * @return CANHandle_t STM32CAN对象的智能指针
 */
CANHandle_t STM32CAN_GetInstance(CAN_HandleTypeDef *_hcan);

void STM32CAN_Init();