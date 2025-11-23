#include "vision.hpp"
#include "bsp/log/log.hpp"
#include "bsp/uart/stm32_uart.hpp"
#include "modules/daemon/daemon.hpp"
#include <cstring>
#include <functional>
#include <memory>

VisionCommand::VisionCommand(UART_HandleTypeDef* uart_handle_)
{
    _uart_handle = STM32UART_Init(
        uart_handle_, std::bind(&VisionCommand::decode, this, std::placeholders::_1, std::placeholders::_2));
    _daemon = registerDaemon(10, 10, std::bind(&VisionCommand::offlineCallback, this));
    queue   = xQueueCreate(1, sizeof(RobotCmd));
}

void VisionCommand::decode(uint8_t* data_, size_t size_)
{
    if (data_ == nullptr || size_ < 25)
        return;
    std::memcpy(&_cmd_buffer, data_, 25);
    uint8_t lrc = 0;
    for (auto ptr = (uint8_t*)&_cmd_buffer.header; ptr < &_cmd_buffer.lrc; ptr++)
        lrc += *ptr;
    if (lrc != _cmd_buffer.lrc)
    {
        LOGWARNING("Vision", "LRC Error");
        return;
    }
    if (!_is_connected)
    {
        _is_connected = true;
        LOGINFO("Vision", "Online!");
    }
    _daemon->feed();
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(queue, &_cmd_buffer, &xHigherPriorityTaskWoken);
}

void VisionCommand::offlineCallback()
{
    if (_is_connected)
    {
        _is_connected = false;
        LOGWARNING("Vision", "Vision Offline");
    }
}