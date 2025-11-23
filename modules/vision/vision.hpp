#pragma once
#include "bsp/system/mutex.hpp"
#include "bsp/uart/stm32_uart.hpp"
#include "main.h"
#include "modules/daemon/daemon.hpp"
#include <FreeRTOS.h>
#include <memory>
#include <task.h>

class VisionCommand
{
public:
    enum class ShootMode : uint8_t
    {
        CRUISE = 0,  // 巡航
        FOLLOW = 1,  // 跟随不发弹
        COMMON = 2,  // 普通模式
    };

    struct RobotCmd
    {
        uint8_t header      = 0xA5;
        float   pitch_angle = 0;                                        // 单位：度
        float   yaw_angle   = 0;                                        // 单位：度
        float   pitch_speed = 0;                                        // 单位：弧度/s
        float   yaw_speed   = 0;                                        // 单位：弧度/s
        uint8_t distance    = 0;                                        // 计算公式 (int)(distance * 10)
        uint8_t shoot_mode  = static_cast<uint8_t>(ShootMode::CRUISE);  // 射击模式
                                                                        // TODO: 下面这部分逻辑需要修改
        uint8_t  game_progress       = 0;                               // 0未开始 4比赛中
        uint16_t shooter_barrel_heat = 0;                               // 热量
        uint16_t current_HP          = 0;                               // 当前血量
        uint8_t  lrc                 = 0;
    } __attribute__((packed));

    VisionCommand(UART_HandleTypeDef* uart_handle_);
    void decode(uint8_t* data_, size_t size_);
    void offlineCallback();

    QueueHandle_t queue;

private:
    RobotCmd     _cmd_buffer;
    UARTHandle_t _uart_handle  = nullptr;
    DaemonPtr    _daemon       = nullptr;
    bool         _is_connected = false;
};