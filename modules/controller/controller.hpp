/*
 * @Author       : Notch-FGJ mail.fgj.com@gmail.com
 * @Date         : 2025-04-07 11:56:33
 * @LastEditors  : Notch-FGJ mail.fgj.com@gmail.com
 * @LastEditTime : 2025-04-11 23:08:14
 * @FilePath     : \FrameworkA_FGJ\modules\controller\controller.hpp
 * @Description  : DR16控制器
 */
#pragma once
#include "bsp/log/log.hpp"
#include "bsp/uart/stm32_uart.hpp"
#include "modules/daemon/daemon.hpp"

//当前 Controller 负责获取并解析遥控器数据
class Controller
{
public:
    struct ControllerData
    {
        int16_t channel0; ///< 右摇杆X轴, 范围: -660 - 0 - 660（改左）
        int16_t channel1; ///< 右摇杆Y轴, 范围: -660 - 0 - 660
        int16_t channel2; ///< 左摇杆X轴, 范围: -660 - 0 - 660（改右边）
        int16_t channel3; ///< 左摇杆Y轴, 范围: -660 - 0 - 660
        uint8_t switch1;  ///< 左拨杆开关, 上: 1, 中: 3, 下: 2
        uint8_t switch2;  ///< 右拨杆开关, 上: 1, 中: 3, 下: 2
    };
    enum class Key : uint16_t
    {
        KEY_W = 0x01 << 0,
        KEY_S = 0x01 << 1,
        KEY_A = 0x01 << 2,
        KEY_D = 0x01 << 3,
        KEY_Q = 0x01 << 4,
        KEY_E = 0x01 << 5,
        KEY_SHIFT = 0x01 << 6,
        KEY_CTRL = 0x01 << 7,
    };
    struct MouseKeyboardData
    {
        int16_t x;            ///< 鼠标X轴, 范围: -32768 - 0 - 32767
        int16_t y;            ///< 鼠标Y轴, 范围: -32768 - 0 - 32767
        int16_t z;            ///< 鼠标Z轴, 范围: -32768 - 0 - 32767
        uint8_t left_button;  ///< 鼠标左键, 按下: 1, 未按下: 0
        uint8_t right_button; ///< 鼠标右键, 按下: 1, 未按下: 0
        Key keyboard;         ///< 键盘按键, 16位, 0: 未按下, 1: 按下
    };
    Controller(UART_HandleTypeDef *_huart)
    {
        uart = STM32UART_Init(_huart, std::bind(&Controller::decode, this, std::placeholders::_1, std::placeholders::_2));
        if (uart == nullptr)
        {
            LOGERROR("Controller", "Failed to initialize UART");
            while (true)
            {
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
        }
        daemon = registerDaemon(20, 20, std::bind(&Controller::disconnectCallback, this));
        LOGINFO("Controller", "Init Success!");
    }
    void disconnectCallback()
    {
        if (is_connected)
        {
            memset(&rc_data, 0, sizeof(rc_data));
            memset(&mouse_data, 0, sizeof(mouse_data));
            is_connected = false;
            LOGWARNING("Controller", "Disconnected!");
        }
    }
    void decode(uint8_t *buf, uint32_t len)
    {
        if (buf == nullptr || len < 8)
            return;
        if (!is_connected)
        {
            is_connected = true;
            LOGINFO("Controller", "Connected!");
        }
        daemon->feed();
        rc_data.channel2 = (((int16_t)buf[0] | ((int16_t)buf[1] << 8)) & 0x07FF) - 1024;
        rc_data.channel3 = ((((int16_t)buf[1] >> 3) | ((int16_t)buf[2] << 5)) & 0x07FF) - 1024;
        rc_data.channel0 = ((((int16_t)buf[2] >> 6) | ((int16_t)buf[3] << 2) |
                            ((int16_t)buf[4] << 10)) & 0x07FF) - 1024;
        rc_data.channel1 = ((((int16_t)buf[4] >> 1) | ((int16_t)buf[5] << 7)) & 0x07FF) - 1024;
        rc_data.switch1 = ((buf[5] >> 4) & 0x000C) >> 2;
        rc_data.switch2 = ((buf[5] >> 4) & 0x0003);
        mouse_data.x = ((int16_t)buf[6] | ((int16_t)buf[7] << 8));
        mouse_data.y = ((int16_t)buf[8] | ((int16_t)buf[9] << 8));
        mouse_data.z = ((int16_t)buf[10] | ((int16_t)buf[11] << 8));
        mouse_data.left_button = buf[12];
        mouse_data.right_button = buf[13];
        mouse_data.keyboard = static_cast<Key>(((int16_t)buf[14] | ((int16_t)buf[15] << 8)));
    }

    ControllerData rc_data;
    MouseKeyboardData mouse_data;
    bool is_connected = false;
private:
    UARTHandle_t uart = nullptr;
    DaemonPtr daemon = nullptr;
};
