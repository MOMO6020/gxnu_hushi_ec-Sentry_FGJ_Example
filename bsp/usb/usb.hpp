/*
 * @Author       : Notch-FGJ mail.fgj.com@gmail.com
 * @Date         : 2025-04-02 15:47:37
 * @LastEditors  : Notch-FGJ mail.fgj.com@gmail.com
 * @LastEditTime : 2025-04-04 17:44:48
 * @FilePath     : \FramworkA\bsp\usb\usb.hpp
 * @Description  : CDC虚拟串口封装
 */
#pragma once
#include "bsp/uart/IUart.hpp"
#include <functional>
#include <memory>

using USBHandle_t = std::shared_ptr<IUart>;
USBHandle_t STM32USB_Init(std::function<void(uint8_t *buf, uint32_t len)> tx_callback = nullptr);