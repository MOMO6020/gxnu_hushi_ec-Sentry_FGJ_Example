/*
 * @Author       : Notch-FGJ mail.fgj.com@gmail.com
 * @Date         : 2025-04-04 18:20:05
 * @LastEditors  : Notch-FGJ mail.fgj.com@gmail.com
 * @LastEditTime : 2025-04-07 13:16:15
 * @FilePath     : \FrameworkA_FGJ\bsp\uart\stm32_uart.hpp
 * @Description  : 通用异步收发传输
 */
#pragma once
#include "IUart.hpp"
#include "main.h"
#include <functional>
#include <memory>
#include <cstdint>

using UARTHandle_t = std::shared_ptr<IUart>;
UARTHandle_t STM32UART_Init(UART_HandleTypeDef *huard, std::function<void(uint8_t *buf, uint32_t len)> rx_callback = nullptr, std::function<void(uint8_t *buf, uint32_t len)> tx_callback = nullptr);