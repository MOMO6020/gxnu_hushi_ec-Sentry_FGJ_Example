/*
 * @Author       : Notch-FGJ mail.fgj.com@gmail.com
 * @Date         : 2025-04-02 15:47:41
 * @LastEditors  : Notch-FGJ mail.fgj.com@gmail.com
 * @LastEditTime : 2025-04-04 18:53:49
 * @FilePath     : \FramworkA\bsp\usb\usb.cpp
 * @Description  : 
 */
#include "usb.hpp"
#include <bsp/log/log.hpp>
#include <cstring>
#include <usb_device.h>

#include "usbd_cdc_if.h"
#include <functional>

extern USBD_HandleTypeDef hUsbDeviceFS;
extern uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];
extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
extern uint8_t *CDC_Init(std::function<void(uint8_t *, uint32_t)> _rx_callback, std::function<void(uint8_t *, uint32_t)> _tx_callback);

class STM32USB : public IUart
{
public:
    STM32USB(std::function<void(uint8_t *buf, uint32_t len)> tx_callback = nullptr);
    ErrorCode transmit(uint8_t *buf, uint16_t len) override;
    void rx_callback_handle(uint8_t *buf, uint32_t len) override;
    std::shared_ptr<std::vector<uint8_t>> receive(uint32_t timeout = 100) override;

private:
    USBD_HandleTypeDef hUsbDeviceFS_;
};

STM32USB::STM32USB(std::function<void(uint8_t *buf, uint32_t len)> tx_callback_): hUsbDeviceFS_(hUsbDeviceFS)
{
    USBD_CDC_SetTxBuffer(&hUsbDeviceFS_, UserTxBufferFS, 0);
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS_, UserRxBufferFS);
    CDC_Init(std::bind(&STM32USB::rx_callback_handle, this, std::placeholders::_1, std::placeholders::_2),
             tx_callback_);
    uart_rx_queue = std::make_unique<QueueBuffer<uint8_t, USART_BUFFER_SIZE, USART_QUEUE_SIZE>>();
    LOGINFO("STM32USB", "USB Init Success!");
}

ErrorCode STM32USB::transmit(uint8_t *buf, uint16_t len)
{
    if (CDC_Transmit_FS(buf, len) == USBD_OK)
        return ErrorCode::OK;
    else
        return ErrorCode::BUSY;
}

void STM32USB::rx_callback_handle(uint8_t *buf, uint32_t len)
{
    if (len > 0 && buf != nullptr)
    {
        uart_rx_queue->sendFromISR(buf, len);
    }
}

std::shared_ptr<std::vector<uint8_t>> STM32USB::receive(uint32_t timeout_us)
{
    return uart_rx_queue->receive(timeout_us / 1000);
}

std::shared_ptr<IUart> usb_uart_instance = nullptr;

std::shared_ptr<IUart> STM32USB_Init(std::function<void(uint8_t *buf, uint32_t len)> tx_callback)
{
    if (usb_uart_instance)
        return usb_uart_instance;
    usb_uart_instance = std::make_shared<STM32USB>(tx_callback);
    return usb_uart_instance;
}
