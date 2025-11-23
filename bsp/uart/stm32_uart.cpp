/*
 * @Author       : Notch-FGJ mail.fgj.com@gmail.com
 * @Date         : 2025-04-04 18:26:03
 * @LastEditors  : Notch-FGJ mail.fgj.com@gmail.com
 * @LastEditTime : 2025-04-07 13:29:07
 * @FilePath     : \FrameworkA_FGJ\bsp\uart\stm32_uart.cpp
 * @Description  :
 */
#include "stm32_uart.hpp"
#include "bsp/log/log.hpp"
#include <unordered_map>

class STM32UART : public IUart
{
public:
    STM32UART(UART_HandleTypeDef*                             huard,
              std::function<void(uint8_t* buf, uint32_t len)> rx_callback,
              std::function<void(uint8_t* buf, uint32_t len)> tx_callback);
    ErrorCode                             transmit(uint8_t* buf, uint16_t len) override;
    void                                  rx_callback_handle(uint8_t* buf, uint32_t len) override;
    void                                  rx_callback_handle(uint16_t size);
    std::shared_ptr<std::vector<uint8_t>> receive(uint32_t timeout = 100) override;
    std::vector<uint8_t>                  rx_buffer_;

private:
    UART_HandleTypeDef*                             huart_       = nullptr;  ///< UART句柄
    std::function<void(uint8_t* buf, uint32_t len)> tx_callback_ = nullptr;  ///< 发送回调函数
    std::function<void(uint8_t* buf, uint32_t len)> rx_callback_ = nullptr;  ///< 接收回调函数
};

STM32UART::STM32UART(UART_HandleTypeDef*                             huart,
                     std::function<void(uint8_t* buf, uint32_t len)> rx_callback,
                     std::function<void(uint8_t* buf, uint32_t len)> tx_callback)
    : huart_(huart), tx_callback_(tx_callback), rx_callback_(rx_callback)
{
    rx_buffer_.resize(USART_BUFFER_SIZE);
    uart_rx_queue = std::make_unique<QueueBuffer<uint8_t, USART_BUFFER_SIZE, USART_QUEUE_SIZE>>();
    if (HAL_UARTEx_ReceiveToIdle_DMA(huart_, rx_buffer_.data(), USART_BUFFER_SIZE) != HAL_OK)
        while (true)
        {
            LOGERROR("UART", "Failed to start DMA receive");
            HAL_Delay(1000);
        }
    __HAL_DMA_DISABLE_IT(huart_->hdmarx, DMA_IT_HT);
    LOGINFO("UART", "Init: " + std::to_string((uint32_t)huart_));
    if (rx_callback_ != nullptr)
    {
        LOGINFO("UART", "Init With RX Callback");
        uart_rx_queue = nullptr;  // 如果有回调函数，则不使用队列
    }
    if (tx_callback_ != nullptr)
        LOGINFO("UART", "Init With TX Callback");
}

ErrorCode STM32UART::transmit(uint8_t* buf, uint16_t len)
{
    if (len > USART_BUFFER_SIZE || buf == nullptr || len == 0)
        return ErrorCode::FAILED;
    auto ret = HAL_UART_Transmit_DMA(huart_, buf, len);
    if (ret == HAL_OK)
        return ErrorCode::OK;
    else if (ret == HAL_BUSY)
        return ErrorCode::BUSY;
    else
        return ErrorCode::FAILED;
}

void STM32UART::rx_callback_handle(uint8_t* buf, uint32_t len)
{
    if (len > 0 && buf != nullptr)
    {
        if (rx_callback_ != nullptr)
            rx_callback_(buf, len);
        else if (uart_rx_queue != nullptr)
            uart_rx_queue->sendFromISR(buf, len);
        else
            while (true)
            {
                LOGERROR("UART", "RX Callback Error: " + std::to_string(len));
                HAL_Delay(1000);
            }
    }
}

void STM32UART::rx_callback_handle(uint16_t len)
{
    rx_callback_handle(rx_buffer_.data(), len);
}

std::shared_ptr<std::vector<uint8_t>> STM32UART::receive(uint32_t timeout)
{
    if (uart_rx_queue == nullptr)
    {
        LOGERROR("UART", "Callback mode is enabled, receive is not supported");
        return nullptr;
    }
    return uart_rx_queue->receive(timeout);
}

std::unordered_map<UART_HandleTypeDef*, UARTHandle_t> uart_map;  ///< UART句柄与对象的映射表
UARTHandle_t                                          STM32UART_Init(UART_HandleTypeDef*                             huard,
                                                                     std::function<void(uint8_t* buf, uint32_t len)> rx_callback,
                                                                     std::function<void(uint8_t* buf, uint32_t len)> tx_callback)
{
    if (uart_map.find(huard) != uart_map.end())
    {
        return uart_map[huard];
    }
    auto uart       = std::make_shared<STM32UART>(huard, rx_callback, tx_callback);
    uart_map[huard] = uart;
    return uart;
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size)
{
    if (uart_map.find(huart) == uart_map.end())
        return;
    auto uart = std::dynamic_pointer_cast<STM32UART>(uart_map[huart]);
    uart->rx_callback_handle(size);
    memset(uart->rx_buffer_.data(), 0, USART_BUFFER_SIZE);
    HAL_UARTEx_ReceiveToIdle_DMA(huart, uart->rx_buffer_.data(), USART_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
    if (uart_map.find(huart) == uart_map.end())
        return;
    auto uart = std::dynamic_pointer_cast<STM32UART>(uart_map[huart]);
    LOGWARNING("UART", "Error: " + std::to_string(HAL_UART_GetError(huart)));
    HAL_UARTEx_ReceiveToIdle_DMA(huart, uart->rx_buffer_.data(), USART_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}