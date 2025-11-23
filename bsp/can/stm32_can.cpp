/*
 * @Author       : Notch-FGJ mail.fgj.com@gmail.com
 * @Date         : 2025-03-30 22:11:35
 * @LastEditors  : Notch-FGJ mail.fgj.com@gmail.com
 * @LastEditTime : 2025-04-15 15:46:20
 * @FilePath     : \FrameworkA_FGJ\bsp\can\stm32_can.cpp
 * @Description  : CAN总线封装
 */
#include "stm32_can.hpp"
#include "bsp/log/log.hpp"
#include "bsp/system/time.hpp"
#include "cmsis_os2.h"

class STM32CAN : public ICAN
{
public:
    STM32CAN(CAN_HandleTypeDef* _hcan) : hcan_(_hcan)
    {
        if (hcan_->Instance == CAN1)  // CAN1 - FIFO0
            fifo_ = CAN_RX_FIFO0;
        else if (hcan_->Instance == CAN2)  // CAN2 - FIFO1
            fifo_ = CAN_RX_FIFO1;
        else
            while (true)
            {
                LOGERROR("CAN", "CAN Instance Error");
                osDelay(1000);
            }
        if (fifo_ == CAN_RX_FIFO0)
            HAL_CAN_ActivateNotification(hcan_, CAN_IT_RX_FIFO0_MSG_PENDING);
        else
            HAL_CAN_ActivateNotification(hcan_, CAN_IT_RX_FIFO1_MSG_PENDING);
        CAN_FilterTypeDef can_filter;
        can_filter.FilterIdHigh         = 0;
        can_filter.FilterIdLow          = 0;
        can_filter.FilterMode           = CAN_FILTERMODE_IDMASK;
        can_filter.FilterScale          = CAN_FILTERSCALE_32BIT;
        can_filter.FilterMaskIdHigh     = 0;
        can_filter.FilterMaskIdLow      = 0;
        can_filter.FilterFIFOAssignment = fifo_;
        can_filter.FilterActivation     = ENABLE;
        can_filter.FilterBank           = 0;
        can_filter.SlaveStartFilterBank = 14;
        if (HAL_CAN_ConfigFilter(hcan_, &can_filter) != HAL_OK)
        {
            LOGERROR("CAN", "CAN Filter Config Failed");
            while (true)
            {
                osDelay(1000);
            }
        }
        if (HAL_CAN_Start(hcan_) != HAL_OK)
        {
            LOGERROR("CAN", "CAN Start Failed");
            while (true)
            {
                osDelay(1000);
            }
        }
        HAL_CAN_ActivateNotification(hcan_, CAN_IT_ERROR);
    }
    ~STM32CAN() override = default;
    ErrorCode transmit(const ClassicPacket& packet, uint32_t timeout = 1000) override
    {
        CAN_TxHeaderTypeDef tx_header;
        tx_header.DLC = sizeof(packet.data);
        switch (packet.type)
        {
            case Type::STANDARD:
                tx_header.IDE = CAN_ID_STD;
                tx_header.RTR = CAN_RTR_DATA;
                break;
            case Type::EXTENDED:
                tx_header.IDE = CAN_ID_EXT;
                tx_header.RTR = CAN_RTR_DATA;
                break;
            case Type::REMOTE_STANDARD:
                tx_header.IDE = CAN_ID_STD;
                tx_header.RTR = CAN_RTR_REMOTE;
                break;
            case Type::REMOTE_EXTENDED:
                tx_header.IDE = CAN_ID_EXT;
                tx_header.RTR = CAN_RTR_REMOTE;
                break;
            default: return ErrorCode::INVALID;
        }
        tx_header.StdId              = (packet.type == Type::EXTENDED) ? 0 : packet.id;
        tx_header.ExtId              = (packet.type == Type::EXTENDED) ? packet.id : 0;
        tx_header.TransmitGlobalTime = DISABLE;
        uint64_t start_time          = STM32TimeDWT::GetMilliseconds();
        while (HAL_CAN_GetTxMailboxesFreeLevel(hcan_) == 0)
        {
            if (STM32TimeDWT::GetMilliseconds() - start_time > timeout)
            {
                LOGWARNING("CAN", "Transmit Timeout");
                return ErrorCode::TIMEOUT;
            }
            osDelay(1);  // 等待发送邮箱空闲
        }
        if (HAL_CAN_AddTxMessage(hcan_, &tx_header, packet.data, &tx_mailbox_) != HAL_OK)
        {
            LOGERROR("CAN", "Transmit Failed");
            return ErrorCode::FAILED;
        }
        return ErrorCode::OK;
    }

    ErrorCode setRxCallback(uint16_t rx_id, std::function<void(const uint8_t*, const uint8_t)> callback) override
    {
        if (callback == nullptr)
        {
            LOGERROR("CAN", "Callback is nullptr");
            return ErrorCode::INVALID;
        }
        callbackMap[rx_id] = callback;
        updateRxFilter();
        LOGDEBUG("CAN", "Set Rx Callback: " + std::to_string(rx_id));
        return ErrorCode::OK;
    }

    std::unordered_map<uint16_t, std::function<void(const uint8_t*, const uint8_t)>> callbackMap;  ///< 回调函数映射表
private:
    // 仅存储STD_ID, 2个CAN共享28个过滤器, 每个过滤器2个ID(CAN2作为从机)
    void updateRxFilter()
    {
        if (callbackMap.size() > 28)
        {
            while (true)
            {
                LOGERROR("CAN", "CAN Filter Size Exceeded");
                osDelay(1000);
            }
        }
        CAN_FilterTypeDef can_filter;
        can_filter.FilterMode           = CAN_FILTERMODE_IDLIST;
        can_filter.FilterScale          = CAN_FILTERSCALE_16BIT;
        can_filter.SlaveStartFilterBank = 14;
        can_filter.FilterActivation     = ENABLE;
        if (fifo_ == CAN_RX_FIFO0)
        {
            can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
            can_filter.FilterBank           = 0;
        }
        else
        {
            can_filter.FilterFIFOAssignment = CAN_RX_FIFO1;
            can_filter.FilterBank           = 14;
        }
        auto it = callbackMap.begin();
        while (it != callbackMap.end())
        {
            uint16_t first_id       = it->first;
            uint16_t second_id      = (++it == callbackMap.end()) ? 0 : it->first;
            can_filter.FilterIdHigh = first_id << 5;
            can_filter.FilterIdLow  = second_id << 5;
            HAL_CAN_ConfigFilter(hcan_, &can_filter);
            can_filter.FilterBank++;
            if (it != callbackMap.end())
                ++it;
        }
        HAL_CAN_ActivateNotification(hcan_, CAN_IT_RX_FIFO0_MSG_PENDING);
        HAL_CAN_ActivateNotification(hcan_, CAN_IT_RX_FIFO1_MSG_PENDING);
    }

private:
    CAN_HandleTypeDef* hcan_;
    uint32_t           fifo_;
    uint32_t           tx_mailbox_;
};

std::shared_ptr<STM32CAN> __can_handle_1 = nullptr;
std::shared_ptr<STM32CAN> __can_handle_2 = nullptr;

CANHandle_t STM32CAN_GetInstance(CAN_HandleTypeDef* _hcan)
{
    auto& handle = (_hcan->Instance == CAN1) ? __can_handle_1 : __can_handle_2;
    return handle;
}

void STM32CAN_Init()
{
    extern CAN_HandleTypeDef hcan1;
    extern CAN_HandleTypeDef hcan2;
    if (__can_handle_1 == nullptr)
    {
        __can_handle_1 = std::make_shared<STM32CAN>(&hcan1);
        LOGINFO("CAN", "CAN1 Init Success");
    }
    if (__can_handle_2 == nullptr)
    {
        __can_handle_2 = std::make_shared<STM32CAN>(&hcan2);
        LOGINFO("CAN", "CAN2 Init Success");
    }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t             fifo0_buffer[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, fifo0_buffer) != HAL_OK)
    {
        LOGERROR("CAN", "Receive Failed");
        return;
    }
    auto it = __can_handle_1->callbackMap.find(rx_header.StdId);
    if (it != __can_handle_1->callbackMap.end())
    {
        it->second(fifo0_buffer, rx_header.DLC);
    }
}

extern "C" void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t             fifo1_buffer[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, fifo1_buffer) != HAL_OK)
    {
        LOGERROR("CAN", "Receive Failed");
        return;
    }
    auto it = __can_handle_2->callbackMap.find(rx_header.StdId);
    if (it != __can_handle_2->callbackMap.end())
    {
        it->second(fifo1_buffer, rx_header.DLC);
    }
}

extern "C" void HAL_CAN_ErrorCallback(CAN_HandleTypeDef* hcan)
{
    if (hcan->ErrorCode != HAL_CAN_ERROR_NONE)
    {
        LOGERROR("CAN", "CAN Error: " + std::to_string(hcan->ErrorCode));
    }
}