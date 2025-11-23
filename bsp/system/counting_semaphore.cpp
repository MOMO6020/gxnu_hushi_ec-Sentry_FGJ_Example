/*
 * @Author       : Notch-FGJ mail.fgj.com@gmail.com
 * @Date         : 2025-04-10 11:54:00
 * @LastEditors  : Notch-FGJ mail.fgj.com@gmail.com
 * @LastEditTime : 2025-04-10 12:41:19
 * @FilePath     : \FrameworkA_FGJ\bsp\system\counting_semaphore.cpp
 * @Description  : 
 */
#include "counting_semaphore.hpp"

CountingSemaphore::CountingSemaphore(uint32_t desired)
    : handle_(xSemaphoreCreateCounting(UINT32_MAX, desired)) {}

CountingSemaphore::~CountingSemaphore()
{
    vSemaphoreDelete(handle_);
}

ErrorCode CountingSemaphore::acquire(uint32_t timeout)
{
    if (xSemaphoreTake(handle_, timeout) == pdTRUE)
        return ErrorCode::OK;
    else
        return ErrorCode::TIMEOUT;
}

ErrorCode CountingSemaphore::release()
{
    if (xSemaphoreGive(handle_) == pdTRUE)
        return ErrorCode::OK;
    else
        return ErrorCode::FAILED;
}

ErrorCode CountingSemaphore::releaseFromISR(bool in_isr)
{
    if (in_isr)
    {
        BaseType_t px_higher_priority_task_woken = 0;
        xSemaphoreGiveFromISR(handle_, &px_higher_priority_task_woken);
        if (px_higher_priority_task_woken != pdFALSE)
            portYIELD_FROM_ISR(px_higher_priority_task_woken);
        return ErrorCode::OK;
    }
    else
        return release();
}
