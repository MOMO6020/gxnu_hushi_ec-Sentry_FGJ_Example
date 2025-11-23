/*
 * @Author       : Notch-FGJ mail.fgj.com@gmail.com
 * @Date         : 2025-04-01 11:19:43
 * @LastEditors  : Notch-FGJ mail.fgj.com@gmail.com
 * @LastEditTime : 2025-04-01 11:33:47
 * @FilePath     : \FrameworkC_CMake\bsp\system\mutex.cpp
 * @Description  : 互斥锁类, 基于FreeRTOS的信号量封装
 */
#include "mutex.hpp"

Mutex::Mutex() : mutex_(xSemaphoreCreateBinary())
{
    unlock();
}

Mutex::~Mutex()
{
    vSemaphoreDelete(mutex_);
}

ErrorCode Mutex::lock()
{
    if (xSemaphoreTake(mutex_, portMAX_DELAY) != pdTRUE)
        return ErrorCode::FAILED;
    return ErrorCode::OK;
}

ErrorCode Mutex::tryLock()
{
    if (xSemaphoreTake(mutex_, 0) != pdTRUE)
        return ErrorCode::BUSY;
    return ErrorCode::OK;
}

void Mutex::unlock()
{
    xSemaphoreGive(mutex_);
}

ErrorCode Mutex::trylockFromISR(bool in_isr)
{
    if (in_isr)
    {
        BaseType_t px_higher_priority_task_woken = 0;
        BaseType_t x_return = xSemaphoreTakeFromISR(mutex_, &px_higher_priority_task_woken);
        if (x_return == pdPASS)
        {
            if (px_higher_priority_task_woken != pdFALSE)
                portYIELD_FROM_ISR(px_higher_priority_task_woken);
            return ErrorCode::OK;
        }
        else
            return ErrorCode::BUSY;
    }
    else
        return tryLock();
}

void Mutex::unlockFromISR(bool in_isr)
{
    if (in_isr)
    {
        BaseType_t px_higher_priority_task_woken = 0;
        xSemaphoreGiveFromISR(mutex_, &px_higher_priority_task_woken);
        if (px_higher_priority_task_woken != pdFALSE)
            portYIELD_FROM_ISR(px_higher_priority_task_woken);
    }
    else
        unlock();
}