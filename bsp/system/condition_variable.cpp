/*
 * @Author       : Notch-FGJ mail.fgj.com@gmail.com
 * @Date         : 2025-04-01 11:48:03
 * @LastEditors  : Notch-FGJ mail.fgj.com@gmail.com
 * @LastEditTime : 2025-04-01 19:31:33
 * @FilePath     : \FrameworkC_CMake\bsp\system\condition_variable.cpp
 * @Description  : 条件变量, 基于FreeRTOS的计数信号量封装
 */
#include "condition_variable.hpp"
#include "time.hpp"
#include "queue.h"
#include "bsp/log/log.hpp"

ConditionVariable::ConditionVariable() : handle_(xSemaphoreCreateCounting(UINT32_MAX, 0)){}

ConditionVariable::~ConditionVariable()
{
    vSemaphoreDelete(handle_);
}

ErrorCode ConditionVariable::wait(lock_guard<Mutex> &lock, std::function<bool()> pred)
{
    return wait_for(lock, portMAX_DELAY, pred);
}

ErrorCode ConditionVariable::wait_for(lock_guard<Mutex> &lock, uint32_t timeout, std::function<bool()> pred)
{
    if (pred != nullptr)
    {
        while (!pred())
            if (wait_for(lock, timeout) != ErrorCode::OK)
                return ErrorCode::TIMEOUT;
        return ErrorCode::OK;
    }
    else
    {
        lock.unlock();
        if (xSemaphoreTake(handle_, timeout) == pdTRUE)
        {
            lock.lock();
            return ErrorCode::OK;
        }
        lock.lock();
        return ErrorCode::TIMEOUT;
    }
}

void ConditionVariable::notify()
{
    xSemaphoreGive(handle_);
}



void ConditionVariable::notifyAll()
{
    uint32_t start;
    STM32TimeDWT::GetDelta(start);
    while (xSemaphoreTake(handle_, 0) == pdFALSE)
    {
        if (STM32TimeDWT::GetDelta(start, false) > 500)
            break;
        notify();
        LOGDEBUG("ConditionVariable", "Notify All");
        portYIELD();
    }
    LOGDEBUG("ConditionVariable", "Notify All Done");
}