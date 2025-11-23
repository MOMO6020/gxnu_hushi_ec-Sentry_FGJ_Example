/*
 * @Author       : Notch-FGJ mail.fgj.com@gmail.com
 * @Date         : 2025-04-10 11:49:21
 * @LastEditors  : Notch-FGJ mail.fgj.com@gmail.com
 * @LastEditTime : 2025-04-10 11:51:39
 * @FilePath     : \FrameworkA_FGJ\bsp\system\counting_semaphore.hpp
 * @Description  : 信号量封装类, 基于FreeRTOS的信号量封装, 实现线程同步机制
 */
#pragma once
#include "bsp/base_def.hpp"
#include "FreeRTOS.h"
#include "semphr.h"
class CountingSemaphore
{
public:
    CountingSemaphore(uint32_t desired);
    ~CountingSemaphore();

    ErrorCode acquire(uint32_t timeout = 0);
    ErrorCode release();
    ErrorCode releaseFromISR(bool in_isr = true);

private:
    SemaphoreHandle_t handle_;
};