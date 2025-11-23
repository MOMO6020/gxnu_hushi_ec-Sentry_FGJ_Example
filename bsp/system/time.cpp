/*
 * @Author       : Notch-FGJ mail.fgj.com@gmail.com
 * @Date         : 2025-04-01 13:13:13
 * @LastEditors  : Notch-FGJ mail.fgj.com@gmail.com
 * @LastEditTime : 2025-04-10 21:50:44
 * @FilePath     : \FrameworkA_FGJ\bsp\system\time.cpp
 * @Description  : 基于DWT的时间管理类实现
 */
#include "bsp/system/time.hpp"
#include "cmsis_os.h"
#include "main.h"
#include "time.hpp"

extern IWDG_HandleTypeDef hiwdg;

static uint32_t CYCCNT_LAST;       ///< 上次调用时的内核计数器值
static uint32_t CYCCNT_RountCount; ///< 内核计数器溢出次数
static uint32_t CPU_FREQ_Hz;       ///< CPU频率, 单位Hz
static uint32_t CPU_FREQ_Hz_ms;    ///< CPU频率, 单位Hz/ms
static uint32_t CPU_FREQ_Hz_us;    ///< CPU频率, 单位Hz/us

void STM32TimeDWT::DWT_Init(uint32_t CPU_Freq_mHz)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // 使能DWT外设
    DWT->CYCCNT = (uint32_t)0u;                     // DWT CYCCNT寄存器计数清0
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // 使能Cortex-M DWT CYCCNT寄存器
    CPU_FREQ_Hz = CPU_Freq_mHz * 1000000;           // 设置CPU频率
    CPU_FREQ_Hz_ms = CPU_FREQ_Hz / 1000;            // 设置CPU频率(毫秒)
    CPU_FREQ_Hz_us = CPU_FREQ_Hz / 1000000;         // 设置CPU频率(微秒)
    CYCCNT_RountCount = 0;                          // 初始化溢出次数
    DWT_CNT_Update();
}

uint64_t STM32TimeDWT::GetMicroseconds()
{
    DWT_CNT_Update(); // 更新DWT计数器
    volatile uint64_t CYCCNT64 = (uint64_t)CYCCNT_RountCount * (uint64_t)UINT32_MAX + (uint64_t)CYCCNT_LAST; // 获取当前计数值
    return uint64_t(CYCCNT64 / (uint64_t)CPU_FREQ_Hz_us);
}

uint32_t STM32TimeDWT::GetDelta(uint32_t &cnt_last, bool update)
{
    DWT_CNT_Update(); // 更新DWT计数器
    uint32_t delta = (CYCCNT_LAST - cnt_last) / CPU_FREQ_Hz_ms; // 计算时间间隔
    if (update) {
        cnt_last = CYCCNT_LAST;
    }
    return delta;
}

uint32_t STM32TimeDWT::GetDeltaMicroseconds(uint32_t &cnt_last, bool update)
{
    DWT_CNT_Update(); // 更新DWT计数器
    uint32_t delta = (CYCCNT_LAST - cnt_last) / CPU_FREQ_Hz_us; // 计算时间间隔
    if (update) {
        cnt_last = CYCCNT_LAST;
    }
    return delta;
}

void STM32TimeDWT::Delay(uint32_t ms)
{
    HAL_IWDG_Refresh(&hiwdg); // 定期手动喂狗
    uint32_t cnt_last = DWT->CYCCNT; // 获取当前计数值
    while (DWT->CYCCNT - cnt_last < ms * CPU_FREQ_Hz_ms)
    {
        portYIELD();
    }
}

void STM32TimeDWT::DWT_CNT_Update()
{
    static volatile uint8_t bit_lock = 0;
    if (!bit_lock)
    {
        bit_lock = 1;
        volatile uint32_t cnt_now = DWT->CYCCNT; // 获取当前计数值
        if (cnt_now < CYCCNT_LAST)               // 检查是否溢出
            CYCCNT_RountCount++;                 // 溢出次数加1
        CYCCNT_LAST = cnt_now;                   // 更新上次计数值
        bit_lock = 0;
    }
}
