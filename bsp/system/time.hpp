/*
 * @Author       : Notch-FGJ mail.fgj.com@gmail.com
 * @Date         : 2025-04-01 12:43:49
 * @LastEditors  : Notch-FGJ mail.fgj.com@gmail.com
 * @LastEditTime : 2025-04-10 20:44:10
 * @FilePath     : \FrameworkA_FGJ\bsp\system\time.hpp
 * @Description  : 基于DWT的时间管理类
 */
//? 经过测试, 大概没有问题
#pragma once
#include <cstdint>

/**
 * @class STM32TimeDWT
 * @brief 获取基于DWT的时间(秒/毫秒/微秒)
 *
 * DWT计时器是Cortex-M系列微控制器的一个硬件计时器, 它可以在不影响系统性能的情况下提供高精度的时间测量.
 * 不受中断/任务等因素影响, 适合用于高精度的时间测量.
 * DWT计时器需要保证两次调用之间的时间间隔不超过一次溢出, 否则会导致计时错误.
 */
class STM32TimeDWT
{
public:
    /**
     * @brief 初始化DWT外设
     * @param[in] CPU_Freq_mHz CPU频率, 单位MHz
     */
    static void DWT_Init(uint32_t CPU_Freq_mHz);

    static uint64_t GetMilliseconds()
    {
        return GetMicroseconds() / 1000;
    }

    /**
     * @brief 获取微秒级时间戳
     * @return 返回当前时间的微秒级时间戳
     */
    static uint64_t GetMicroseconds();

    /**
     * @brief 获取两次调用的时间间隔
     * @param[in,out] cnt_last 上一次调用的时间戳, 在函数返回时更新为当前时间戳
     * @param[in] update 是否更新cnt_last, 默认为true
     * @return 返回两次调用之间的时间间隔, 单位为毫秒
     * @warning 此函数假设两次调用之间的时间间隔不超过一次溢出(约10s)
     */
    static uint32_t GetDelta(uint32_t &cnt_last, bool update = true);

    /**
     * @brief 获取两次调用的时间间隔
     * @param[in,out] cnt_last 上一次调用的时间戳, 在函数返回时更新为当前时间戳
     * @param[in] update 是否更新cnt_last, 默认为true
     * @return 返回两次调用之间的时间间隔, 单位为微秒
     * @warning 此函数假设两次调用之间的时间间隔不超过一次溢出(约10s)
     */
    static uint32_t GetDeltaMicroseconds(uint32_t &cnt_last, bool update = true);


    /**
     * @brief 基于DWT的延时函数, 唯一允许在初始化临界区中使用的延时函数
     * @param ms 延迟的毫秒数
     * @warning 不允许过长的延迟, 否则会导致DWT计数器溢出
     */
    static void Delay(uint32_t ms);

private:
    /**
     * @brief 更新DWT计数器
     * @details 检查DWT CYCCNT寄存器是否溢出, 并更新CYCCNT_RountCount
     * @warning 此函数假设两次调用之间的时间间隔不超过一次溢出
     */
    static void DWT_CNT_Update();
};