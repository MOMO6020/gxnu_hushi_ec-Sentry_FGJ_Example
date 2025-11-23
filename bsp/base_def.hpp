/*
 * @Author       : Notch-FGJ mail.fgj.com@gmail.com
 * @Date         : 2025-04-01 11:00:00
 * @LastEditors  : Notch-FGJ mail.fgj.com@gmail.com
 * @LastEditTime : 2025-04-11 16:15:51
 * @FilePath     : \FrameworkA_FGJ\bsp\base_def.hpp
 * @Description  : 基础定义文件, 包括常用的类型定义和宏定义
 */
#pragma once
#include <cstdint>
#include <type_traits>

#define MAX_DJIMOTOR_COUNT 12  ///< 最大大疆电机数量

#define DAEMON_PERIOD 10  ///< 守护进程周期(单位:ms)

#define USART_BUFFER_SIZE 256  ///< USART缓冲区大小(同时适用于虚拟串口)
#define USART_QUEUE_SIZE 6     ///< USART队列大小  (同时适用于虚拟串口)

#define MAX_DAEMON_NUM 32  ///< 最大守护进程数量

/**
 * @enum ErrorCode
 * @brief 错误码枚举
 */
enum class ErrorCode : int8_t
{
    OK      = 0,   ///< 成功
    FAILED  = -1,  ///< 失败
    BUSY    = -2,  ///< 锁已被占用
    TIMEOUT = -3,  ///< 超时
    INVALID = -4,  ///< 无效参数
    // TODO: 添加更多错误码
};

template <typename Enum>
constexpr auto operator|(Enum a, Enum b) -> std::underlying_type_t<Enum>
{
    return static_cast<std::underlying_type_t<Enum>>(a) | static_cast<std::underlying_type_t<Enum>>(b);
}

template <typename Enum>
constexpr auto operator&(Enum a, Enum b) -> std::underlying_type_t<Enum>
{
    return static_cast<std::underlying_type_t<Enum>>(a) & static_cast<std::underlying_type_t<Enum>>(b);
}