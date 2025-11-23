/*
 * @Author       : Notch-FGJ mail.fgj.com@gmail.com
 * @Date         : 2025-04-01 11:35:21
 * @LastEditors  : Notch-FGJ mail.fgj.com@gmail.com
 * @LastEditTime : 2025-04-01 19:23:24
 * @FilePath     : \FrameworkC_CMake\bsp\system\condition_variable.hpp
 * @Description  : 条件变量, 基于FreeRTOS的计数信号量封装
 * @warning      : //!未经测试, 可能存在问题
 */
#pragma once
#include "FreeRTOS.h"
#include "semphr.h"
#include "bsp/base_def.hpp"
#include "mutex.hpp"
#include <functional>

/**
 * @class ConditionVariable
 * @brief 条件变量封装类
 * 
 * 该类提供了条件变量的封装, 实现线程间的同步机制
 * 允许一个线程等待某个条件的发生, 另一个线程通知该条件的发生.
 */
class ConditionVariable
{
public:
    /**
     * @brief 构造函数, 创建条件变量
     */
    ConditionVariable();

    /**
     * @brief 析构函数, 删除条件变量
     */
    ~ConditionVariable();

    // template <class Predicate>
    // void wait(std::unique_lock<std::mutex> &lock, Predicate pred);
    /**
     * @brief wait 导致当前线程阻塞直至条件变量被通知,或虚假唤醒发生.可以提供 pred 以检测虚假唤醒.
     * @param[in] lock 必须已经由调用线程锁定的锁
     * @param[in] pred 检查是否可以完成等待的谓词
     * @return 操作结果
     * @retval `ErrorCode::OK` 成功
     * @retval `ErrorCode::TIMEOUT` 超时(portMAX_DELAY)
     */
    ErrorCode wait(lock_guard<Mutex> &lock, std::function<bool()> pred = nullptr);

    /**
     * @brief wait_for 导致当前线程阻塞，直至条件变量被通知，超过指定的时长，或发生虚假唤醒。可以提供 pred 以检测虚假唤醒。
     * @param[in] lock 必须已经由调用线程锁定的锁
     * @param[in] timeout 可以等待的最长时长(ms)
     * @param[in] pred 检查是否可以完成等待的谓词
     * @return 操作结果
     * @retval `ErrorCode::OK` 成功
     * @retval `ErrorCode::TIMEOUT` 超时
     */
    ErrorCode wait_for(lock_guard<Mutex> &lock, uint32_t timeout, std::function<bool()> pred = nullptr);

    /**
     * @brief 发送信号唤醒一个等待线程
     */
    void notify();

    /**
     * @brief 通知所有条件变量, 唤醒所有等待的线程
     * @warning 避免使用 notify_all() 进行唤醒!
     */
    void notifyAll();
private:
    SemaphoreHandle_t handle_;
};