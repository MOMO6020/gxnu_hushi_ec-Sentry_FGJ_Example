/*
 * @Author       : Notch-FGJ mail.fgj.com@gmail.com
 * @Date         : 2025-04-01 10:49:55
 * @LastEditors  : Notch-FGJ mail.fgj.com@gmail.com
 * @LastEditTime : 2025-04-01 17:21:21
 * @FilePath     : \FrameworkC_CMake\bsp\system\mutex.hpp
 * @Description  : 互斥锁类, 基于FreeRTOS的信号量封装
 */
//? 已经过测试, 大概没有问题
#pragma once
#include "FreeRTOS.h"
#include "semphr.h"

#include "bsp/base_def.hpp"

/**
 * @class Mutex
 * @brief 互斥锁类, 提供线程同步机制
 * @details 该类实现了一个互斥锁, 用于确保线程安全, 支持加/解锁操作, 并对中断服务程序(ISR)进行特殊处理.
 */
class Mutex
{
public:
    /**
     * @brief 构造函数, 创建互斥锁
     */
    Mutex();
    /**
     * @brief 析构函数, 删除互斥锁
     */
    ~Mutex();
    /**
     * @brief 加锁, 如果锁已被占用, 则阻塞当前线程直到锁可用
     * @return 操作结果
     * @retval `ErrorCode::OK` 锁定成功
     */
    ErrorCode lock();

    /**
     * @brief 尝试加锁, 如果锁已被占用, 则立即返回
     * @return 操作结果
     * @retval `ErrorCode::OK` 锁定成功
     * @retval `ErrorCode::BUSY` 锁定失败, 锁已被占用
     */
    ErrorCode tryLock();
    
    /**
     * @brief 解锁互斥锁
     */
    void unlock();

    /**
     * @brief 在中断服务程序(ISR)中尝试加锁, 如果锁已被占用, 则立即返回
     * @param[in] in_isr 是否在中断服务程序中调用
     * @return 操作结果
     * @retval `ErrorCode::OK` 锁定成功
     * @retval `ErrorCode::BUSY` 锁定失败, 锁已被占用
     */
    ErrorCode trylockFromISR(bool in_isr);

    /**
     * @brief 在中断服务程序(ISR)中解锁互斥锁
     * @param[in] in_isr 是否在中断服务程序中调用
     */
    void unlockFromISR(bool in_isr);

private:
    SemaphoreHandle_t mutex_; ///< 互斥锁句柄
};


/**
 * @class lock_guard
 * @brief 互斥锁的 RAII 机制封装, 自动加锁和解锁
 * @details 该类实现了互斥锁的 RAII 机制, 在构造函数中加锁, 在析构函数中解锁.
 */
template <class Mutex>
class lock_guard
{

public:
    explicit lock_guard(Mutex& m) : mutex_(m) {
        mutex_.lock();
    }

    ~lock_guard() {
        mutex_.unlock();
    }

    ErrorCode lock() {
        return mutex_.lock();
    }

    void unlock() {
        mutex_.unlock();
    }

private:
    Mutex& mutex_;
    lock_guard(const lock_guard&) = delete;
    lock_guard& operator=(const lock_guard&) = delete;
};

/**
 * @class lock_guard_isr
 * @brief 互斥锁的 RAII 机制封装, 自动加锁和解锁, 用于中断服务程序(ISR)
 * @details 该类实现了互斥锁的 RAII 机制, 在构造函数中加锁, 在析构函数中解锁.
 */
template <class Mutex>
class lock_guard_isr
{
public:
    explicit lock_guard_isr(Mutex& m) : mutex_(m) {
        mutex_.trylockFromISR(true);
    }

    ~lock_guard_isr() {
        mutex_.unlockFromISR(true);
    }

private:
    Mutex& mutex_;
    lock_guard_isr(const lock_guard_isr&) = delete;
    lock_guard_isr& operator=(const lock_guard_isr&) = delete;
};