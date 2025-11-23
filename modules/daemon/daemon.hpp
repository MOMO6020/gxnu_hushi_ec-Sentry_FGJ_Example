/*
 * @Author       : Notch-FGJ mail.fgj.com@gmail.com
 * @Date         : 2025-04-08 11:56:51
 * @LastEditors  : Notch-FGJ mail.fgj.com@gmail.com
 * @LastEditTime : 2025-04-08 13:07:12
 * @FilePath     : \FrameworkA_FGJ\modules\daemon\daemon.hpp
 * @Description  : 守护进程, 用于监控系统的运行状态, 以及处理一些系统级别的任务
 */
#pragma once
#include "bsp/base_def.hpp"
#include <functional>
#include <memory>

/**
 * @class IDaemon
 * @brief 守护进程接口类
 * 在需要维护的对象内使用registerDaemon注册一个守护进程, 并设置需要的超时计数器,
 * 在每次该对象状态更新后进行喂狗操作以实现看门狗机制 可以在守护进程超时后执行回调函数, 例如重启系统或发送错误信息
 * 内部计数器`current_cnt_`将递减, 每次喂狗后`current_cnt_`将被重置为`timeout_cnt_`
 * 初始化时设置`initial_cnt`可以为`current_cnt_`设置一个初始值, 例如在系统启动时需要等待一段时间才能开始工作
 * @note 该类设置为抽象类仅仅是为了防止外部直接创建对象, 该类的实例化仅在registerDaemon函数中进行
 */
class IDaemon
{
public:
    virtual ~IDaemon()                                                = default;
    virtual void          setCallback(std::function<void()> callback) = 0;
    virtual void          feed()                                      = 0;
    static const uint16_t daemon_task_period_                         = DAEMON_PERIOD;
};

void StartDaemonTask();

using DaemonPtr = std::shared_ptr<IDaemon>;
/**
 * @brief 注册守护进程
 * @param[in] timeout_cnt 超时计数器, 每次喂狗将当前计数器重置为timeout_cnt
 * @param[in] initial_cnt 初始化计数器, 为current_cnt_设置一个初始值, 给系统一个缓冲时间
 * @param[in] callback 超时后执行的回调函数
 * @return `DaemonPtr` 返回一个守护进程指针
 * 该函数维护一个守护进程列表, 并在第一次调用时创建守护进程任务, 周期为宏定义 `DAEMON_PERIOD`
 * 任务中将遍历所有守护进程, 减少计数器并检查计数器是否到达0, 如果到达0则执行回调函数
 * 同时该任务将维护IWDG硬件看门狗
 */
DaemonPtr registerDaemon(uint16_t timeout_cnt, uint16_t initial_cnt, std::function<void()> callback);