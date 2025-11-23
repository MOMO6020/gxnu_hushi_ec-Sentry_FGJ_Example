/*
 * @Author       : Notch-FGJ mail.fgj.com@gmail.com
 * @Date         : 2025-04-08 13:00:44
 * @LastEditors  : Notch-FGJ mail.fgj.com@gmail.com
 * @LastEditTime : 2025-06-24 19:32:53
 * @FilePath     : \gxnu_hushi_ec\modules\daemon\daemon.cpp
 * @Description  :
 */
#include "daemon.hpp"
#include "cmsis_os2.h"
#include <bsp/log/log.hpp>
#include <main.h>
#include <vector>

class Daemon : public IDaemon
{
public:
    Daemon(uint16_t timeout_cnt, uint16_t initial_cnt, std::function<void()> callback)
        : timeout_cnt_(timeout_cnt), current_cnt_(initial_cnt), callback_(callback)
    {
    }

    void setCallback(std::function<void()> callback) override { callback_ = callback; }

    void feed() override { current_cnt_ = timeout_cnt_; }

    void check()
    {
        if (current_cnt_ > 0)
            --current_cnt_;
        else if (callback_)
            callback_();
    }

private:
    uint16_t              timeout_cnt_;
    uint16_t              current_cnt_;
    std::function<void()> callback_;
};

extern IWDG_HandleTypeDef            hiwdg;
std::vector<std::shared_ptr<Daemon>> daemon_list_;

void daemonTask(void* arg __attribute__((unused)))
{
    LOGINFO("Daemon", "Daemon task started");
    while (true)
    {
        // HAL_IWDG_Refresh(&hiwdg);
        for (auto& daemon : daemon_list_)
            daemon->check();
        osDelay(DAEMON_PERIOD);
    }
}

void StartDaemonTask()
{
    // xTaskCreate(daemonTask, "DaemonTask", 512, nullptr, osPriorityNormal, nullptr);
    osThreadAttr_t attr = {"DaemonTask", 0, NULL, 0, NULL, 512, osPriorityAboveNormal, 0, 0};
    if (osThreadNew(daemonTask, nullptr, &attr) == nullptr)
    {
        LOGERROR("Daemon", "Failed to create daemon task");
        while (true)
            osDelay(1000);
    }
}

DaemonPtr registerDaemon(uint16_t timeout_cnt, uint16_t initial_cnt, std::function<void()> callback)
{
    auto daemon = std::make_shared<Daemon>(timeout_cnt, initial_cnt, callback);
    daemon_list_.push_back(daemon);
    return daemon;
}