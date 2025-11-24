#include "app_main.h"
#include "app/chassis/chassis.hpp"
#include "app/robotcmd/robotcmd.hpp"
#include "bsp/system/mutex.hpp"
#include "cmsis_os.h"

#include "bsp/log/log.hpp"
#include "bsp/system/time.hpp"

#include "gimbal/gimbal.hpp"
#include "modules/imu/ins_task.h"
#include "modules/motor/motor.hpp"
#include <memory>

std::shared_ptr<float>    gimbal_yaw_motor_angle_ptr = nullptr;
std::shared_ptr<RobotCMD> robotcmd                   = nullptr;
std::shared_ptr<Chassis>  chassis                    = nullptr;
std::shared_ptr<Gimbal>   gimbal                     = nullptr;
attitude_t*               imu_data                   = nullptr;

void RobotCMDTask(void* arg __attribute__((unused)));
void ChassisTask(void* arg __attribute__((unused)));
void INSTask(void* arg __attribute__((unused)));
void GimbalTask(void* arg __attribute__((unused)));

void app_main()
{
    taskENTER_CRITICAL();
    RTTLog::init();
    STM32TimeDWT::DWT_Init(168);
    LOGINFO("Robot", "System Init");
    STM32CAN_Init();
    StartDaemonTask();
    StartMotorControlTask();

    gimbal_yaw_motor_angle_ptr = std::make_shared<float>(0.0f);
    chassis                    = std::make_shared<Chassis>();
    imu_data                   = INS_Init();
    robotcmd                   = std::make_shared<RobotCMD>(imu_data, gimbal_yaw_motor_angle_ptr);
    gimbal                     = std::make_shared<Gimbal>(imu_data, gimbal_yaw_motor_angle_ptr);

    xTaskCreate(RobotCMDTask, "RobotCMDTask", 1024, nullptr, osPriorityNormal, nullptr);
    xTaskCreate(ChassisTask, "ChassisTask", 1024, nullptr, osPriorityNormal, nullptr);
    xTaskCreate(INSTask, "INSTask", 1024, nullptr, osPriorityNormal, nullptr);
    xTaskCreate(GimbalTask, "GimbalTask", 1024, nullptr, osPriorityNormal, nullptr);

    taskEXIT_CRITICAL();
    LOGINFO("Robot", "Robot Init");
}

void RobotCMDTask(void* arg __attribute__((unused)))
{
    while (true)
    {
        robotcmd->RobotCMDTask();
        vTaskDelay(2 / portTICK_PERIOD_MS);
    }
}

void ChassisTask(void* arg __attribute__((unused)))
{
    while (true)
    {
        chassis->ChassisTask();
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
}

void INSTask(void* arg __attribute__((unused)))
{
    while (true)
    {
        INS_Task();
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void GimbalTask(void* arg __attribute__((unused)))
{
    while (true)
    {
        gimbal->GimbalTask();
        vTaskDelay(2 / portTICK_PERIOD_MS);
    }
}
