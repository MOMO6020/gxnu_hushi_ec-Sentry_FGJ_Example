#include "motor.hpp"
#include "DJIMotor/DJIMotor.hpp"
#include "bsp/log/log.hpp"
#include "cmsis_os2.h"
#include "modules/motor/DMMotor/DMMotor.hpp"
#include <string.h>

void motor_control_task(void* arg __attribute__((unused)))
{
    LOGINFO("MotorControl", "Motor control task started");
    while (true)
    {
        // 调用所有类型电机的控制函数, 后续添加其他电机类型时, 只需在此处添加调用即可
        DJIMotor::DJIMotorControl();
        DMMotor::DMMotorControl();
        osDelay(1);
    }
}

void StartMotorControlTask()
{
    osThreadAttr_t attr = {
        .name       = "MotorControlTask",
        .stack_size = 1024 * 6,
        .priority   = osPriorityAboveNormal,
    };
    osThreadNew(motor_control_task, nullptr, &attr);
}