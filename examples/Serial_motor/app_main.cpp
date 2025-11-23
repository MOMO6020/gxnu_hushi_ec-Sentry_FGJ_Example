#include "app_main.h"
#include "FreeRTOS.h"
#include "bsp/can/stm32_can.hpp"
#include "main.h"
#include "modules/pid/PIDController.hpp"
#include "stm32f4xx_hal_can.h"
#include "task.h"

#include "bsp/log/log.hpp"
#include "bsp/system/time.hpp"
#include "bsp/uart/stm32_uart.hpp"
#include "modules/motor/DJIMotor/DJIMotor.hpp"
#include "modules/motor/motor.hpp"
#include "stm32f4xx_hal_uart.h"
#include <cstdlib>
#include <memory>
#include <string>

extern UART_HandleTypeDef huart1;
extern CAN_HandleTypeDef  hcan1;

UARTHandle_t              usart1  = nullptr;
std::shared_ptr<DJIMotor> motor   = nullptr;
int                       receive = 0;

void uart1_callback(uint8_t* buf, uint32_t len);
void uart_test(void* arg);
void motor_test(void* arg);
void app_main()
{
    __disable_irq();
    RTTLog::init();
    STM32TimeDWT::DWT_Init(168);
    STM32CAN_Init();
    StartMotorControlTask();

    usart1 = STM32UART_Init(&huart1, uart1_callback);
    motor  = std::make_shared<DJIMotor>(
        &hcan1, 1, MotorType::M2006,
        MotorPIDSetting{

             .outer_loop = CloseloopType::SPEED_LOOP,
             .close_loop = CloseloopType::SPEED_LOOP,
             .reverse    = false,
             .external_angle_feedback = FeedbackType::INTERNAL,
             .external_speed_feedback = FeedbackType::INTERNAL,
        },
        MotorPID{
             .pid_angle_ =
                PIDController(PIDConfig{4.0f, 0.0f, 0.0f, 500.0f, 3.0f, PIDImprovement::PID_Integral_Limit, 4000.0f}),
             .pid_speed_ =
                PIDController(PIDConfig{4.5f, 0.0f, 0.0f, 16384.0f, 3.0f, PIDImprovement::PID_Integral_Limit, 4000.0f}),
        });

    xTaskCreate(uart_test, "uart_test", 256, nullptr, 0, nullptr);
    xTaskCreate(motor_test, "motor_test", 2048, nullptr, 0, nullptr);

    __enable_irq();
}

void uart1_callback(uint8_t* buf, uint32_t len)
{
    if (buf != nullptr && len > 0)
    {
        receive = std::atoi((char*)buf);
    }
}

void uart_test(void* arg __attribute__((unused)))
{
    while (true)
    {
        std::string message = std::to_string(receive);
        usart1->transmit((uint8_t*)message.c_str(), message.length());
        vTaskDelay(1000);
    }
}

void motor_test(void* arg __attribute__((unused)))
{
    while (true)
    {
        motor->setRef(receive);
        vTaskDelay(5);
    }
}
