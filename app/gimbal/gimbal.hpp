#pragma once
#include "app/app_def.hpp"
#include "bsp/log/log.hpp"
#include "bsp/system/mutex.hpp"
#include "bsp/uart/stm32_uart.hpp"
#include "modules/imu/ins_task.h"
#include "modules/motor/DJIMotor/DJIMotor.hpp"
#include "modules/motor/DMMotor/DMMotor.hpp"
#include "modules/umt/Message.hpp"
#include "stm32f4xx_hal_uart.h"
#include <format>
#include <memory>

// For Test:
UARTHandle_t              uart6 = nullptr;
extern UART_HandleTypeDef huart6;

class Gimbal
{
public:
    Gimbal(attitude_t* imu_data, std::shared_ptr<float> gimbal_yaw_motor_angle_ptr) : imu_data_(imu_data)
    {
        uart6 = STM32UART_Init(&huart6);
        if (gimbal_yaw_motor_angle_ptr == nullptr)
        {
            while (true)
            {
                LOGERROR("Gimbal", "gimbal_yaw_motor_angle_ptr is nullptr");
                STM32TimeDWT::Delay(1000);
            }
        }
        gimbal_yaw_motor_angle_ptr_ = gimbal_yaw_motor_angle_ptr;
        extern CAN_HandleTypeDef hcan1;
        // motor_yaw_ = std::make_shared<DJIMotor>(
        //     &hcan1, 1, MotorType::GM6020,
        //     MotorPIDSetting{
        //         CloseloopType::ANGLE_LOOP,
        //         static_cast<CloseloopType>(CloseloopType::ANGLE_LOOP | CloseloopType::SPEED_LOOP),
        //         false,
        //         FeedbackType::EXTERNAL,
        //         FeedbackType::EXTERNAL,
        //     },
        //     MotorPID{
        //         .pid_angle_ = PIDController(
        //             PIDConfig{30.0f, 25.5f, 0.95f, 500.0f, 0.0f, PIDImprovement::PID_Integral_Limit, 200.0f}),
        //         .pid_speed_ = PIDController(
        //             PIDConfig{30.0f, 22.0f, 0.0f, 16384.0f, 0.0f, PIDImprovement::PID_Integral_Limit, 8000.0f}),
        //         .pid_angle_feedback_ptr_ = std::shared_ptr<float>(&imu_data_->YawTotalAngle),
        //         .pid_speed_feedback_ptr_ = std::shared_ptr<float>(&imu_data_->Gyro[2]),
        //     });
        motor_yaw_ = std::make_shared<DMMotor>(
            &hcan1, 0x001, 0x000, MotorType::M2006,
            MotorPIDSetting{
                .outer_loop              = CloseloopType::ANGLE_LOOP,
                .close_loop              = CloseloopType::ANGLE_AND_SPEED_LOOP,
                .reverse                 = false,
                .external_angle_feedback = FeedbackType::EXTERNAL,
                .external_speed_feedback = FeedbackType::EXTERNAL,
            },
            MotorPID{
                .pid_angle_ = PIDController(
                    PIDConfig{3.0f, 0.0f, 0.0f, 600.0f, 3.0f, PIDImprovement::PID_Integral_Limit, 4000.0f}),
                .pid_speed_ = PIDController(
                    PIDConfig{0.01f, 0.0f, 0.0f, 10.0f, 3.0f, PIDImprovement::PID_Integral_Limit, 4000.0f}),
                .pid_angle_feedback_ptr_ = std::shared_ptr<float>(&imu_data_->YawTotalAngle),
                .pid_speed_feedback_ptr_ = std::shared_ptr<float>(&imu_data_->Gyro[2]),
            });
        motor_pitch_ = std::make_shared<DJIMotor>(
            &hcan1, 2, MotorType::GM6020,
            MotorPIDSetting{
                CloseloopType::ANGLE_LOOP,
                static_cast<CloseloopType>(CloseloopType::ANGLE_LOOP | CloseloopType::SPEED_LOOP),
                false,
                FeedbackType::EXTERNAL,
                FeedbackType::EXTERNAL,
            },
            MotorPID{
                .pid_angle_ = PIDController(
                    PIDConfig{28.0f, 22.5f, 0.55f, 500.0f, 0.0f, PIDImprovement::PID_Integral_Limit, 200.0f}),
                .pid_speed_ = PIDController(
                    PIDConfig{30.0f, 22.0f, 0.0f, 16384.0f, 0.0f, PIDImprovement::PID_Integral_Limit, 8000.0f}),
                .pid_angle_feedback_ptr_ = std::shared_ptr<float>(&imu_data_->Pitch),
                .pid_speed_feedback_ptr_ = std::shared_ptr<float>(&imu_data_->Gyro[0]),
            });
        gimbal_cmd_sub_.bind("gimbal_cmd");
    }
    void GimbalTask()
    {
        try
        {
            gimbal_cmd cmd = gimbal_cmd_sub_.pop();
            if (cmd.force_stop)
            {
                motor_yaw_->disable();
                motor_pitch_->disable();
            }
            else
            {
                motor_yaw_->enable();
                motor_pitch_->enable();
                motor_yaw_->setRef(cmd.yaw);
                motor_pitch_->setRef(cmd.pitch);
            }
            *gimbal_yaw_motor_angle_ptr_ = motor_yaw_->measure_.angle;
            // LOGINFO("Gimbal", std::format("Yaw Encoder: {}", motor_yaw_->measure_.encoder));
            // taskENTER_CRITICAL();
            // LOGINFO("Gimbal",
            //         std::format("PIDOut:{:5}, PIDRef:{:3}, Temp:{:2}, Current:{:5}, Encoder:{:4}",
            //                     motor_pitch_->pid_out_, motor_pitch_->pid_ref_, motor_pitch_->measure_.temperature,
            //                     motor_pitch_->measure_.torque_current, motor_pitch_->measure_.encoder));
            // taskEXIT_CRITICAL();
        }
        catch (umt::MessageError& e)
        {
            vTaskDelay(1);
        }
    }

private:
    umt::Subscriber<gimbal_cmd> gimbal_cmd_sub_;
    std::shared_ptr<DMMotor>    motor_yaw_                  = nullptr;
    std::shared_ptr<DJIMotor>   motor_pitch_                = nullptr;
    std::shared_ptr<float>      gimbal_yaw_motor_angle_ptr_ = nullptr;
    attitude_t*                 imu_data_                   = nullptr;
};