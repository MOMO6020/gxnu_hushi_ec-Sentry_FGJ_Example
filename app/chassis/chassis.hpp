/*
 * @Author       : Notch-FGJ mail.fgj.com@gmail.com
 * @Date         : 2025-04-15 13:34:24
 * @LastEditors  : Notch-FGJ mail.fgj.com@gmail.com
 * @LastEditTime : 2025-04-22 14:24:25
 * @FilePath     : \FrameworkA_FGJ\app\chassis\chassis.hpp
 * @Description  : 底盘任务
 */
#pragma once
#include "app/app_def.hpp"
#include "bsp/log/log.hpp"
#include "modules/motor/DJIMotor/DJIMotor.hpp"
#include "modules/pid/PIDController.hpp"
#include "modules/umt/Message.hpp"
#include <cmath>
#include <format>
#include <memory>

#define HALF_TRACK_WIDTH (CHASSIS_WHEEL_LATERAL / 2.0f)  ///< 轮距的一半
#define HALF_WHEEL_BASE (CHASSIS_WHEEL_BASE / 2.0f)      ///< 轴距的一半
#define DEGREE_TO_RADIAN (M_PI / 180.0f)

//它通过订阅"chassis_cmd"指令接收的指令转电机转速
class Chassis
{
public:
    Chassis()
    {
        extern CAN_HandleTypeDef hcan1;
        motor_lf = std::make_shared<DJIMotor>(
            &hcan1, 1, MotorType::M3508,
            MotorPIDSetting{
                CloseloopType::SPEED_LOOP,
                CloseloopType::SPEED_LOOP,
                false,
                FeedbackType::INTERNAL,
                FeedbackType::INTERNAL,
            },
            MotorPID{
                .pid_speed_ = PIDController(
                    PIDConfig{4.5f, 0.0f, 0.0f, 16384.0f, 10.0f, PIDImprovement::PID_Integral_Limit, 4000.0f}),
            });
        motor_rf = std::make_shared<DJIMotor>(
            &hcan1, 2, MotorType::M3508,
            MotorPIDSetting{
                CloseloopType::SPEED_LOOP,
                CloseloopType::SPEED_LOOP,
                false,
                FeedbackType::INTERNAL,
                FeedbackType::INTERNAL,
            },
            MotorPID{
                .pid_speed_ = PIDController(
                    PIDConfig{4.5f, 0.0f, 0.0f, 16384.0f, 10.0f, PIDImprovement::PID_Integral_Limit, 4000.0f}),
            });
        motor_lb = std::make_shared<DJIMotor>(
            &hcan1, 3, MotorType::M3508,
            MotorPIDSetting{
                CloseloopType::SPEED_LOOP,
                CloseloopType::SPEED_LOOP,
                false,
                FeedbackType::INTERNAL,
                FeedbackType::INTERNAL,
            },
            MotorPID{
                .pid_speed_ = PIDController(
                    PIDConfig{4.5f, 0.0f, 0.0f, 16384.0f, 10.0f, PIDImprovement::PID_Integral_Limit, 4000.0f}),
            });
        motor_rb = std::make_shared<DJIMotor>(
            &hcan1, 4, MotorType::M3508,
            MotorPIDSetting{
                CloseloopType::SPEED_LOOP,
                CloseloopType::SPEED_LOOP,
                false,
                FeedbackType::INTERNAL,
                FeedbackType::INTERNAL,
            },
            MotorPID{
                .pid_speed_ = PIDController(
                    PIDConfig{4.5f, 0.0f, 0.0f, 16384.0f, 10.0f, PIDImprovement::PID_Integral_Limit, 4000.0f}),
            });
        chassis_cmd_sub_.bind("chassis_cmd");
    }

    void ChassisTask()
    {
        chassis_cmd cmd;
        try
        {
            cmd = chassis_cmd_sub_.pop();
            if (cmd.mode == chassis_mode::CHASSIS_STOP)
            {
                motor_lf->disable();
                motor_rf->disable();
                motor_lb->disable();
                motor_rb->disable();
            }
            else
            {
                motor_lf->enable();
                motor_rf->enable();
                motor_lb->enable();
                motor_rb->enable();
            }

            switch (cmd.mode)
            {
                case chassis_mode::CHASSIS_NO_FOLLOW: chassis_vr = 0; break;
                case chassis_mode::CHASSIS_ROTATE: chassis_vr = 100; break;
                case chassis_mode::CHASSIS_FOLLOW_GIMBAL:
                    chassis_vr = 0.5 * cmd.offset_angle * std::abs(cmd.offset_angle);  // 可用？
                default: break;
            }

            sin_theta  = std::sin(cmd.offset_angle * DEGREE_TO_RADIAN);
            cos_theta  = std::cos(cmd.offset_angle * DEGREE_TO_RADIAN);
            chassis_vx = cmd.vx * cos_theta - cmd.vy * sin_theta;
            chassis_vy = cmd.vx * sin_theta + cmd.vy * cos_theta;

            // MecanumCalculate();
            OmniCalculate();  // TODO: 需要根据底盘类型选择计算方式

            // TODO: 输出限制

            motor_lf->setRef(vt_lf);
            motor_rf->setRef(vt_rf);
            motor_lb->setRef(vt_lb);
            motor_rb->setRef(vt_rb);
        }
        catch (umt::MessageError& e)
        {
            vTaskDelay(1);
        }
    }

private:
    umt::Subscriber<chassis_cmd> chassis_cmd_sub_;
    std::shared_ptr<DJIMotor>    motor_lf = nullptr;
    std::shared_ptr<DJIMotor>    motor_rf = nullptr;
    std::shared_ptr<DJIMotor>    motor_lb = nullptr;
    std::shared_ptr<DJIMotor>    motor_rb = nullptr;

    float sin_theta  = 0;
    float cos_theta  = 0;
    float chassis_vr = 0;
    float chassis_vx = 0;
    float chassis_vy = 0;
    float vt_lf      = 0;
    float vt_rf      = 0;
    float vt_lb      = 0;
    float vt_rb      = 0;

#define LF_CENTER                                                                                                      \
    ((HALF_TRACK_WIDTH + CHASSIS_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CHASSIS_GIMBAL_OFFSET_Y) * DEGREE_TO_RADIAN)
#define RF_CENTER                                                                                                      \
    ((HALF_TRACK_WIDTH - CHASSIS_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CHASSIS_GIMBAL_OFFSET_Y) * DEGREE_TO_RADIAN)
#define LB_CENTER                                                                                                      \
    ((HALF_TRACK_WIDTH + CHASSIS_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CHASSIS_GIMBAL_OFFSET_Y) * DEGREE_TO_RADIAN)
#define RB_CENTER                                                                                                      \
    ((HALF_TRACK_WIDTH - CHASSIS_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CHASSIS_GIMBAL_OFFSET_Y) * DEGREE_TO_RADIAN)

    void MecanumCalculate()
    {
        vt_lf = chassis_vx + chassis_vy - chassis_vr * LF_CENTER;
        vt_rf = chassis_vx - chassis_vy - chassis_vr * RF_CENTER;
        vt_lb = -chassis_vx + chassis_vy - chassis_vr * LB_CENTER;
        vt_rb = -chassis_vx - chassis_vy - chassis_vr * RB_CENTER;
    }

    void OmniCalculate()
    {
        vt_lf = chassis_vx + chassis_vy + chassis_vr * LF_CENTER;
        vt_rf = chassis_vx - chassis_vy + chassis_vr * RF_CENTER;
        vt_lb = -chassis_vx - chassis_vy + chassis_vr * LB_CENTER;
        vt_rb = -chassis_vx + chassis_vy + chassis_vr * RB_CENTER;
    }
};