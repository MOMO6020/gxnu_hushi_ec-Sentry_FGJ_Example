/*
 * @Author       : Notch-FGJ mail.fgj.com@gmail.com
 * @Date         : 2025-04-11 21:11:16
 * @LastEditors  : Notch-FGJ mail.fgj.com@gmail.com
 * @LastEditTime : 2025-04-11 23:18:06
 * @FilePath     : \FrameworkA_FGJ\app\robotcmd\robotcmd.hpp
 * @Description  : 机器人控制中心, 所有的控制命令都在这里处理,
 * 并将控制命令分发到各个模块
 */
#pragma once
#include "app/app_def.hpp"
#include <cstdint>
#include <cstdlib>
#include <format>
#include <main.h>
#include <memory>

#include "bsp/log/log.hpp"
#include "bsp/system/mutex.hpp"
#include "bsp/system/time.hpp"

#include "modules/controller/controller.hpp"
#include "modules/imu/ins_task.h"
#include "modules/umt/umt.hpp"
#include "modules/vision/vision.hpp"
#include "usart.h"

#define YAW_ALIGEN_ANGLE (YAW_CHASSIS_ALIGEN_ENCODER * 360.0f / 8192.0f)  ///< 云台与地盘对其时电机编码器值对应的角度
#define PITCH_HORIZON_ANGLE (PITCH_HORIZON_ENCODER * 360.0f / 8192.0f)    ///< 云台水平时电机编码器值对应的角度

class RobotCMD
{
public:
    RobotCMD(attitude_t* imu_data, std::shared_ptr<float> gimbal_yaw_motor_angle_ptr)
        : _gimbal_yaw_motor_angle_ptr(gimbal_yaw_motor_angle_ptr), _controller(&huart3), imu_data_(imu_data)
    {
        if (_gimbal_yaw_motor_angle_ptr == nullptr)
        {
            while (true)
            {
                LOGERROR("RobotCMD", "gimbal_yaw_motor_angle_ptr_ is nullptr");
                STM32TimeDWT::Delay(1000);
            }
        }
        _vision = std::make_shared<VisionCommand>(&huart1);
        _gimbal_cmd_pub.bind("gimbal_cmd");
        _chassis_cmd_pub.bind("chassis_cmd");
    }

    void RobotCMDTask()
    {
        if (_controller.is_connected)
        {
            if (_controller.rc_data.switch1 == 1)  // 遥控器左侧开关[上], 遥控器控制
                rcControlSet();
            else if (_controller.rc_data.switch1 == 3)  // 遥控器左侧开关[中], 键盘控制
                mouseKeySet();
            else if (_controller.rc_data.switch1 == 2)  // 遥控器左侧开关[下], 视觉控制
                visionSet();
            chassis_cmd_msg.offset_angle = calculateOffsetAngle();
            _gimbal_cmd_pub.push(gimbal_cmd_msg);
            _chassis_cmd_pub.push(chassis_cmd_msg);
        }
        else
        {
            __asm volatile("nop");
        }
    }

private:
    umt::Publisher<gimbal_cmd>     _gimbal_cmd_pub;
    umt::Publisher<chassis_cmd>    _chassis_cmd_pub;
    std::shared_ptr<float>         _gimbal_yaw_motor_angle_ptr = nullptr;
    Controller                     _controller;
    std::shared_ptr<VisionCommand> _vision   = nullptr;
    attitude_t*                    imu_data_ = nullptr;

    gimbal_cmd  gimbal_cmd_msg;
    chassis_cmd chassis_cmd_msg;

    float calculateOffsetAngle()
    {
#if YAW_CHASSIS_ALIGEN_THAN_4096
        if (*_gimbal_yaw_motor_angle_ptr > YAW_ALIGEN_ANGLE &&
            *_gimbal_yaw_motor_angle_ptr <= (YAW_ALIGEN_ANGLE + 180.0f))
            return *_gimbal_yaw_motor_angle_ptr - YAW_ALIGEN_ANGLE;
        else if (*_gimbal_yaw_motor_angle_ptr > (YAW_ALIGEN_ANGLE + 180.0f))
            return *_gimbal_yaw_motor_angle_ptr - YAW_ALIGEN_ANGLE - 360.0f;
        else
            return *_gimbal_yaw_motor_angle_ptr - YAW_ALIGEN_ANGLE;
#else
        if (*gimbal_yaw_motor_angle_ptr_ > YAW_ALIGEN_ANGLE)
            return *gimbal_yaw_motor_angle_ptr_ - YAW_ALIGEN_ANGLE;
        else if (*gimbal_yaw_motor_angle_ptr_ <= YAW_ALIGEN_ANGLE &&
                 *gimbal_yaw_motor_angle_ptr_ >= (YAW_ALIGEN_ANGLE - 180.0f))
            return *gimbal_yaw_motor_angle_ptr_ - YAW_ALIGEN_ANGLE;
        else
            return *gimbal_yaw_motor_angle_ptr_ - YAW_ALIGEN_ANGLE + 360.0f;
#endif
    }

    void rcControlSet()
    {
        if (_controller.rc_data.switch2 == 1)  // 遥控器右侧开关[上]
        {
            chassis_cmd_msg.mode      = chassis_mode::CHASSIS_NO_FOLLOW;
            gimbal_cmd_msg.force_stop = false;
            gimbal_cmd_msg.yaw        = gimbal_cmd_msg.yaw - _controller.rc_data.channel2 * 0.0005f;
            gimbal_cmd_msg.pitch      = gimbal_cmd_msg.pitch + _controller.rc_data.channel3 * 0.0005f;
        }
        else if (_controller.rc_data.switch2 == 3)  // 遥控器右侧开关[中]
        {
            chassis_cmd_msg.mode      = chassis_mode::CHASSIS_STOP;
            gimbal_cmd_msg.force_stop = true;
        }
        else if (_controller.rc_data.switch2 == 2)  // 遥控器右侧开关[下]
        {
            chassis_cmd_msg.mode      = chassis_mode::CHASSIS_ROTATE;
            gimbal_cmd_msg.force_stop = false;
            gimbal_cmd_msg.yaw        = gimbal_cmd_msg.yaw - _controller.rc_data.channel2 * 0.0005f;
            gimbal_cmd_msg.pitch      = gimbal_cmd_msg.pitch + _controller.rc_data.channel3 * 0.0005f;
        }
        chassis_cmd_msg.vx = _controller.rc_data.channel0 * 8.0f;
        chassis_cmd_msg.vy = _controller.rc_data.channel1 * 8.0f;
    }

    void mouseKeySet() { __asm volatile("nop"); }

    void visionSet()
    {
        VisionCommand::RobotCmd visionCmd;
        static uint32_t         fps       = 0;
        static uint32_t         fps_count = 0;
        static uint32_t         time_cnt  = STM32TimeDWT::GetMilliseconds();

        if (_controller.rc_data.switch2 == 3)  // 遥控器右侧开关[中]
        {
            chassis_cmd_msg.mode      = chassis_mode::CHASSIS_STOP;
            gimbal_cmd_msg.force_stop = true;
        }
        else
        {
            gimbal_cmd_msg.yaw   = gimbal_cmd_msg.yaw - _controller.rc_data.channel2 * 0.0005f;
            gimbal_cmd_msg.pitch = gimbal_cmd_msg.pitch + _controller.rc_data.channel3 * 0.0005f;

            chassis_cmd_msg.mode =
                _controller.rc_data.switch2 == 2 ? chassis_mode::CHASSIS_ROTATE : chassis_mode::CHASSIS_NO_FOLLOW;
            gimbal_cmd_msg.force_stop = false;

            if (xQueueReceive(_vision->queue, &visionCmd, 0) == pdTRUE)
            {
                if (visionCmd.shoot_mode != 0)
                {
                    gimbal_cmd_msg.yaw   = imu_data_->YawTotalAngle - visionCmd.yaw_angle;
                    gimbal_cmd_msg.pitch = imu_data_->Pitch + visionCmd.pitch_angle;
                    chassis_cmd_msg.vx   = _controller.rc_data.channel0 * 8.0f;
                    chassis_cmd_msg.vy   = _controller.rc_data.channel1 * 8.0f;
                    float yaw_vision     = visionCmd.yaw_angle;
                    taskENTER_CRITICAL();
                    LOGINFO("Vision", std::format("Y:{}, Yi:{}, Yv:{}, FPS:{}", gimbal_cmd_msg.yaw,
                                                  imu_data_->YawTotalAngle, yaw_vision, fps));
                    taskEXIT_CRITICAL();
                    fps_count++;
                }
            }
            if (STM32TimeDWT::GetMilliseconds() - time_cnt > 1000)
            {
                fps       = fps_count;
                fps_count = 0;
                time_cnt  = STM32TimeDWT::GetMilliseconds();
            }
        }
    }
};