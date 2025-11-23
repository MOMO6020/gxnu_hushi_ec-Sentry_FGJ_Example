/*
 * @Author       : Notch-FGJ mail.fgj.com@gmail.com
 * @Date         : 2025-06-25 00:23:33
 * @LastEditors  : Notch-FGJ mail.fgj.com@gmail.com
 * @LastEditTime : 2025-06-25 01:46:15
 * @FilePath     : \gxnu_hushi_ec\modules\motor\DMMotor\DMMotor.cpp
 * @Description  :
 * @todo         : ID冲突检查
 */
#include "DMMotor.hpp"
#include "bsp/base_def.hpp"
#include <cstdint>
#include <cstring>
#include <math.h>

#include <format>

#define PMAX 3.14159265
#define VMAX 30
#define TMAX 10

// 将uint值与float值进行映射
inline uint16_t float_to_uint16(float value, float min, float max, uint8_t bits)
{
    float span   = max - min;
    float offset = min;
    return (uint16_t)((value - offset) * ((float)((1 << bits) - 1)) / span);
}

inline float uint16_to_float(uint16_t value, float min, float max, uint8_t bits)
{
    return ((float)value) * (max - min) / ((float)((1 << bits) - 1)) + min;
}

DMMotor::DMMotor(CAN_HandleTypeDef* _hcan,
                 uint16_t           _tx_id,
                 uint16_t           _rx_id,
                 MotorType          _motor_type,
                 MotorPIDSetting    _setting,
                 MotorPID           _pid_config)
    : IMotor(_motor_type, _setting, _pid_config), motor_can_tx_id_(_tx_id), motor_can_rx_id_(_rx_id)
{
    can_handle_ = STM32CAN_GetInstance(_hcan);
    can_handle_->setRxCallback(motor_can_rx_id_,
                               std::bind(&DMMotor::decode, this, std::placeholders::_1, std::placeholders::_2));
    tx_buffer_.id   = motor_can_tx_id_;
    tx_buffer_.type = ICAN::Type::STANDARD;
    if (sendControlCommand(ControlCommand::Motor_Enable) != ErrorCode::OK)
    {
        LOGERROR("DMMotor", "Failed to enable motor #" + std::to_string(motor_can_tx_id_));
        return;
    }
    registered_motors_.push_back(this);  // 添加到电机数组
    LOGINFO("DMMotor",
            "Motor TX ID: " + std::to_string(motor_can_tx_id_) + ", RX ID: " + std::to_string(motor_can_rx_id_));
}

void DMMotor::decode(const uint8_t* buf, const uint8_t len)
{
    if (buf == nullptr || len < 8)
    {
        LOGERROR("DJIMotor", "Invalid buffer");
        return;
    }
    if (!is_online)
    {
        is_online = true;
        LOGINFO("DMMotor", "DaMiao Motor #" + std::to_string(motor_can_tx_id_) + " online");
    }
    daemon_->feed();
    uint16_t temp;
    measure_.id       = buf[0] & 0x0F;                     // 电机ID
    measure_.state    = static_cast<Status>(buf[0] >> 4);  // 电机状态
    temp              = (uint16_t)(buf[1] << 8 | buf[2]);
    measure_.position = uint16_to_float(temp, -PMAX, PMAX, 16);  // 电机位置 (rad)

    temp              = (uint16_t)(buf[3] << 4 | (buf[4] >> 4));
    measure_.velocity = uint16_to_float(temp, -VMAX, VMAX, 12);  // 电机速度 (rad/s)

    temp            = (uint16_t)((buf[4] & 0x0F) << 8 | buf[5]);
    measure_.torque = uint16_to_float(temp, -TMAX, TMAX, 12);  // 电机扭矩

    measure_.temperature_mos   = (float)buf[6];
    measure_.temperature_rotor = (float)buf[7];

    measure_.last_encoder = measure_.encoder;  // 更新上次编码器值
    measure_.encoder      = ((measure_.position + PMAX) / (2 * PMAX)) * 8192.0f;
    measure_.angle        = (float)measure_.encoder / 8192.0f * 360.0f;  // 计算电机单圈角度
    measure_.speed_dps    = measure_.velocity * 180.0f / M_PI;
    if (measure_.encoder - measure_.last_encoder > 4096)
        measure_.total_round--;
    else if (measure_.last_encoder - measure_.encoder > 4096)
        measure_.total_round++;
    measure_.total_angle = measure_.total_round * 360.0f + measure_.angle;
}

ErrorCode DMMotor::sendControlCommand(DMMotor::ControlCommand command)
{
    std::memset(tx_buffer_.data, 0xff, sizeof(tx_buffer_.data));
    tx_buffer_.data[7] = static_cast<uint8_t>(command);
    return can_handle_->transmit(tx_buffer_, 1);
}

void DMMotor::offlineCallback()
{
    if (is_online)
    {
        memset(static_cast<void*>(&measure_), 0, sizeof(measure_));
        is_online = false;
        LOGWARNING("DMMotor", "Motor #" + std::to_string(motor_can_tx_id_) + " offline");
    }
}

void DMMotor::motorControl()
{
    if (!enable_)
    {
        sendControlCommand(ControlCommand::Motor_Disable);
        return;
    }
    if (measure_.state == Status::DISABLED)
    {
        sendControlCommand(ControlCommand::Motor_Enable);
        return;
    }
    // TODO: 添加错误处理逻辑

    // 计算PID输出并发送控制命令
    float pid_measure;
    float pid_ref = pid_ref_;
    if (setting_.reverse)
        pid_ref *= -1.0f;
    // 位置环
    if ((setting_.close_loop & CloseloopType::ANGLE_LOOP) && setting_.outer_loop == CloseloopType::ANGLE_LOOP)
    {
        if (setting_.external_angle_feedback == FeedbackType::EXTERNAL)
            if (pidControllers_.pid_angle_feedback_ptr_ != nullptr)
                pid_measure = *pidControllers_.pid_angle_feedback_ptr_;
            else
            {
                LOGERROR("DMMotor", "Feedback pointer is null");
                return;
            }
        else
            pid_measure = measure_.total_angle;
        pid_ref = pidControllers_.pid_angle_.PIDCalculate(pid_measure, pid_ref * reduce_rate_);
    }
    // 速度环
    if ((setting_.close_loop & CloseloopType::SPEED_LOOP) &&
        (uint8_t(setting_.outer_loop) & (CloseloopType::ANGLE_LOOP | CloseloopType::SPEED_LOOP)))
    {
        if (setting_.external_speed_feedback == FeedbackType::EXTERNAL)
            if (pidControllers_.pid_speed_feedback_ptr_ != nullptr)
                pid_measure = *pidControllers_.pid_speed_feedback_ptr_;
            else
            {
                LOGERROR("DMMotor", "Feedback pointer is null");
                return;
            }
        else
            pid_measure = measure_.speed_dps;
        pid_ref = pidControllers_.pid_speed_.PIDCalculate(pid_measure, pid_ref * reduce_rate_);
    }
    // 电流环
    if (setting_.close_loop & CloseloopType::CURRENT_LOOP)
    {
        pid_ref = pidControllers_.pid_current_.PIDCalculate(measure_.torque, pid_ref);
    }

    setMITControlPacket(0, 0, pid_ref, 0, 0);
    can_handle_->transmit(tx_buffer_, 1);
}

void DMMotor::DMMotorControl()
{
    for (auto& motor : registered_motors_)
    {
        motor->motorControl();
    }
}

void DMMotor::setMITControlPacket(float position_des, float velocity_des, float torque_des, uint16_t kp, uint16_t kd)
{
    uint16_t position  = float_to_uint16(position_des, -PMAX, PMAX, 16);
    uint16_t velocity  = float_to_uint16(velocity_des, -VMAX, VMAX, 12);
    uint16_t torque    = float_to_uint16(torque_des, -TMAX, TMAX, 12);
    tx_buffer_.data[0] = (uint8_t)(position >> 8);
    tx_buffer_.data[1] = (uint8_t)(position & 0xFF);
    tx_buffer_.data[2] = (uint8_t)(velocity >> 4);
    tx_buffer_.data[3] = (uint8_t)((velocity & 0x0F) << 4 | (kp >> 8));
    tx_buffer_.data[4] = (uint8_t)(kp & 0xFF);
    tx_buffer_.data[5] = (uint8_t)(kd >> 4);
    tx_buffer_.data[6] = (uint8_t)(((kd & 0x0F) << 4) | (uint8_t)(torque >> 8));
    tx_buffer_.data[7] = (uint8_t)(torque & 0xFF);
    // LOGINFO("DMMotor", std::format("Set MIT control packet: Position: {}, Velocity: {}, Torque: {}", position_des,
    //                                velocity_des, torque_des));
}