/*
 * @Author       : Notch-FGJ mail.fgj.com@gmail.com
 * @Date         : 2025-04-07 13:59:32
 * @LastEditors  : Notch-FGJ mail.fgj.com@gmail.com
 * @LastEditTime : 2025-07-13 23:10:57
 * @FilePath     : \gxnu_hushi_ec\modules\motor\DJIMotor\DJIMotor.cpp
 * @Description  :
 */
#include "DJIMotor.hpp"
#include "bsp/can/stm32_can.hpp"
#include "bsp/log/log.hpp"
#include "cmsis_os2.h"
#include <cstring>

#define SPEED_SMOOTH_COEF 0.9f
#define CURRENT_SMOOTH_COEF 0.9f

/**
 * @brief 电机组使能标志
 * 与control_data_数组一一对应, 当对应组存在电机时, 使能标志为1
 * 用于防止发送空数据包
 */
bool group_enable_flag[8] = {false, false, false, false, false, false, false, false};

DJIMotor::DJIMotor(CAN_HandleTypeDef* _hcan,
                   uint16_t           _motor_id,
                   MotorType          _motor_type,
                   MotorPIDSetting    _setting,
                   MotorPID           _pid_config)
    : IMotor(_motor_type, _setting, _pid_config), motor_id_(_motor_id)
{
    if (motor_count_ >= MAX_DJIMOTOR_COUNT)
    {
        LOGERROR("DJIMotor", "Motor count exceeds maximum limit");
        while (true)
            osDelay(1000);
    }
    auto     can    = STM32CAN_GetInstance(_hcan);
    uint16_t can_id = motor_type_ == MotorType::GM6020 ? 0x204 + motor_id_ : 0x200 + motor_id_;
    can->setRxCallback(can_id, std::bind(&DJIMotor::decode, this, std::placeholders::_1, std::placeholders::_2));
    switch (motor_type_)
    {
        case MotorType::M2006:
        case MotorType::M3508:
            motor_tx_group_ = motor_id_ <= 4 ? 0 : 1;
            motor_tx_group_ = _hcan->Instance == CAN1 ? motor_tx_group_ : motor_tx_group_ + 4;
            reduce_rate_    = motor_type_ == MotorType::M2006 ? 36 : 19;
            break;
        case MotorType::GM6020:
            motor_tx_group_ = _hcan->Instance == CAN1 ? 2 : 3;
            motor_tx_group_ = motor_id_ <= 4 ? motor_tx_group_ : motor_tx_group_ + 4;
            reduce_rate_    = 1.0f;
            break;
        default: LOGERROR("DJIMotor", "Invalid motor type"); break;
    }
    // 检查ID冲突
    for (uint8_t i = 0; i < MAX_DJIMOTOR_COUNT; i++)
    {
        if (registered_motors_[i]->motor_tx_group_ == motor_tx_group_ && registered_motors_[i]->motor_id_ == motor_id_)
        {
            LOGERROR("DJIMotor", "Motor ID already exists");
            while (true)
                osDelay(1000);
        }
    }

    registered_motors_[motor_count_++] = this;  // 添加到电机数组
    group_enable_flag[motor_tx_group_] = true;  // 设置使能标志
}

void DJIMotor::decode(const uint8_t* buf, const uint8_t len)
{
    if (buf == nullptr || len < 8)
    {
        LOGERROR("DJIMotor", "Invalid buffer");
        return;
    }
    if (!is_online)
    {
        is_online = true;
        switch (motor_type_)
        {
            case MotorType::GM6020: LOGINFO("Motor GM6020#" + std::to_string(motor_id_), "Online"); break;
            case MotorType::M3508: LOGINFO("Motor M3508#" + std::to_string(motor_id_), "Online"); break;
            case MotorType::M2006: LOGINFO("Motor M2006#" + std::to_string(motor_id_), "Online"); break;
        }
    }
    daemon_->feed();
    measure_.last_encoder = measure_.encoder;
    measure_.encoder      = ((uint16_t)buf[0] << 8) | buf[1];
    measure_.angle        = (float)measure_.encoder / 8192.0f * 360.0f;
    measure_.speed        = (buf[2] << 8) | buf[3];
    measure_.speed_dps =
        (1.0f - SPEED_SMOOTH_COEF) * measure_.speed_dps + 6.0f * SPEED_SMOOTH_COEF * (float)measure_.speed;
    measure_.torque_current = (1.0f - CURRENT_SMOOTH_COEF) * measure_.torque_current +
                              CURRENT_SMOOTH_COEF * (float)((int16_t)(buf[4] << 8 | buf[5]));
    measure_.temperature = buf[6];
    if (measure_.encoder - measure_.last_encoder > 4096)
        measure_.total_round--;
    else if (measure_.last_encoder - measure_.encoder > 4096)
        measure_.total_round++;
    measure_.total_angle = measure_.total_round * 360.0f + measure_.angle;
}

void DJIMotor::offlineCallback()
{
    if (is_online)
    {
        memset(static_cast<void*>(&measure_), 0, sizeof(measure_));
        is_online = false;
        switch (motor_type_)
        {
            case MotorType::GM6020: LOGWARNING("Motor GM6020#" + std::to_string(motor_id_), "Offline"); break;
            case MotorType::M3508: LOGWARNING("Motor M3508#" + std::to_string(motor_id_), "Offline"); break;
            case MotorType::M2006: LOGWARNING("Motor M2006#" + std::to_string(motor_id_), "Offline"); break;
            default: LOGERROR("DJIMotor", "Invalid motor type"); break;
        }
    }
}

int16_t DJIMotor::calculateOutputCurrent()
{
    if (!is_online || !enable_)
        return 0;
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
                LOGERROR("DJIMotor", "Feedback pointer is null");
                return 0;
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
                LOGERROR("DJIMotor", "Feedback pointer is null");
                return 0;
            }
        else
            pid_measure = measure_.speed_dps;
        pid_ref = pidControllers_.pid_speed_.PIDCalculate(pid_measure, pid_ref * reduce_rate_);
    }
    // 电流环
    if (setting_.close_loop & CloseloopType::CURRENT_LOOP)
    {
        pid_ref = pidControllers_.pid_current_.PIDCalculate(measure_.torque_current, pid_ref);
    }
    pid_out_ = (int16_t)pid_ref;
    return pid_out_;
}

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void DJIMotor::DJIMotorControl()
{
    static CANHandle_t can1_handle = STM32CAN_GetInstance(&hcan1);
    static CANHandle_t can2_handle = STM32CAN_GetInstance(&hcan2);

    // 1. 清空控制数据和组使能标志
    for (auto& packet : control_data_)
        std::memset(packet.data, 0, 8);
    std::memset(group_enable_flag, 0, sizeof(group_enable_flag));

    // 2. 单次遍历：计算电流 + 更新组使能标志
    for (uint8_t i = 0; i < motor_count_; i++)
    {
        DJIMotor* motor = registered_motors_[i];
        if (motor->is_online)
        {
            // 计算电流并填充数据
            int16_t output_current = motor->calculateOutputCurrent();
            uint8_t idx            = motor->motor_id_ <= 4 ? 2 * (motor->motor_id_ - 1) : 2 * (motor->motor_id_ - 5);
            control_data_[motor->motor_tx_group_].data[idx]     = output_current >> 8;
            control_data_[motor->motor_tx_group_].data[idx + 1] = output_current & 0xFF;

            // 标记组有效（无需重复检查）
            group_enable_flag[motor->motor_tx_group_] = true;
        }
    }

    // 3. 发送有效组数据
    for (uint8_t group = 0; group < 8; group++)
    {
        if (group_enable_flag[group])
        {
            (group < 4 ? can1_handle : can2_handle)->transmit(control_data_[group]);
        }
    }
}
