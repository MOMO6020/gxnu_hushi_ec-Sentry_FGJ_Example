/*
 * @Author       : Notch-FGJ mail.fgj.com@gmail.com
 * @Date         : 2025-06-24 22:08:25
 * @LastEditors  : Notch-FGJ mail.fgj.com@gmail.com
 * @LastEditTime : 2025-06-26 22:44:29
 * @FilePath     : \gxnu_hushi_ec\modules\motor\DMMotor\DMMotor.hpp
 * @Description  : 达妙电机驱动
 */
#pragma once
#include "../IMotor.hpp"
#include "../motor_def.hpp"
#include "bsp/can/stm32_can.hpp"
#include "stm32f4xx_hal_can.h"
#include <cstdint>

/**
 * @brief 达妙电机驱动
 */
class DMMotor : public IMotor
{
public:
    enum class Status : uint8_t
    {
        DISABLED        = 0x00,  ///< 失能
        ENABLED         = 0x01,  ///< 使能
        OVERVOLT        = 0x08,  ///< 过压
        UNDERVOLT       = 0x09,  ///< 欠压
        OVERCURRENT     = 0x0A,  ///< 过流
        MOS_OVERHEAT    = 0x0B,  ///< MOS管过热
        ROTOR_OVERHEAT  = 0x0C,  ///< 线圈过热
        CONNECTION_LOST = 0x0D,  ///< 连接丢失
        OVERLOAD        = 0x0E,  ///< 过载
    };

    struct Measure
    {
        uint8_t id                = 0;                 ///< 电机ID
        Status  state             = Status::DISABLED;  ///< 电机状态
        float   position          = 0.0f;              ///< 电机位置 (rad)
        float   velocity          = 0.0f;              ///< 电机速度 (rad/s)
        float   torque            = 0.0f;              ///< 电机扭矩
        float   temperature_mos   = 0.0f;              ///< MOS管温度 (℃)
        float   temperature_rotor = 0.0f;              ///< 线圈温度 (℃)
        int32_t total_round       = 0;                 ///< 电机转动圈数

        uint16_t encoder      = 0;     ///< 编码器值(使用反馈的弧度进行转换)
        uint16_t last_encoder = 0;     ///< 上次编码器值(用于计算转动圈数)
        float    angle        = 0.0f;  ///< 电机单圈角度 (degree)
        float    total_angle  = 0.0f;  ///< 总转动角度 (degree)
        float    speed_dps    = 0.0f;  ///< 电机转速 (degree/s)
    };

    enum class ControlCommand : uint8_t
    {
        Motor_Enable            = 0xfc,  ///< 电机使能
        Motor_Disable           = 0xfd,  ///< 电机禁用
        Motor_Set_Zero_Position = 0xfe,  ///< 设置电机零位
        Motor_Clear_Error       = 0xfb,  ///< 清除电机错误
    };

    DMMotor(CAN_HandleTypeDef* _hcan,
            uint16_t           _tx_id,
            uint16_t           _rx_id,
            MotorType          _motor_type,
            MotorPIDSetting    _setting,
            MotorPID           _pid_config);

    void      decode(const uint8_t* buf, const uint8_t len);
    void      offlineCallback() override;
    ErrorCode sendControlCommand(ControlCommand command);  ///< 发送控制命令

    void motorControl();

    static void DMMotorControl();

    Measure measure_;  ///< 电机测量数据
private:
    CANHandle_t         can_handle_;  ///< CAN句柄
    uint16_t            motor_can_tx_id_ = 0;
    uint16_t            motor_can_rx_id_ = 0;  ///< 电机反馈报文ID
    ICAN::ClassicPacket tx_buffer_;            ///< 发送缓冲区

    inline static std::vector<DMMotor*> registered_motors_;  ///< 注册的DMMotor实例

    void setMITControlPacket(float position_des, float velocity_des, float torque_des, uint16_t kp, uint16_t kd);
};