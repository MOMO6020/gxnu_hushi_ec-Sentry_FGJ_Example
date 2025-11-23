/*
 * @Author       : Notch-FGJ mail.fgj.com@gmail.com
 * @Date         : 2025-04-07 10:59:49
 * @LastEditors  : Notch-FGJ mail.fgj.com@gmail.com
 * @LastEditTime : 2025-06-24 17:21:03
 * @FilePath     : \gxnu_hushi_ec\modules\motor\DJIMotor\DJIMotor.hpp
 * @Description  : 大疆电机驱动
 */
#pragma once
#include "../motor_def.hpp"
#include <array>

#include "bsp/can/stm32_can.hpp"
#include "modules/daemon/daemon.hpp"
#include "modules/motor/IMotor.hpp"

/**
 * @class DJIMotor
 * @brief 大疆电机类, 仅计算电机输出电流, 不控制电机, 使用MotorController类控制电机
 * @todo 完善注释, 添加PID前馈控制
 */
class DJIMotor : public IMotor
{
public:
    struct Measure
    {
        uint16_t last_encoder   = 0;  ///< 上次编码器值(0-8191)
        uint16_t encoder        = 0;  ///< 编码器值(0-8191)
        int16_t  speed          = 0;  ///< 电机转速(RPM)
        int16_t  torque_current = 0;  ///< 电机电流
        uint8_t  temperature    = 0;  ///< 电机温度
        float    speed_dps      = 0;  ///< 电机转速(degree/s)
        float    angle          = 0;  ///< 电机单圈角度(degree)
        float    total_angle    = 0;  ///< 总转动角度(degree)

        int32_t total_round = 0;  ///< 电机转动圈数
    };

    DJIMotor(CAN_HandleTypeDef* _hcan,
             uint16_t           _motor_id,
             MotorType          _motor_type,
             MotorPIDSetting    _setting,
             MotorPID           _pid_config);
    void                  decode(const uint8_t* buf, const uint8_t len);
    void                  offlineCallback();
    int16_t               calculateOutputCurrent();
    std::function<void()> calculateCallback = nullptr;  // 测试用回调函数, 在calculateControlData()中调用

    static void DJIMotorControl();

    Measure measure_;  ///< 电机测量数据

private:
    uint8_t                                                 motor_id_;                    ///< 电机ID
    uint8_t                                                 motor_tx_group_ = 0;          ///< 电机发送组ID
    inline static std::array<DJIMotor*, MAX_DJIMOTOR_COUNT> registered_motors_{nullptr};  ///< 电机数组
    inline static uint8_t                                   motor_count_ = 0;             ///< 电机数量

    /**
     * @brief 存储DJI电机控制数据的静态数组
     *
     * 由于DJI电机发送以4个为一组, 专门使用一个数组存储控制数据, 在DJIMotorControl()中统一管理并发送
     * M2006/M3508: 0x200, 0x1FF
     * GM6020:      0x1FE, 0x2FE
     *
     * 按照CAN总线与发送ID分组
     * CAN1: [0]: 0x200, [1]: 0x1FF, [2]: 0x1FE, [3]: 0x2FE
     * CAN2: [4]: 0x200, [5]: 0x1FF, [6]: 0x1FE, [7]: 0x2FE
     */
    inline static std::array<ICAN::ClassicPacket, 8> control_data_{
        // CAN1:
        ICAN::ClassicPacket{0x200, ICAN::Type::STANDARD, {0, 0, 0, 0, 0, 0, 0, 0}},  // M2006/M3508 1-4
        ICAN::ClassicPacket{0x1FF, ICAN::Type::STANDARD, {0, 0, 0, 0, 0, 0, 0, 0}},  // M2006/M3508 5-8
        ICAN::ClassicPacket{0x1FE, ICAN::Type::STANDARD, {0, 0, 0, 0, 0, 0, 0, 0}},  // GM6020 1-4
        ICAN::ClassicPacket{0x2FE, ICAN::Type::STANDARD, {0, 0, 0, 0, 0, 0, 0, 0}},  // GM6020 5-8
        // CAN2:
        ICAN::ClassicPacket{0x200, ICAN::Type::STANDARD, {0, 0, 0, 0, 0, 0, 0, 0}},  // M2006/M3508 1-4
        ICAN::ClassicPacket{0x1FF, ICAN::Type::STANDARD, {0, 0, 0, 0, 0, 0, 0, 0}},  // M2006/M3508 5-8
        ICAN::ClassicPacket{0x1FE, ICAN::Type::STANDARD, {0, 0, 0, 0, 0, 0, 0, 0}},  // GM6020 1-4
        ICAN::ClassicPacket{0x2FE, ICAN::Type::STANDARD, {0, 0, 0, 0, 0, 0, 0, 0}},  // GM6020 5-8
    };  ///< 控制数据数组
};