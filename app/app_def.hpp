/*
 * @Author       : Notch-FGJ mail.fgj.com@gmail.com
 * @Date         : 2025-04-11 21:05:26
 * @LastEditors  : Notch-FGJ mail.fgj.com@gmail.com
 * @LastEditTime : 2025-04-11 21:44:48
 * @FilePath     : \FrameworkA_FGJ\App\app_def.hpp
 * @Description  : app层公用定义
 */
#pragma once
#include "bsp/base_def.hpp"

//* --------------- 云台相关定义 ------------------ */
#define YAW_CHASSIS_ALIGEN_ENCODER 6800  ///< 云台与地盘对其时电机编码器值
#define YAW_CHASSIS_ALIGEN_THAN_4096 1   ///< ALIGEN_ENCODER是否大于4096, 用于过零处理判断
#define PITCH_HORIZON_ENCODER 3412       ///< 云台水平时电机编码器值

#define GYRO_GIMBAL_DIR_YAW 1    ///< 陀螺仪与云台YAW方向 (1:相同, -1:相反)
#define GYRO_GIMBAL_DIR_PITCH 1  ///< 陀螺仪与云台PITCH方向 (1:相同, -1:相反)
#define GYRO_GIMBAL_DIR_ROLL 1   ///< 陀螺仪与云台ROLL方向 (1:相同, -1:相反)

struct gimbal_cmd
{
    float yaw        = 0;      ///< 云台偏航角
    float pitch      = 0;      ///< 云台俯仰角
    bool  force_stop = false;  ///< 强制停止标志
};

//* --------------- 底盘相关定义 ------------------ */
#define CHASSIS_WHEEL_BASE 340     ///< 底盘纵向轴距 (前进方向)
#define CHASSIS_WHEEL_LATERAL 340  ///< 底盘横向轮距 (平移方向)
#define CHASSIS_WHEEL_RADIUS 60    ///< 轮子半径

#define CHASSIS_GIMBAL_OFFSET_X 0  ///< 云台旋转中心距底盘中心的距离 (前后方向)
#define CHASSIS_GIMBAL_OFFSET_Y 0  ///< 云台旋转中心距底盘中心的距离 (左右方向)

enum class chassis_mode
{
    CHASSIS_STOP,
    CHASSIS_ROTATE,         ///< 小陀螺模式
    CHASSIS_NO_FOLLOW,      ///< 通常模式
    CHASSIS_FOLLOW_GIMBAL,  ///< 底盘跟随云台
};

struct chassis_cmd
{
    float        vx           = 0;                           ///< 前进方向速度
    float        vy           = 0;                           ///< 横向速度
    float        offset_angle = 0;                           ///< 底盘和归中位置的夹角
    chassis_mode mode         = chassis_mode::CHASSIS_STOP;  ///< 底盘模式
};