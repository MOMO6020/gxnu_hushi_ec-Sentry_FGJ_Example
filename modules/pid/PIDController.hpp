/*
 * @Author       : Notch-FGJ mail.fgj.com@gmail.com
 * @Date         : 2025-03-31 10:24:44
 * @LastEditors  : Notch-FGJ mail.fgj.com@gmail.com
 * @LastEditTime : 2025-04-11 15:57:07
 * @FilePath     : \FrameworkA_FGJ\modules\pid\PIDController.hpp
 * @Description  : PID控制器
 */

#pragma once
#include <bsp/system/time.hpp>
#include <bsp/base_def.hpp>

class PIDController
{
public:
    // TODO: 完善注释
    /// @brief PID改进选项, 通过位掩码控制是否启用某些功能, 例: PID_Integral_Limit | PID_Derivative_On_Measurement
    enum class PID_Improvement : uint8_t
    {
        /// @brief 无优化
        PID_IMPROVE_NONE = 0b00000000,
        /**
         * @brief   积分限幅
         * @details 限制积分项 Iout 的最大值，防止积分饱和(Windup). 当积分累计值超过 IntegralLimit 时，强制限制积分输出.
         * @note    IntegralLimit须合理设置,过小会导致积分作用不足, 过大则仍导致积分饱和
         *          当 PID_Integral_Limit 和 PID_Changing_Integration_Rate 同时使用时，PID_Integral_Limit 的限制优先级更高. (先进行变速积分计算, 后进行积分限幅)
         */
        PID_Integral_Limit = 0b00000001,
        /**
         * @brief   微分先行
         * @details 微分项仅作用于测量值(Last_Measure - Measure)，而非误差变化(Err - Last_Err), 避免设定值(Ref)突变时微分项的剧烈波动(减少“微分冲击”). 适用于设定值频繁变化的系统.
         * @note    可能导致系统对测量噪声更敏感(需配合 PID_Derivative_Filter 使用).
         */
        PID_Derivative_On_Measurement = 0b00000010,
        /// @brief 梯形积分: 将矩形积分(Ki * Err * dt)改为梯形积分(Ki * (Err + Last_Err) / 2 * dt)，提高积分精度
        PID_Trapezoid_Integral = 0b00000100,
        /// @brief 比例先行: 比例项基于测量值变化(Pout = Kp * (Last_Measure - Measure))，而非误差(Pout = Kp * Err)
        PID_Proportional_On_Measurement = 0b00001000,   //! < 该选项未实现
        /// @brief 输出滤波: 对 PID 输出进行一阶低通滤波(Output = α * Last_Output + (1 - α) * Current_Output)
        PID_Output_Filter = 0b00010000,
        /// @brief 变速积分: 动态调整积分强度：误差大时减弱积分，误差小时增强积分(ITerm *= (A - |Err| + B) / A (B < |err| < A + B))
        PID_Changing_Integration_Rate = 0b00100000,
        /// @brief 微分滤波: 对微分项进行一阶低通滤波(Dout = (Dout * dt + Last_Dout * RC) / (dt + RC))
        PID_Derivative_Filter = 0b01000000,
        /// @brief 错误检测: 检测电机堵转：当输出较大但测量值长期不变化时，触发错误标志(逻辑: |Ref - Measure| / |Ref| > Threshold 持续超过500次计算周期)
        PID_ErrorHandle = 0b10000000
    };

    enum class PID_Error : uint8_t
    {
        PID_ERROR_NONE = 0b00000000,
        PID_ERROR_STALL = 0b00000001 //< 电机堵转
    };

    /// @brief PID参数配置结构体, 用于初始化PID控制器
    struct PID_Config
    {
        float Kp = 0;                                                    //< 比例增益
        float Ki = 0;                                                    //< 积分增益
        float Kd = 0;                                                    //< 微分增益
        float MaxOutput = 0;                                             //< 最大输出
        float Deadband = 0;                                              //< 死区
        PID_Improvement Improve = PID_Improvement::PID_IMPROVE_NONE; //< PID改进选项
        float IntegralLimit = 0;                                     //< 积分限幅
        float CoefA = 0;                                             //< 变速积分系数A
        float CoefB = 0;                                             //< 变速积分系数B
        float Output_LPF_RC = 0;                                     //< 输出滤波器 RC = 1 / Omegac
        float Derivative_LPF_RC = 0;                                 //< 微分滤波器系数
    };

    /**
     * @brief PID控制器构造函数
     * @param config PID参数配置结构体
     */
    PIDController(const PID_Config &config);

    PIDController();

    /// @brief PID计算
    /// @param measure 测量值
    /// @param ref     设定值
    /// @return PID输出值
    float PIDCalculate(float measure, float ref);

private:
    //*------------- PID参数 ------------
    float Kp = 0;    //< 比例增益
    float Ki = 0;    //< 积分增益
    float Kd = 0;    //< 微分增益
    float MaxOutput = 0; //< 最大输出
    float Deadband = 0; //< 死区
    //*------------- PID改进选项及参数 ------------
    PID_Improvement Improve; //< PID改进选项
    float IntegralLimit = 0; //< 积分限幅
    float CoefA = 0;         //< 变速积分系数A
    float CoefB = 0;         //< 变速积分系数B
    float Output_LPF_RC = 0; //< 输出滤波器 RC = 1 / Omegac
    float Derivative_LPF_RC = 0; //< 微分滤波器系数
    //*------------- PID计算中间变量 ------------
    float Measure = 0;      //< 测量值
    float Last_Measure = 0; //< 上次测量值
    float Err = 0;          //< 误差
    float Last_Err = 0;     //< 上次误差
    float Pout = 0;         //< 比例项
    float Iout = 0;         //< 积分项
    float Dout = 0;         //< 微分项
    float Last_Dout = 0;    //< 上次微分项
    float ITerm = 0;        //< 积分累计值
    float Last_ITerm = 0;   //< 上次积分累计值
    float Output = 0;       //< 输出值
    float Last_Output = 0;  //< 上次输出值
    float Ref = 0;          //< 设定值
    uint32_t DWT_CNT = 0;   //< DWT计时器, 获取两次PID计算的时间间隔, 用于积分与微分计算
    float dt = 0;           //< PID计算周期, 单位: 秒
    uint32_t Stall_CNT = 0; //< 堵转计数器, 用于检测电机堵转
    PID_Error Error;    //< 错误标志
    //*------------- 私有计算函数 ------------
    void f_ErrorHandle(); //< 错误检测及处理
    void f_TrapezoidIntegral(); //< 梯形积分
    void f_ChangingIntegrationRate(); //< 变速积分
    void f_DerivativeOnMeasurement(); //< 微分先行
    void f_DerivativeFilter();        //< 微分滤波
    void f_IntegralLimit();           //< 积分限幅
    void f_OutputFilter();            //< 输出滤波
    void f_OutputLimit();             //< 输出限幅
};

using PIDConfig = PIDController::PID_Config;
using PIDImprovement = PIDController::PID_Improvement;