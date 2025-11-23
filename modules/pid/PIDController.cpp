/*
 * @Author       : Notch-FGJ mail.fgj.com@gmail.com
 * @Date         : 2025-03-31 11:50:26
 * @LastEditors  : Notch-FGJ mail.fgj.com@gmail.com
 * @LastEditTime : 2025-04-11 18:52:51
 * @FilePath     : \FrameworkA_FGJ\modules\pid\PIDController.cpp
 * @Description  :
 */
#include "PIDController.hpp"
#include <cmath>
#include <algorithm>

PIDController::PIDController(const PID_Config &config)
    : Kp(config.Kp), Ki(config.Ki), Kd(config.Kd), MaxOutput(config.MaxOutput), Deadband(config.Deadband),
      Improve(config.Improve), IntegralLimit(config.IntegralLimit), CoefA(config.CoefA), CoefB(config.CoefB),
      Output_LPF_RC(config.Output_LPF_RC), Derivative_LPF_RC(config.Derivative_LPF_RC)
{
    STM32TimeDWT::GetDelta(DWT_CNT);
    dt = 0.001f; // 默认时间间隔为1ms
}

PIDController::PIDController()
{
    STM32TimeDWT::GetDelta(DWT_CNT);
    dt = 0.001f; // 默认时间间隔为1ms
}

float PIDController::PIDCalculate(float measure, float ref)
{                                     
    if (static_cast<uint8_t>(Improve & PID_Improvement::PID_ErrorHandle))
        f_ErrorHandle();
    dt = (STM32TimeDWT::GetDelta(DWT_CNT)) / 1000.0f; // 获取时间间隔(秒)
    if (dt < 0.001f)
        dt = 0.001f; // 防止除0错误
    // 更新测量值及设定值, 计算误差
    Measure = measure;
    Ref = ref;
    Err = Ref - Measure;
    if (std::abs(Err) > Deadband)
    {
        // 基本的PID计算, 使用位置式
        Pout = Kp * Err;
        ITerm = Ki * Err * dt;
        Dout = Kd * (Err - Last_Err) / dt;
        // 梯形积分
        if (Improve & PID_Improvement::PID_Trapezoid_Integral)
            f_TrapezoidIntegral();
        // 变速积分
        if (Improve & PID_Improvement::PID_Changing_Integration_Rate)
            f_ChangingIntegrationRate();
        // 微分先行
        if (Improve & PID_Improvement::PID_Derivative_On_Measurement)
            f_DerivativeOnMeasurement();
        // 微分滤波
        if (Improve & PID_Improvement::PID_Derivative_Filter)
            f_DerivativeFilter();
        // 积分限幅
        if (Improve & PID_Improvement::PID_Integral_Limit)
            f_IntegralLimit();
        Iout += ITerm;               // 累加积分
        Output = Pout + Iout + Dout; // 计算输出
        // 输出滤波
        if (Improve & PID_Improvement::PID_Output_Filter)
            f_OutputFilter();
        // 输出限幅
        f_OutputLimit();
    }
    else // 进入死区, 清空积分及输出
    {
        Output = 0.0f;
        ITerm = 0.0f;
    }
    // 保存当前数据
    Last_Measure = Measure;
    Last_Err = Err;
    Last_Output = Output;
    Last_ITerm = ITerm;
    Last_Dout = Dout;
    return Output;
}

void PIDController::f_ErrorHandle()
{
    if (std::abs(Output) < MaxOutput * 0.001f || std::abs(Ref) < 0.0001f)
        return;
    if (std::abs(Ref - Measure) / std::abs(Ref) > 0.95f)
        Stall_CNT++;
    else
        Stall_CNT = 0;
    if (Stall_CNT > 500)
        Error = PID_Error::PID_ERROR_STALL;
    else
        Error = PID_Error::PID_ERROR_NONE;
    // TODO: 可以添加错误处理
}

void PIDController::f_TrapezoidIntegral()
{
    ITerm = Ki * ((Err + Last_Err) / 2.0f) * dt;
}

void PIDController::f_ChangingIntegrationRate()
{
    if (Err * Iout > 0)
    {
        if (std::abs(Err) <= CoefB)
            return;
        if (abs(Err) <= (CoefA + CoefB))
            ITerm *= (CoefA - (abs(Err) - CoefB)) / CoefA;
        else
            ITerm = 0.0f;
    }
}

void PIDController::f_DerivativeOnMeasurement()
{
    Dout = Kd * (Last_Dout - Last_Measure) / dt;
}

void PIDController::f_DerivativeFilter()
{
    Dout = Dout * dt / (Derivative_LPF_RC + dt) + Last_Dout * Derivative_LPF_RC / (Derivative_LPF_RC + dt);
}

void PIDController::f_IntegralLimit()
{
    static float temp_output, temp_Iout;
    temp_Iout = Iout + ITerm;
    temp_output = Pout + Iout + Dout;
    if (std::abs(temp_output) > MaxOutput)
        if (Err * Iout > 0)
            ITerm = 0;
    if (temp_Iout > IntegralLimit)
    {
        ITerm = 0;
        Iout = IntegralLimit;
    }
    if (temp_Iout < -IntegralLimit)
    {
        ITerm = 0;
        Iout = -IntegralLimit;
    }
}

void PIDController::f_OutputFilter()
{
    Output = Output * dt / (Output_LPF_RC + dt) +
                Last_Output * Output_LPF_RC / (Output_LPF_RC + dt); // 低通滤波器
}

void PIDController::f_OutputLimit()
{
    Output = std::clamp(Output, -MaxOutput, MaxOutput);
}
