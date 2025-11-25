#include "app_main.h"
#include "FreeRTOS.h"
#include "bsp/can/stm32_can.hpp"
#include "main.h"
#include "modules/pid/PIDController.hpp"
#include "stm32f4xx_hal_can.h"
#include "task.h"

#include "bsp/log/log.hpp"
#include "bsp/system/time.hpp"
#include "bsp/uart/stm32_uart.hpp"
#include "modules/motor/DMMotor/DMMotor.hpp"//damiao
#include "modules/motor/DJIMotor/DJIMotor.hpp"
#include "modules/motor/motor.hpp"
#include "stm32f4xx_hal_uart.h"
#include <cstdlib>
#include <memory>
#include <string>
#include <algorithm>// 用于std::clamp（C++17及以上支持


// 1. 新增：串口指令结构体（解析"电机类型:目标角度"格式）
struct SerialAngleCmd {
    std::string motor_type;  // 电机类型（GM6020/J4310/J8009）
    float target_angle;      // 目标角度（度）
    bool is_valid = false;   // 指令是否有效
};

extern UART_HandleTypeDef huart1;
extern CAN_HandleTypeDef  hcan1;

UARTHandle_t              usart1  = nullptr;
// 2. 新增：支持多电机（GM6020/J4310/J8009，根据实际硬件调整ID和类型）
std::shared_ptr<DJIMotor> gm6020_motor = nullptr;
std::shared_ptr<DMMotor> j4310_motor  = nullptr;
std::shared_ptr<DMMotor> j8009_motor  = nullptr;

volatile SerialAngleCmd            g_serial_cmd;  // 全局串口指令缓存
//std::mutex                g_cmd_mutex;   // 修正2：添加互斥锁，保护全局指令

void uart1_callback(uint8_t* buf, uint32_t len);
void uart_feedback_task(void* arg);
void motor_angle_control_task(void* arg);
// 新增：解析串口指令的工具函数
SerialAngleCmd parse_serial_cmd(const std::string& cmd_str);

// 1. 在函数声明区域添加（app_main函数上方）
void uart_send_with_retry(UARTHandle_t uart, const std::string& msg);

void app_main()
{
    __disable_irq();
    RTTLog::init();
    STM32TimeDWT::DWT_Init(168);
    STM32CAN_Init();
    StartMotorControlTask();
    
    // 3. 初始化串口（保持原有，回调函数用于解析指令）
    usart1 = STM32UART_Init(&huart1, uart1_callback, nullptr);
    if (usart1 == nullptr) {
        LOGERROR("Serial", "USART1 init failed!");
    } else {
        LOGINFO("Serial", "USART1 ready, send cmd like 'GM6020:90' or 'J4310:180'");
        std::string welcome = "System ready! Support cmd: GM6020:angle, J4310:angle, J8009:angle\r\n";
        uart_send_with_retry(usart1, welcome);  // 改用带重试的发送函数
    }
    
    // 4. 初始化电机（关键：配置为角度环+速度环，适配位置控制）
    // GM6020初始化（CAN1，ID=1，限位±180度，根据机械结构调整）   
    gm6020_motor  = std::make_shared<DJIMotor>(
        &hcan1, 1, MotorType::GM6020,
        MotorPIDSetting{

             .outer_loop = CloseloopType::ANGLE_LOOP,
             .close_loop = CloseloopType::ANGLE_AND_SPEED_LOOP,
             .reverse    = false,
             .external_angle_feedback = FeedbackType::INTERNAL,
             .external_speed_feedback = FeedbackType::INTERNAL,
        },
        MotorPID{
             .pid_angle_ =
                PIDController(PIDConfig{35.0f, 0.8f, 1.2f, 5000.0f, 300.0f, PIDImprovement::PID_Integral_Limit, 180.0f}),
             .pid_speed_ =
                PIDController(PIDConfig{12.0f, 0.2f, 0.1f, 16384.0f, 1000.0f, PIDImprovement::PID_Integral_Limit}),
        });

    // 2. J4310（达妙电机，用DMMotor，关键适配）
    j4310_motor = std::make_shared<DMMotor>(
        &hcan1,                  // CAN总线句柄
        2,                       // CAN发送ID（根据实际硬件调整）
        2 + 0x200,               // CAN接收ID（达妙电机通常是发送ID+0x200，需确认）
        MotorType::J4310,        // 电机类型（框架中需定义J4310，若无则自定义）
        MotorPIDSetting{
            .outer_loop = CloseloopType::ANGLE_LOOP,          // 外环=角度环（位置控制核心）
            .close_loop = CloseloopType::ANGLE_AND_SPEED_LOOP,// 串级环=角度+速度
            .reverse = false,                                 // 转向是否反转（实际调试调整）
            .external_angle_feedback = FeedbackType::INTERNAL,// 用电机内置编码器
            .external_speed_feedback = FeedbackType::INTERNAL
        },
        MotorPID{
            // 角度环PID（达妙J4310推荐参数，需调试优化）
            .pid_angle_ = PIDController(PIDConfig{25.0f, 0.3f, 0.8f, 1000.0f, 200.0f, PIDImprovement::PID_Integral_Limit}),
            // 速度环PID（辅助角度环，优化动态响应）
            .pid_speed_ = PIDController(PIDConfig{8.0f, 0.1f, 0.05f, 3000.0f, 500.0f, PIDImprovement::PID_Integral_Limit})
        }
    );

    // 3. J8009（达妙电机，用DMMotor，关键适配）
    j8009_motor = std::make_shared<DMMotor>(
        &hcan1,
        3,                       // CAN发送ID
        3 + 0x200,               // CAN接收ID（达妙电机默认规则）
        MotorType::J8009,
        MotorPIDSetting{
            .outer_loop = CloseloopType::ANGLE_LOOP,
            .close_loop = CloseloopType::ANGLE_AND_SPEED_LOOP,
            .reverse = true,                                  // 转向反转（根据实际调整）
            .external_angle_feedback = FeedbackType::INTERNAL,
            .external_speed_feedback = FeedbackType::INTERNAL
        },
        MotorPID{
            .pid_angle_ = PIDController(PIDConfig{22.0f, 0.4f, 0.7f, 900.0f, 180.0f, PIDImprovement::PID_Integral_Limit}),
            .pid_speed_ = PIDController(PIDConfig{7.0f, 0.15f, 0.03f, 2800.0f, 450.0f, PIDImprovement::PID_Integral_Limit})
        }
    );

    // 使能电机（达妙电机和DJI电机使能接口一致）
    if (gm6020_motor) gm6020_motor->enable();
    if (j4310_motor)  j4310_motor->enable();
    if (j8009_motor)  j8009_motor->enable();

    xTaskCreate(uart_feedback_task, "uart_feedback", 512, nullptr, 2, nullptr);
    xTaskCreate(motor_angle_control_task, "motor_angle_control", 1024, nullptr,3, nullptr);

    __enable_irq();
}

// 串口发送辅助函数（带重试，避免BUSY失败+空指针检查）
void uart_send_with_retry(UARTHandle_t uart, const std::string& msg) {
    if (uart == nullptr || msg.empty()) return;
    uint32_t retry = 3;
    while (retry-- > 0) {
        ErrorCode err = uart->transmit((uint8_t*)msg.c_str(), msg.length());
        if (err == ErrorCode::OK) break;
        osDelay(1);
    }
}

// 串口回调+指令解析
void uart1_callback(uint8_t* buf, uint32_t len)
{
    if (buf == nullptr || len == 0) return;

    // 转换为字符串，去除换行/空格等无效字符
    std::string cmd_str((char*)buf, len);
    cmd_str.erase(std::remove_if(cmd_str.begin(), cmd_str.end(), [](char c) {
        return c == '\r' || c == '\n' || c == ' ';
    }), cmd_str.end());

    // 解析指令
    SerialAngleCmd cmd = parse_serial_cmd(cmd_str);
    if (cmd.is_valid) {
        // 中断中直接赋值（volatile保证可见性）
        g_serial_cmd.motor_type = cmd.motor_type;
        g_serial_cmd.target_angle = cmd.target_angle;
        g_serial_cmd.is_valid = true;
        // 串口反馈：指令接收成功（让用户知道已收到）
        std::string ack = "Received: " + cmd.motor_type + ":" + std::to_string(cmd.target_angle) + "°\r\n";
        uart_send_with_retry(usart1, ack);  // 改用带重试的发送函数
    } else {
        // 无效指令反馈：明确告知格式要求
        std::string err = "Error: Invalid cmd! Use 'GM6020:90' (angle range: GM6020±180, J4310 0-360, J8009±90)\r\n";
        uart_send_with_retry(usart1, err);  // 改用带重试的发送函数
    }
}


// 7. 指令解析工具函数（修正9：校验电机类型合法性）
SerialAngleCmd parse_serial_cmd(const std::string& cmd_str)
{
    SerialAngleCmd cmd;
    size_t colon_pos = cmd_str.find(':');

    // 校验格式：必须包含 ':'，且前后有内容
    if (colon_pos == std::string::npos || colon_pos == 0 || colon_pos == cmd_str.length()-1) {
        return cmd;
    }

    // 提取电机类型和角度
    cmd.motor_type = cmd_str.substr(0, colon_pos);
    try {
        cmd.target_angle = std::stof(cmd_str.substr(colon_pos + 1));
        // 修正10：只允许3种电机类型，避免无效指令
        if (cmd.motor_type == "GM6020" || cmd.motor_type == "J4310" || cmd.motor_type == "J8009") {
            cmd.is_valid = true;
        }
    } catch (...) {
        cmd.is_valid = false;
    }

    return cmd;
}

// 8. 串口反馈任务（修正11：定时输出当前角度，确保串口有数据）
void uart_feedback_task(void* arg __attribute__((unused)))
{
    while (true)
    {
        if (usart1 == nullptr) {
            vTaskDelay(100);
            continue;
        }

        std::string feedback = "Current angles: ";
        // GM6020：DJIMotor读取 measure_.total_angle
        if (gm6020_motor) {
            feedback += "GM6020=" + std::to_string(gm6020_motor->measure_.total_angle) + "°; ";
        }
        // J4310/J8009：DMMotor读取 get_current_angle()
        if (j4310_motor) {
            feedback += "J4310=" + std::to_string(j4310_motor->get_current_angle()) + "°; ";
        }
        if (j8009_motor) {
            feedback += "J8009=" + std::to_string(j8009_motor->get_current_angle()) + "°; ";
        }
        feedback += "\r\n";

        // 发送反馈（忽略BUSY，避免阻塞）
        uart_send_with_retry(usart1, feedback);  // 改用带重试的发送函数
        vTaskDelay(500); // 每500ms更新一次
    }
}

// 电机角度控制任务（任务上下文，用临界区读指令）
void motor_angle_control_task(void* arg __attribute__((unused)))
{
    while (true)
    {
        // 用FreeRTOS临界区保护，读取全局指令
        SerialAngleCmd current_cmd;
        taskENTER_CRITICAL();
        if (g_serial_cmd.is_valid) {
            current_cmd = g_serial_cmd;
            g_serial_cmd.is_valid = false;
        }
        taskEXIT_CRITICAL();

        if (!current_cmd.is_valid) {
            vTaskDelay(5);
            continue;
        }

        float clamped_angle = 0.0f;
        bool is_clamped = false;

        if (current_cmd.motor_type == "GM6020" && gm6020_motor) {
            clamped_angle = std::clamp(current_cmd.target_angle, -180.0f, 180.0f);
            is_clamped = (std::abs(current_cmd.target_angle) > 180.0f);
            gm6020_motor->setRef(clamped_angle);
        }
        else if (current_cmd.motor_type == "J4310" && j4310_motor) {
            clamped_angle = std::clamp(current_cmd.target_angle, 0.0f, 360.0f);
            is_clamped = (current_cmd.target_angle < 0.0f || current_cmd.target_angle > 360.0f);
            j4310_motor->set_target_angle(clamped_angle);
        }
        else if (current_cmd.motor_type == "J8009" && j8009_motor) {
            clamped_angle = std::clamp(current_cmd.target_angle, -90.0f, 90.0f);
            is_clamped = (std::abs(current_cmd.target_angle) > 90.0f);
            j8009_motor->set_target_angle(clamped_angle);
        }
        else {
            std::string err = "Error: Motor " + current_cmd.motor_type + " not found!\r\n";
            uart_send_with_retry(usart1, err);
            vTaskDelay(5);
            continue;
        }

        if (is_clamped) {
            std::string warn = current_cmd.motor_type + ": Angle clamped to " + std::to_string(clamped_angle) + "° (exceed limit)\r\n";
            uart_send_with_retry(usart1, warn);
        }

        vTaskDelay(5);
    }
}
