// ------------ 强制包含STM32核心头文件（解决CAN_HandleTypeDef问题）------------
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"  // 直接包含CAN相关定义
#include "stm32f4xx_hal_uart.h" // 直接包含UART相关定义
#include "cmsis_gcc.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"

// ------------ 工程框架头文件 ------------
#include "app_main.h"
#include "bsp/system/mutex.hpp"
#include "FreeRTOS.h"
#include "task.h"

// ------------ 外设和模块头文件 ------------
#include "bsp/log/log.hpp"
#include "bsp/system/time.hpp"
#include "bsp/uart/stm32_uart.hpp"
#include "modules/imu/ins_task.h"
#include "modules/motor/motor.hpp"
#include "modules/motor/DMMotor/DMMotor.hpp"
#include "modules/motor/DJIMotor/DJIMotor.hpp"

// ------------ C++标准库 ------------
#include <memory>
#include <string>
#include <algorithm>
#include <cstdio>
#include <cmath>
#include <functional>  // 确保std::function可用

// ------------ 全局对象声明 ------------
// 外部外设句柄（必须确保main.c中已声明这两个全局变量）
extern UART_HandleTypeDef huart1;
extern CAN_HandleTypeDef  hcan1;

// 串口句柄
UARTHandle_t              usart1  = nullptr;

// 电机对象（智能指针管理）
std::shared_ptr<DJIMotor> gm6020_motor = nullptr;
std::shared_ptr<DMMotor>  j4310_motor  = nullptr;
std::shared_ptr<DMMotor>  j8009_motor  = nullptr;

// 串口指令结构体（简化版，避免std::string兼容问题）
typedef struct {
    char motor_type[16];  // 用C字符串替代std::string，彻底避免volatile和权限问题
    float target_angle;
    bool is_valid;
} SerialAngleCmd;

// 全局指令（用C结构体，无兼容问题）
SerialAngleCmd g_serial_cmd = {0};

// ------------ 函数声明 ------------
void uart1_callback(uint8_t* buf, uint32_t len);
SerialAngleCmd parse_serial_cmd(const char* cmd_str);
void uart_send_with_retry(UARTHandle_t uart, const char* msg);
void my_clamp_float(float* value, float min_val, float max_val);  // 用C风格 clamp
void uart_feedback_task(void* arg);
void motor_control_task(void* arg);

// ------------ 工具函数实现 ------------
// C风格clamp（无模板，避免兼容性问题）
void my_clamp_float(float* value, float min_val, float max_val) {
    if (*value < min_val) *value = min_val;
    else if (*value > max_val) *value = max_val;
}

// 串口发送（带重试，稳定可靠）
void uart_send_with_retry(UARTHandle_t uart, const char* msg) {
    if (uart == nullptr || msg == nullptr) return;
    uint32_t retry = 3;
    uint32_t len = strlen(msg);
    while (retry-- > 0) {
        ErrorCode err = uart->transmit((uint8_t*)msg, len);
        if (err == ErrorCode::OK) break;
        vTaskDelay(1);
    }
}

// ------------ 核心初始化函数 ------------
void app_main() {
    taskENTER_CRITICAL();

    // 1. 基础初始化
    RTTLog::init();
    STM32TimeDWT::DWT_Init(168);
    LOGINFO("SerialMotor", "System Init Start");

    // 2. CAN初始化
    STM32CAN_Init();
    StartMotorControlTask();

    // 3. 串口初始化（兼容两种回调类型：函数指针/ std::function）
#ifdef USE_STD_FUNCTION_CALLBACK
    usart1 = STM32UART_Init(&huart1, std::bind(&uart1_callback, std::placeholders::_1, std::placeholders::_2));
#else
    usart1 = STM32UART_Init(&huart1, uart1_callback, nullptr);
#endif
    if (usart1 == nullptr) {
        LOGERROR("SerialMotor", "USART1 init failed!");
    } else {
        LOGINFO("SerialMotor", "USART1 Ready!");
        uart_send_with_retry(usart1, "Serial Motor Control Ready!\r\n");
        uart_send_with_retry(usart1, "Cmd Format: GM6020:90 / J4310:180 / J8009:-45\r\n");
    }

    // 4. GM6020电机初始化
    gm6020_motor = std::make_shared<DJIMotor>(
        &hcan1, 1, MotorType::GM6020,
        MotorPIDSetting{
            .outer_loop = CloseloopType::ANGLE_LOOP,
            .close_loop = CloseloopType::ANGLE_AND_SPEED_LOOP,
            .reverse    = false,
            .external_angle_feedback = FeedbackType::INTERNAL,
            .external_speed_feedback = FeedbackType::INTERNAL,
        },
        MotorPID{
            .pid_angle_ = PIDController(PIDConfig{35.0f, 0.8f, 1.2f, 5000.0f, 300.0f, PIDImprovement::PID_Integral_Limit, 180.0f}),
            .pid_speed_ = PIDController(PIDConfig{12.0f, 0.2f, 0.1f, 16384.0f, 1000.0f, PIDImprovement::PID_Integral_Limit}),
        }
    );

    // 5. J4310电机初始化（达妙）
    j4310_motor = std::make_shared<DMMotor>(
        &hcan1, 2, 2 + 0x200, MotorType::J4310,
        MotorPIDSetting{
            .outer_loop = CloseloopType::ANGLE_LOOP,
            .close_loop = CloseloopType::ANGLE_AND_SPEED_LOOP,
            .reverse = false,
            .external_angle_feedback = FeedbackType::INTERNAL,
            .external_speed_feedback = FeedbackType::INTERNAL
        },
        MotorPID{
            .pid_angle_ = PIDController(PIDConfig{25.0f, 0.3f, 0.8f, 1000.0f, 200.0f, PIDImprovement::PID_Integral_Limit}),
            .pid_speed_ = PIDController(PIDConfig{8.0f, 0.1f, 0.05f, 3000.0f, 500.0f, PIDImprovement::PID_Integral_Limit})
        }
    );

    // 6. J8009电机初始化（达妙）
    j8009_motor = std::make_shared<DMMotor>(
        &hcan1, 3, 3 + 0x200, MotorType::J8009,
        MotorPIDSetting{
            .outer_loop = CloseloopType::ANGLE_LOOP,
            .close_loop = CloseloopType::ANGLE_AND_SPEED_LOOP,
            .reverse = true,
            .external_angle_feedback = FeedbackType::INTERNAL,
            .external_speed_feedback = FeedbackType::INTERNAL
        },
        MotorPID{
            .pid_angle_ = PIDController(PIDConfig{22.0f, 0.4f, 0.7f, 900.0f, 180.0f, PIDImprovement::PID_Integral_Limit}),
            .pid_speed_ = PIDController(PIDConfig{7.0f, 0.15f, 0.03f, 2800.0f, 450.0f, PIDImprovement::PID_Integral_Limit})
        }
    );

    // 7. 使能电机
    if (gm6020_motor) gm6020_motor->enable();
    if (j4310_motor)  j4310_motor->enable();
    if (j8009_motor)  j8009_motor->enable();

    // 8. 创建任务
    xTaskCreate(uart_feedback_task, "uart_feedback", 512,  nullptr, 2, nullptr);
    xTaskCreate(motor_control_task, "motor_control", 1024, nullptr, 3, nullptr);

    taskEXIT_CRITICAL();
    LOGINFO("SerialMotor", "Init Complete! Waiting for CMD...");
}

// ------------ 串口回调函数（C风格，无std::string）------------
void uart1_callback(uint8_t* buf, uint32_t len) {
    if (buf == nullptr || len == 0) return;

    // 1. 缓冲区处理（C风格，避免std::string）
    char cmd_buf[64] = {0};
    len = (len > 63) ? 63 : len;
    memcpy(cmd_buf, buf, len);

    // 2. 清除无效字符
    for (uint32_t i = 0; i < len; i++) {
        if (cmd_buf[i] == '\r' || cmd_buf[i] == '\n' || cmd_buf[i] == ' ') {
            cmd_buf[i] = '\0';
            break;
        }
    }

    // 3. 解析指令
    SerialAngleCmd cmd = parse_serial_cmd(cmd_buf);
    if (cmd.is_valid) {
        // 临界区保护
        taskENTER_CRITICAL();
        strncpy(g_serial_cmd.motor_type, cmd.motor_type, sizeof(g_serial_cmd.motor_type)-1);
        g_serial_cmd.target_angle = cmd.target_angle;
        g_serial_cmd.is_valid = true;
        taskEXIT_CRITICAL();

        // 反馈
        char ack[64];
        snprintf(ack, sizeof(ack), "Received: %s=%.1f°\r\n", cmd.motor_type, cmd.target_angle);
        uart_send_with_retry(usart1, ack);
    } else {
        uart_send_with_retry(usart1, "Error: Invalid Cmd! Use 'MotorType:Angle' (e.g., J4310:90)\r\n");
    }
}

// ------------ 指令解析函数（C风格）------------
SerialAngleCmd parse_serial_cmd(const char* cmd_str) {
    SerialAngleCmd cmd = {0};
    char* colon_ptr = strchr(cmd_str, ':');
    if (colon_ptr == nullptr || colon_ptr == cmd_str || *(colon_ptr+1) == '\0') {
        return cmd;
    }

    // 提取电机类型
    uint32_t type_len = colon_ptr - cmd_str;
    if (type_len > sizeof(cmd.motor_type)-1) type_len = sizeof(cmd.motor_type)-1;
    strncpy(cmd.motor_type, cmd_str, type_len);
    cmd.motor_type[type_len] = '\0';

    // 提取角度（C风格转换，避免std::stof异常）
    char* angle_str = colon_ptr + 1;
    char* end_ptr = nullptr;
    float angle = strtof(angle_str, &end_ptr);
    if (end_ptr != angle_str) {  // 转换成功
        // 校验电机类型
        if (strcmp(cmd.motor_type, "GM6020") == 0 || 
            strcmp(cmd.motor_type, "J4310") == 0 || 
            strcmp(cmd.motor_type, "J8009") == 0) {
            cmd.target_angle = angle;
            cmd.is_valid = true;
        }
    }

    return cmd;
}

// ------------ 串口反馈任务（直接读取DMMotor的measure_成员，绕开函数权限）------------
void uart_feedback_task(void* arg __attribute__((unused))) {
    while (true) {
        if (usart1 == nullptr) {
            vTaskDelay(100);
            continue;
        }

        char feedback[128] = "Current Angles: ";
        size_t feedback_len = strlen(feedback);

        // GM6020
        if (gm6020_motor) {
            char gm_buf[32];
            snprintf(gm_buf, sizeof(gm_buf), "GM6020=%.1f°; ", gm6020_motor->measure_.total_angle);
            strncat(feedback, gm_buf, sizeof(feedback)-feedback_len-1);
            feedback_len = strlen(feedback);
        }

        // J4310（直接访问measure_成员，绕开get_current_angle函数权限问题）
        if (j4310_motor) {
            char j4_buf[32];
            snprintf(j4_buf, sizeof(j4_buf), "J4310=%.1f°; ", j4310_motor->measure_.total_angle);
            strncat(feedback, j4_buf, sizeof(feedback)-feedback_len-1);
            feedback_len = strlen(feedback);
        }

        // J8009（直接访问measure_成员）
        if (j8009_motor) {
            char j8_buf[32];
            snprintf(j8_buf, sizeof(j8_buf), "J8009=%.1f°; ", j8009_motor->measure_.total_angle);
            strncat(feedback, j8_buf, sizeof(feedback)-feedback_len-1);
        }

        strncat(feedback, "\r\n", sizeof(feedback)-1);
        uart_send_with_retry(usart1, feedback);
        vTaskDelay(500);
    }
}

// ------------ 电机控制任务（直接调用setRef，绕开set_target_angle函数权限）------------
void motor_control_task(void* arg __attribute__((unused))) {
    while (true) {
        // 读取指令
        SerialAngleCmd current_cmd = {0};
        taskENTER_CRITICAL();
        if (g_serial_cmd.is_valid) {
            strncpy(current_cmd.motor_type, g_serial_cmd.motor_type, sizeof(current_cmd.motor_type)-1);
            current_cmd.target_angle = g_serial_cmd.target_angle;
            current_cmd.is_valid = true;
            g_serial_cmd.is_valid = false;
        }
        taskEXIT_CRITICAL();

        if (!current_cmd.is_valid) {
            vTaskDelay(5);
            continue;
        }

        float target_angle = current_cmd.target_angle;
        char warn[64] = {0};

        // GM6020
        if (strcmp(current_cmd.motor_type, "GM6020") == 0 && gm6020_motor) {
            my_clamp_float(&target_angle, -180.0f, 180.0f);
            gm6020_motor->setRef(target_angle);
            if (fabs(current_cmd.target_angle) > 180.0f) {
                snprintf(warn, sizeof(warn), "GM6020: Clamped to %.1f° (±180°)\r\n", target_angle);
            }
        }
        // J4310（直接调用基类setRef，绕开set_target_angle权限）
        else if (strcmp(current_cmd.motor_type, "J4310") == 0 && j4310_motor) {
            my_clamp_float(&target_angle, 0.0f, 360.0f);
            j4310_motor->setRef(target_angle);  // 直接调用基类IMotor的setRef（必须是public）
            if (current_cmd.target_angle < 0.0f || current_cmd.target_angle > 360.0f) {
                snprintf(warn, sizeof(warn), "J4310: Clamped to %.1f° (0-360°)\r\n", target_angle);
            }
        }
        // J8009（直接调用基类setRef）
        else if (strcmp(current_cmd.motor_type, "J8009") == 0 && j8009_motor) {
            my_clamp_float(&target_angle, -90.0f, 90.0f);
            j8009_motor->setRef(target_angle);  // 直接调用基类IMotor的setRef
            if (fabs(current_cmd.target_angle) > 90.0f) {
                snprintf(warn, sizeof(warn), "J8009: Clamped to %.1f° (±90°)\r\n", target_angle);
            }
        }
        // 未知电机
        else {
            char err[64];
            snprintf(err, sizeof(err), "Error: Motor %s Not Found!\r\n", current_cmd.motor_type);
            uart_send_with_retry(usart1, err);
            vTaskDelay(5);
            continue;
        }

        // 发送警告
        if (warn[0] != '\0') {
            uart_send_with_retry(usart1, warn);
        }

        vTaskDelay(5);
    }
}