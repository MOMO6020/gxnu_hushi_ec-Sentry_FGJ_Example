/*
 * @Author       : Notch-FGJ mail.fgj.com@gmail.com
 * @Date         : 2025-03-28 11:57:06
 * @LastEditors  : Notch-FGJ mail.fgj.com@gmail.com
 * @LastEditTime : 2025-04-01 14:13:15
 * @FilePath     : \FrameworkC_CMake\bsp\log\log.hpp
 * @Description  : 日志系统
 */
//? 孩子不懂事封装着玩的, 经过测试, 大概没问题
//* 设置ENABLE_RTT_LOG=0关闭日志系统以节省空间
#pragma once
#include <string>

#ifndef ENABLE_RTT_LOG
#define LOGINFO(node_name, message)
#define LOGDEBUG(node_name, message)
#define LOGWARNING(node_name, message)
#define LOGERROR(node_name, message)
#else
#define LOGINFO(node_name, message) RTTLog::info(node_name, message)
#define LOGDEBUG(node_name, message) RTTLog::debug(node_name, message)
#define LOGWARNING(node_name, message) RTTLog::warning(node_name, message)
#define LOGERROR(node_name, message) RTTLog::error(node_name, message)
#endif

class RTTLog
{
public:
    static void init(void);
    static void debug(const std::string &node_name, const std::string &message);
    static void info(const std::string &node_name, const std::string &message);
    static void warning(const std::string &node_name, const std::string &message);
    static void error(const std::string &node_name, const std::string &message);
};