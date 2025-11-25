/*
 * @Author       : Notch-FGJ mail.fgj.com@gmail.com
 * @Date         : 2025-04-15 13:43:59
 * @LastEditors  : Notch-FGJ mail.fgj.com@gmail.com
 * @LastEditTime : 2025-11-27 11:30:00
 * @FilePath     : \gxnu_hushi_ec\modules\motor\motor_def.hpp
 * @Description  : 电机相关枚举和结构体定义
 */
#pragma once
#include "modules/pid/PIDController.hpp"
#include <memory>

// 闭环控制类型枚举
enum class CloseloopType : uint8_t
{
    OPEN_LOOP    = 0b00000000,  ///< 开环控制
    CURRENT_LOOP = 0b00000001,  ///< 电流闭环控制
    SPEED_LOOP   = 0b00000010,  ///< 速度闭环控制
    ANGLE_LOOP   = 0b00000100,  ///< 角度闭环控制

    SPEED_AND_CURRENT_LOOP = 0b00000011,  ///< 速度和电流闭环控制
    ANGLE_AND_SPEED_LOOP   = 0b00000110,  ///< 角度和速度闭环控制
    ALL_THREE_LOOP         = 0b00000111,  ///< 三个闭环控制
};

// 电机类型枚举（顺序与参数表一致）
enum class MotorType : uint8_t
{
    GM6020,
    M3508,
    M2006,
    J4310,
    J8009
};

// 反馈类型枚举
enum class FeedbackType : uint8_t
{
    INTERNAL,  ///< 内部反馈（电机自带编码器）
    EXTERNAL,  ///< 外部反馈（外接传感器）
};

// 电机PID控制设置结构体
struct MotorPIDSetting
{
    CloseloopType outer_loop;               ///< 外环闭环类型
    CloseloopType close_loop;               ///< 串级闭环类型
    bool          reverse = false;          ///< 是否反转电机输出
    FeedbackType  external_angle_feedback;  ///< 角度反馈类型
    FeedbackType  external_speed_feedback;  ///< 速度反馈类型（修复语法错误：移除多余下划线）
};

// 电机PID配置结构体（修改智能指针为普通指针，解决类型不匹配）
struct MotorPID
{
    PIDController  pid_angle_              = PIDController();  ///< 角度PID控制器
    PIDController  pid_speed_              = PIDController();  ///< 速度PID控制器
    PIDController  pid_current_            = PIDController();  ///< 电流PID控制器
    float*         pid_angle_feedback_ptr_ = nullptr;          ///< 角度PID反馈指针（改为普通指针）
    float*         pid_speed_feedback_ptr_ = nullptr;          ///< 速度PID反馈指针（改为普通指针）
};