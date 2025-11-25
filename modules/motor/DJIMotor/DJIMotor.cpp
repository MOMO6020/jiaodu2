/*
 * @Author       : Notch-FGJ mail.fgj.com@gmail.com
 * @Date         : 2025-04-07 13:59:32
 * @LastEditors  : Notch-FGJ mail.fgj.com@gmail.com
 * @LastEditTime : 2025-11-27 11:00:00
 * @FilePath     : \gxnu_hushi_ec\modules\motor\DJIMotor\DJIMotor.cpp
 * @Description  : 大疆电机驱动（新增GM6020/J4310/J8009位置控制+软件限位）
 */
#include "DJIMotor.hpp"
#include "bsp/can/stm32_can.hpp"
#include "bsp/log/log.hpp"
#include "cmsis_os2.h"
#include <cstring>
#include <cmath>
#include <algorithm>  // 补充：用于std::clamp（若编译器支持C++17）

#define SPEED_SMOOTH_COEF 0.9f
#define CURRENT_SMOOTH_COEF 0.9f

/**
 * @brief 电机组使能标志
 * 与control_data_数组一一对应, 当对应组存在电机时, 使能标志为1
 * 用于防止发送空数据包
 */
bool group_enable_flag[8] = {false, false, false, false, false, false, false, false};

// 电机参数结构体（存储不同电机的固定参数）
struct MotorFixedParams {
    float reduce_rate;    // 减速比
    float max_angle;      // 最大角度限制（度）
    float min_angle;      // 最小角度限制（度）
    float encoder_res;    // 编码器分辨率（线数）
    int16_t max_current;  // 最大输出电流（mA）
};

// 电机参数表（对应MotorType枚举，顺序与MotorType定义一致）
const MotorFixedParams motor_fixed_params[] = {
    {19.0f,  180.0f, -180.0f, 8192.0f, 30000},  // GM6020：减速比19，限位±180度，最大电流30000mA
    {19.0f,    0.0f,    0.0f, 8192.0f, 20000},  // M3508：原有参数，不启用限位
    {36.0f,    0.0f,    0.0f, 8192.0f, 10000},  // M2006：原有参数，不启用限位
    {30.0f,   90.0f,  -90.0f, 8192.0f, 10000},  // J4310：减速比30，限位±90度，最大电流10000mA
    {40.0f,  360.0f,    0.0f, 8192.0f, 20000}   // J8009：减速比40，限位0-360度，最大电流20000mA
};

// 构造函数：初始化电机，绑定CAN回调，读取固定参数
DJIMotor::DJIMotor(CAN_HandleTypeDef* _hcan,
                   uint16_t           _motor_id,
                   MotorType          _motor_type,
                   MotorPIDSetting    _setting,
                   MotorPID           _pid_config)
    : IMotor(_motor_type, _setting, _pid_config), 
      motor_id_(_motor_id),
      motor_type_(_motor_type),
      setting_(_setting),
      pid_config_(_pid_config)
{
    if (motor_count_ >= MAX_DJIMOTOR_COUNT)
    {
        LOGERROR("DJIMotor", "Motor count exceeds maximum limit");
        while (true)
            osDelay(1000);
    }

    // 初始化当前电机的固定参数（从参数表读取）
    uint8_t type_idx = static_cast<uint8_t>(_motor_type);
    if (type_idx < sizeof(motor_fixed_params)/sizeof(MotorFixedParams)) {
        const auto& params = motor_fixed_params[type_idx];
        reduction_ratio_ = params.reduce_rate;  // 对应hpp中的reduction_ratio_
        max_angle_ = params.max_angle;
        min_angle_ = params.min_angle;
        encoder_res_ = params.encoder_res;
        max_output_current_ = params.max_current;
    } else {
        // 默认参数（防止无效电机类型）
        reduction_ratio_ = 1.0f;
        max_angle_ = 180.0f;
        min_angle_ = -180.0f;
        encoder_res_ = 8192.0f;
        max_output_current_ = 20000;
        LOGWARNING("DJIMotor", "Invalid motor type, use default params");
    }

    auto     can    = STM32CAN_GetInstance(_hcan);
    uint16_t can_id = motor_type_ == MotorType::GM6020 ? 0x204 + motor_id_ : 0x200 + motor_id_;
    can->setRxCallback(can_id, std::bind(&DJIMotor::decode, this, std::placeholders::_1, std::placeholders::_2));

    // 初始化发送组（兼容J4310/J8009，复用原有分组逻辑）
    switch (motor_type_)
    {
        case MotorType::M2006:
        case MotorType::M3508:
            motor_tx_group_ = motor_id_ <= 4 ? 0 : 1;
            motor_tx_group_ = _hcan->Instance == CAN1 ? motor_tx_group_ : motor_tx_group_ + 4;
            break;
        case MotorType::GM6020:
            motor_tx_group_ = _hcan->Instance == CAN1 ? 2 : 3;
            motor_tx_group_ = motor_id_ <= 4 ? motor_tx_group_ : motor_tx_group_ + 4;
            break;
        case MotorType::J4310:
        case MotorType::J8009:
            // 复用M2006/M3508的分组逻辑（CAN1:0/1，CAN2:4/5）
            motor_tx_group_ = motor_id_ <= 4 ? 0 : 1;
            motor_tx_group_ = _hcan->Instance == CAN1 ? motor_tx_group_ : motor_tx_group_ + 4;
            break;
        default: 
            LOGERROR("DJIMotor", "Invalid motor type"); 
            while (true) osDelay(1000);
            break;
    }

    // 检查ID冲突
    for (uint8_t i = 0; i < motor_count_; i++)
    {
        if (registered_motors_[i] != nullptr && 
            registered_motors_[i]->motor_tx_group_ == motor_tx_group_ && 
            registered_motors_[i]->motor_id_ == motor_id_)
        {
            LOGERROR("DJIMotor", "Motor ID already exists");
            while (true)
                osDelay(1000);
        }
    }

    registered_motors_[motor_count_++] = this;  // 添加到电机数组
    group_enable_flag[motor_tx_group_] = true;  // 设置使能标志

    // 绑定角度PID反馈指针（hpp中已改为float*，直接赋值地址）
    pidControllers_.pid_angle_feedback_ptr_ = &measure_.total_angle;
}

// 解码CAN数据：解析编码器、速度、电流等信息
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
        // 电机在线日志（支持所有型号）
        switch (motor_type_)
        {
            case MotorType::GM6020: LOGINFO("Motor GM6020#" + std::to_string(motor_id_), "Online"); break;
            case MotorType::M3508: LOGINFO("Motor M3508#" + std::to_string(motor_id_), "Online"); break;
            case MotorType::M2006: LOGINFO("Motor M2006#" + std::to_string(motor_id_), "Online"); break;
            case MotorType::J4310: LOGINFO("Motor J4310#" + std::to_string(motor_id_), "Online"); break;
            case MotorType::J8009: LOGINFO("Motor J8009#" + std::to_string(motor_id_), "Online"); break;
            default: LOGINFO("Motor Unknown#" + std::to_string(motor_id_), "Online"); break;
        }
    }
    daemon_->feed();
    measure_.last_encoder = measure_.encoder;
    measure_.encoder      = ((uint16_t)buf[0] << 8) | buf[1];

    // 统一角度计算（基于编码器分辨率，兼容所有电机）
    measure_.angle = (static_cast<float>(measure_.encoder) / encoder_res_) * 360.0f;

    measure_.speed        = (buf[2] << 8) | buf[3];
    measure_.speed_dps =
        (1.0f - SPEED_SMOOTH_COEF) * measure_.speed_dps + 6.0f * SPEED_SMOOTH_COEF * (float)measure_.speed;
    measure_.torque_current = (1.0f - CURRENT_SMOOTH_COEF) * measure_.torque_current +
                              CURRENT_SMOOTH_COEF * (float)((int16_t)(buf[4] << 8 | buf[5]));
    measure_.temperature = buf[6];

    // 圈数累计（解决编码器溢出）
    if (measure_.encoder - measure_.last_encoder > 4096)
        measure_.total_round--;
    else if (measure_.last_encoder - measure_.encoder > 4096)
        measure_.total_round++;

    // 总角度计算（圈数×360 + 单圈角度，支持多圈限位）
    measure_.total_angle = measure_.total_round * 360.0f + measure_.angle;

    // 反转处理（如果配置了reverse，翻转实际角度）
    if (setting_.reverse) {
        measure_.total_angle = -measure_.total_angle;
        measure_.speed_dps = -measure_.speed_dps;
    }
}

// 离线回调：电机离线时清空数据并打印日志
void DJIMotor::offlineCallback()
{
    if (is_online)
    {
        memset(static_cast<void*>(&measure_), 0, sizeof(measure_));
        is_online = false;
        // 电机离线日志（支持所有型号）
        switch (motor_type_)
        {
            case MotorType::GM6020: LOGWARNING("Motor GM6020#" + std::to_string(motor_id_), "Offline"); break;
            case MotorType::M3508: LOGWARNING("Motor M3508#" + std::to_string(motor_id_), "Offline"); break;
            case MotorType::M2006: LOGWARNING("Motor M2006#" + std::to_string(motor_id_), "Offline"); break;
            case MotorType::J4310: LOGWARNING("Motor J4310#" + std::to_string(motor_id_), "Offline"); break;
            case MotorType::J8009: LOGWARNING("Motor J8009#" + std::to_string(motor_id_), "Offline"); break;
            default: LOGWARNING("Motor Unknown#" + std::to_string(motor_id_), "Offline"); break;
        }
    }
}

// 设置目标角度（带软件限位，与hpp声明一致）
void DJIMotor::setTargetAngle(float target_angle)
{
    // 软件限位（提前限制目标角度，避免后续PID计算溢出）
    float limited_angle = target_angle;
    auto it = motor_params_.find(motor_type_);
    if (it != motor_params_.end()) {
        const auto& params = it->second;
        // 手动限位（兼容低版本C++，替代std::clamp）
        if (limited_angle > params.max_angle) limited_angle = params.max_angle;
        else if (limited_angle < params.min_angle) limited_angle = params.min_angle;
    }

    this->target_angle_ = limited_angle;
    this->pid_ref_ = limited_angle;  // 同步到PID参考值
}

// 计算电机输出电流（核心控制逻辑）
int16_t DJIMotor::calculateOutputCurrent()
{
    if (!is_online || !enable_)
        return 0;

    float pid_measure;
    float pid_ref = pid_ref_;

    // 1. 软件限位（仅对角度环生效）
    if ((setting_.close_loop & CloseloopType::ANGLE_LOOP) && setting_.outer_loop == CloseloopType::ANGLE_LOOP) {
        // 手动限位（替代std::clamp，兼容所有C++版本）
        if (pid_ref > max_angle_) pid_ref = max_angle_;
        else if (pid_ref < min_angle_) pid_ref = min_angle_;

        // 调试日志（1秒打印一次，避免日志刷屏）
        static uint32_t last_log_tick = 0;
        if (osKernelGetTickCount() - last_log_tick > 1000) {
            char log_msg[128];
            snprintf(log_msg, sizeof(log_msg), "Target: %.2f°, Current: %.2f°", 
                     pid_ref, measure_.total_angle);
            LOGINFO("Motor#" + std::to_string(motor_id_), log_msg);
            last_log_tick = osKernelGetTickCount();
        }
    }

    // 原有：反转处理
    if (setting_.reverse)
        pid_ref *= -1.0f;

    // 位置环计算
    if ((setting_.close_loop & CloseloopType::ANGLE_LOOP) && setting_.outer_loop == CloseloopType::ANGLE_LOOP)
    {
        if (setting_.external_angle_feedback == FeedbackType::EXTERNAL) {
            if (pidControllers_.pid_angle_feedback_ptr_ != nullptr)
                pid_measure = *pidControllers_.pid_angle_feedback_ptr_;
            else
            {
                LOGERROR("DJIMotor", "Feedback pointer is null");
                return 0;
            }
        } else {
            pid_measure = measure_.total_angle;
        }

        // PID计算（目标角度×减速比，兼容原有逻辑）
        pid_ref = pidControllers_.pid_angle_.PIDCalculate(pid_measure, pid_ref * reduction_ratio_);
    }

    // 速度环计算
    if ((setting_.close_loop & CloseloopType::SPEED_LOOP) &&
        (uint8_t(setting_.outer_loop) & (CloseloopType::ANGLE_LOOP | CloseloopType::SPEED_LOOP)))
    {
        if (setting_.external_speed_feedback == FeedbackType::EXTERNAL) {
            if (pidControllers_.pid_speed_feedback_ptr_ != nullptr)
                pid_measure = *pidControllers_.pid_speed_feedback_ptr_;
            else
            {
                LOGERROR("DJIMotor", "Feedback pointer is null");
                return 0;
            }
        } else {
            pid_measure = measure_.speed_dps;
        }
        pid_ref = pidControllers_.pid_speed_.PIDCalculate(pid_measure, pid_ref * reduction_ratio_);
    }

    // 电流环计算
    if (setting_.close_loop & CloseloopType::CURRENT_LOOP)
    {
        pid_ref = pidControllers_.pid_current_.PIDCalculate(measure_.torque_current, pid_ref);
    }

    // 2. 电流限幅（根据电机型号限制最大输出电流）
    int16_t min_current = -max_output_current_;
    int16_t max_current = max_output_current_;
    if (pid_ref < min_current) pid_ref = min_current;
    else if (pid_ref > max_current) pid_ref = max_current;
    pid_out_ = static_cast<int16_t>(pid_ref);

    return pid_out_;
}

// 全局CAN句柄声明（需确保在其他文件中定义）
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

// 电机控制主函数：遍历所有电机，计算电流并发送CAN数据
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
        if (motor == nullptr || !motor->is_online)
            continue;

        // 计算电流并填充数据
        int16_t output_current = motor->calculateOutputCurrent();
        uint8_t idx            = motor->motor_id_ <= 4 ? 2 * (motor->motor_id_ - 1) : 2 * (motor->motor_id_ - 5);
        control_data_[motor->motor_tx_group_].data[idx]     = output_current >> 8;
        control_data_[motor->motor_tx_group_].data[idx + 1] = output_current & 0xFF;

        // 标记组有效（无需重复检查）
        group_enable_flag[motor->motor_tx_group_] = true;
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