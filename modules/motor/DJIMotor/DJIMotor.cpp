/*
 * @Author       : Notch-FGJ mail.fgj.com@gmail.com
 * @Date         : 2025-04-07 13:59:32
 * @LastEditors  : Notch-FGJ mail.fgj.com@gmail.com
 * @LastEditTime : 2025-11-28 16:00:00
 * @FilePath     : \gxnu_hushi_ec\modules\motor\DJIMotor\DJIMotor.cpp
 * @Description  : 大疆电机驱动（最终编译通过版本）
 */
#include "DJIMotor.hpp"
#include "bsp/can/stm32_can.hpp"
#include "bsp/log/log.hpp"
#include "cmsis_os2.h"
#include <cstring>
#include <cmath>
#include <algorithm>

#define SPEED_SMOOTH_COEF 0.9f
#define CURRENT_SMOOTH_COEF 0.9f
#define LOG_INTERVAL_MS 500  // 日志间隔

// 【关键修复】删除静态成员的重复定义（.hpp中inline static已完成定义+初始化）

// 构造函数：初始化电机，绑定CAN回调
DJIMotor::DJIMotor(CAN_HandleTypeDef* _hcan,
                   uint16_t           _motor_id,
                   MotorType          _motor_type,
                   MotorPIDSetting    _setting,
                   MotorPID           _pid_config)
    : IMotor(_motor_type, _setting, _pid_config),  // 初始化父类
      motor_id_(_motor_id),
      motor_type_(_motor_type),
      setting_(_setting),
      pid_config_(_pid_config),
      enable_(false),
      is_online_(false),
      pid_out_(0)
{
    // 检查电机数量上限
    if (motor_count_ >= MAX_DJIMOTOR_COUNT)
    {
        char err_msg[64];
        snprintf(err_msg, sizeof(err_msg), "Motor count exceeds limit (%d)", MAX_DJIMOTOR_COUNT);
        LOGERROR("DJIMotor", err_msg);
        while (true) osDelay(1000);
    }

    // 从参数表读取电机参数
    auto param_it = motor_params_.find(_motor_type);
    if (param_it != motor_params_.end())
    {
        const auto& params = param_it->second;
        reduction_ratio_ = params.reduction_ratio;
        max_angle_ = params.max_angle;
        min_angle_ = params.min_angle;
        encoder_res_ = params.encoder_res;
        max_output_current_ = params.max_current;
        // 计算控制ID和反馈ID（基础值+电机ID）
        ctrl_can_id_ = params.ctrl_id_base + _motor_id;
        feedback_can_id_ = params.feedback_id_base + _motor_id;
    }
    else
    {
        // 默认参数（无效电机类型）
        LOGWARNING("DJIMotor", "Invalid motor type, use default params");
        reduction_ratio_ = 1.0f;
        max_angle_ = 180.0f;
        min_angle_ = -180.0f;
        encoder_res_ = 8192.0f;
        max_output_current_ = 20000;
        ctrl_can_id_ = 0x200 + _motor_id;
        feedback_can_id_ = 0x200 + _motor_id;
    }

    // 绑定CAN接收回调（接收电机反馈数据）
    auto can = STM32CAN_GetInstance(_hcan);
    can->setRxCallback(feedback_can_id_, std::bind(&DJIMotor::decode, this, std::placeholders::_1, std::placeholders::_2));
    
    // 日志：用snprintf拼接字符串（适配LOGINFO仅2个参数）
    char info_msg[128];
    snprintf(info_msg, sizeof(info_msg), "Init Motor#%d: CtrlID=0x%03X, FeedbackID=0x%03X",
             motor_id_, ctrl_can_id_, feedback_can_id_);
    LOGINFO("DJIMotor", info_msg);

    // 初始化发送组（根据电机类型和CAN总线）
    if (motor_type_ == MotorType::GM6020)
    {
        motor_tx_group_ = (_hcan->Instance == CAN1) ? 2 : 6;  // GM6020 1-4→组2（CAN1）/6（CAN2）
        if (_motor_id > 4)
            motor_tx_group_++;  // GM6020 5-8→组3（CAN1）/7（CAN2）
    }
    else
    {
        motor_tx_group_ = (_hcan->Instance == CAN1) ? 0 : 4;  // M/J系列 1-4→组0（CAN1）/4（CAN2）
        if (_motor_id > 4)
            motor_tx_group_++;  // M/J系列 5-8→组1（CAN1）/5（CAN2）
    }

    // 检查控制ID冲突
    for (uint8_t i = 0; i < motor_count_; i++)
    {
        if (registered_motors_[i] && registered_motors_[i]->ctrl_can_id_ == ctrl_can_id_)
        {
            char err_msg[64];
            snprintf(err_msg, sizeof(err_msg), "CtrlID 0x%03X conflict", ctrl_can_id_);
            LOGERROR("DJIMotor", err_msg);
            while (true) osDelay(1000);
        }
    }

    // 注册电机
    registered_motors_[motor_count_++] = this;

    // 初始化测量数据（用赋值代替memset，避免非平凡类型警告）
    measure_ = Measure{};
}

// 解码CAN反馈数据（修正角度计算和日志）
void DJIMotor::decode(const uint8_t* buf, const uint8_t len)
{
    if (buf == nullptr || len < 8) return;

    // 标记在线状态
    if (!is_online_)
    {
        is_online_ = true;
        char info_msg[64];
        snprintf(info_msg, sizeof(info_msg), "Motor#%d Online (Feedback received)", motor_id_);
        LOGINFO("DJIMotor", info_msg);
    }
    daemon_->feed();  // 喂守护进程

    // 解析编码器值
    measure_.last_encoder = measure_.encoder;
    measure_.encoder = (buf[0] << 8) | buf[1];

    // 计算单圈机械角度（编码器值→度，除以减速比）
    measure_.angle = (static_cast<float>(measure_.encoder) / encoder_res_) * 360.0f / reduction_ratio_;

    // 解析转速（RPM→度/秒，除以减速比，平滑滤波）
    int16_t raw_speed = (buf[2] << 8) | buf[3];
    measure_.speed_dps = (1.0f - SPEED_SMOOTH_COEF) * measure_.speed_dps +
                         SPEED_SMOOTH_COEF * static_cast<float>(raw_speed) * 6.0f / reduction_ratio_;
    measure_.speed = raw_speed;

    // 解析电流（平滑滤波）
    int16_t raw_current = (buf[4] << 8) | buf[5];
    measure_.torque_current = (1.0f - CURRENT_SMOOTH_COEF) * measure_.torque_current +
                              CURRENT_SMOOTH_COEF * static_cast<float>(raw_current);

    // 解析温度
    measure_.temperature = buf[6];

    // 处理圈数溢出（编码器半量程判断）
    if (measure_.encoder - measure_.last_encoder > static_cast<int16_t>(encoder_res_ / 2))
        measure_.total_round--;
    else if (measure_.last_encoder - measure_.encoder > static_cast<int16_t>(encoder_res_ / 2))
        measure_.total_round++;

    // 计算总机械角度
    measure_.total_angle = measure_.total_round * 360.0f + measure_.angle;

    // 反转处理
    if (setting_.reverse)
    {
        measure_.total_angle = -measure_.total_angle;
        measure_.speed_dps = -measure_.speed_dps;
    }

    // 实时状态日志（500ms一次）
    static uint32_t last_log_tick = 0;
    if (osKernelGetTickCount() - last_log_tick > LOG_INTERVAL_MS)
    {
        char log_msg[128];
        // 修正：torque_current是int类型，用%d格式化
        snprintf(log_msg, sizeof(log_msg), "Motor#%d: Angle=%.1f°, Speed=%.0f°/s, Temp=%d°C, Current=%dmA",
                 motor_id_, measure_.total_angle, measure_.speed_dps, measure_.temperature, measure_.torque_current);
        LOGINFO("DJIMotor", log_msg);
        last_log_tick = osKernelGetTickCount();
    }

    // 测试用回调
    if (calculateCallback) calculateCallback();
}

// 离线回调
void DJIMotor::offlineCallback()
{
    if (is_online_)
    {
        is_online_ = false;
        measure_ = Measure{};  // 用赋值代替memset
        char warn_msg[64];
        snprintf(warn_msg, sizeof(warn_msg), "Motor#%d Offline (No feedback)", motor_id_);
        LOGWARNING("DJIMotor", warn_msg);
    }
}

// 设置目标角度（带软件限位）
void DJIMotor::setTargetAngle(float target_angle)
{
    // 限位处理
    target_angle_ = std::clamp(target_angle, min_angle_, max_angle_);
    // 同步到PID参考值
    pid_ref_ = target_angle_;
}

// 计算输出电流（核心逻辑：修正PID、强制测试电流）
int16_t DJIMotor::calculateOutputCurrent()
{
    // 未使能或离线，输出0
    if (!enable_ || !is_online_)
    {
        pid_out_ = 0;
        return 0;
    }

    float pid_measure = 0.0f;
    float pid_ref = pid_ref_;

    // -------------------------- 临时测试：强制输出电流（优先测试硬件）--------------------------
    // 注释下面的PID逻辑，打开这行，直接输出1000mA（GM6020可承受，测试电机是否动）
    // pid_out_ = 1000;  // 正数正转，负数反转，测试后注释此行
    // return pid_out_;
    // -----------------------------------------------------------------------------------

    // 角度环控制（目标角度→速度指令）
    if ((setting_.close_loop & CloseloopType::ANGLE_LOOP) && setting_.outer_loop == CloseloopType::ANGLE_LOOP)
    {
        // 反馈角度：使用电机总机械角度
        pid_measure = measure_.total_angle;

        // PID计算（目标角度和反馈角度均为机械角度，无需乘减速比）
        pid_ref = pidControllers_.pid_angle_.PIDCalculate(pid_measure, pid_ref);

        // 角度误差日志
        static uint32_t last_angle_log = 0;
        if (osKernelGetTickCount() - last_angle_log > LOG_INTERVAL_MS)
        {
            char log_msg[128];
            snprintf(log_msg, sizeof(log_msg), "Motor#%d: Target=%.1f°, Current=%.1f°, Error=%.1f°",
                     motor_id_, target_angle_, measure_.total_angle, target_angle_ - measure_.total_angle);
            LOGINFO("DJIMotor", log_msg);
            last_angle_log = osKernelGetTickCount();
        }
    }

    // 速度环控制（速度指令→电流指令）
    if ((setting_.close_loop & CloseloopType::SPEED_LOOP) &&
        (static_cast<uint8_t>(setting_.outer_loop) & (static_cast<uint8_t>(CloseloopType::ANGLE_LOOP) | static_cast<uint8_t>(CloseloopType::SPEED_LOOP))))
    {
        // 反馈速度：使用电机机械速度（度/秒）
        pid_measure = measure_.speed_dps;

        // PID计算（速度指令→电流指令）
        pid_ref = pidControllers_.pid_speed_.PIDCalculate(pid_measure, pid_ref);
    }

    // 电流环控制（可选，根据配置）
    if (setting_.close_loop & CloseloopType::CURRENT_LOOP)
    {
        pid_measure = measure_.torque_current;
        pid_ref = pidControllers_.pid_current_.PIDCalculate(pid_measure, pid_ref);
    }

    // 电流限幅（根据电机最大电流）
    pid_ref = std::clamp(pid_ref, static_cast<float>(-max_output_current_), static_cast<float>(max_output_current_));
    pid_out_ = static_cast<int16_t>(pid_ref);

    // 输出电流日志
    static uint32_t last_current_log = 0;
    if (osKernelGetTickCount() - last_current_log > LOG_INTERVAL_MS)
    {
        char log_msg[64];
        snprintf(log_msg, sizeof(log_msg), "Motor#%d: Output Current=%dmA", motor_id_, pid_out_);
        LOGINFO("DJIMotor", log_msg);
        last_current_log = osKernelGetTickCount();
    }

    return pid_out_;
}

// 全局CAN句柄声明（确保在其他文件中定义hcan1/hcan2）
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

// 电机控制主函数：统一发送CAN控制帧（核心修正）
void DJIMotor::DJIMotorControl()
{
    static CANHandle_t can1_handle = STM32CAN_GetInstance(&hcan1);
    static CANHandle_t can2_handle = STM32CAN_GetInstance(&hcan2);

    // 1. 清空控制数据（仅清空数据，保留ID和类型）
    for (auto& packet : control_data_)
    {
        memset(packet.data, 0, 8);
        // 移除：ICAN::ClassicPacket没有len成员，DJI电机固定8字节
    }

    // 2. 遍历所有电机，填充控制数据
    for (uint8_t i = 0; i < motor_count_; i++)
    {
        DJIMotor* motor = registered_motors_[i];
        if (!motor || !motor->is_online_ || !motor->enable_)
            continue;

        // 计算输出电流
        int16_t output_current = motor->calculateOutputCurrent();
        if (output_current == 0)
            continue;

        // 确定电流填充位置（1-4号电机→0/2/4/6字节，5-8号→1/3/5/7字节）
        uint8_t idx = (motor->motor_id_ - 1) * 2;
        if (motor->motor_id_ > 4)
            idx = (motor->motor_id_ - 5) * 2 + 1;

        // 关键：根据电机类型设置控制掩码（CAN帧高2字节）
        if (motor->motor_type_ == MotorType::GM6020)
        {
            // GM6020：高2字节=0x0003（启用位置环+速度环）
            control_data_[motor->motor_tx_group_].data[0] = 0x00;
            control_data_[motor->motor_tx_group_].data[1] = 0x03;
            // 电流填充到对应位置
            control_data_[motor->motor_tx_group_].data[idx] = output_current >> 8;
            control_data_[motor->motor_tx_group_].data[idx + 1] = output_current & 0xFF;
        }
        else
        {
            // M/J系列：高2字节=0x0000（电流模式）
            control_data_[motor->motor_tx_group_].data[0] = 0x00;
            control_data_[motor->motor_tx_group_].data[1] = 0x00;
            // 电流填充到对应位置
            control_data_[motor->motor_tx_group_].data[idx] = output_current >> 8;
            control_data_[motor->motor_tx_group_].data[idx + 1] = output_current & 0xFF;
        }
    }

    // 3. 发送CAN数据（打印发送日志，调试用）
    for (uint8_t group = 0; group < 8; group++)
    {
        // 检查是否有非零数据（避免发送空帧）
        bool has_data = false;
        for (uint8_t i = 0; i < 8; i++)
        {
            if (control_data_[group].data[i] != 0)
            {
                has_data = true;
                break;
            }
        }
        if (!has_data)
            continue;

        // 打印发送日志（修正：ID是uint32_t，用%03lX格式化）
        char can_log[128];
        snprintf(can_log, sizeof(can_log), "Group%d: ID=0x%03lX, Data=[0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X]",
                 group, static_cast<long unsigned int>(control_data_[group].id),
                 control_data_[group].data[0], control_data_[group].data[1],
                 control_data_[group].data[2], control_data_[group].data[3],
                 control_data_[group].data[4], control_data_[group].data[5],
                 control_data_[group].data[6], control_data_[group].data[7]);
        LOGINFO("CAN Send", can_log);

        // 发送数据（CAN1→组0-3，CAN2→组4-7）
        if (group < 4)
            can1_handle->transmit(control_data_[group]);
        else
            can2_handle->transmit(control_data_[group]);
    }
}