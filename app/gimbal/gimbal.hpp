#pragma once
#include "app/app_def.hpp"
#include "bsp/log/log.hpp"
#include "bsp/system/mutex.hpp"
#include "bsp/uart/stm32_uart.hpp"
#include "modules/imu/ins_task.h"
#include "modules/motor/DJIMotor/DJIMotor.hpp"
#include "modules/motor/DMMotor/DMMotor.hpp"
#include "modules/umt/Message.hpp"
#include "modules/pid/PIDController.hpp"  // 确保包含PIDController头文件
#include "stm32f4xx_hal_uart.h"
#include <cstdio>  // 包含snprintf所需头文件
#include <memory>

// For Test:
UARTHandle_t              uart6 = nullptr;
extern UART_HandleTypeDef huart6;

class Gimbal
{
public:
    // 构造函数：初始化GM6020（yaw轴和pitch轴均适配GM6020，按需调整ID和CAN句柄）
    Gimbal(attitude_t* imu_data, std::shared_ptr<float> gimbal_yaw_motor_angle_ptr) : imu_data_(imu_data)
    {
        uart6 = STM32UART_Init(&huart6);
        if (gimbal_yaw_motor_angle_ptr == nullptr)
        {
            while (true)
            {
                LOGERROR("Gimbal", "gimbal_yaw_motor_angle_ptr is nullptr");
                STM32TimeDWT::Delay(1000);
            }
        }
        gimbal_yaw_motor_angle_ptr_ = gimbal_yaw_motor_angle_ptr;
        extern CAN_HandleTypeDef hcan1;  // 确保hcan1已在bsp层初始化

        // Yaw轴GM6020初始化（适配GM6020，PID配置与PIDController结构体完全匹配）
        motor_yaw_ = std::make_shared<DJIMotor>(
            &hcan1,                                  // 硬件CAN句柄（与电机实际连接的CAN通道一致）
            1,                                       // 电机ID（1-7，必须与拨码开关设置完全匹配）
            MotorType::GM6020,                       // 明确电机类型为GM6020
            MotorPIDSetting{
                .outer_loop              = CloseloopType::ANGLE_LOOP,  // 外环：角度环（目标是指定角度）
                .close_loop              = CloseloopType::ANGLE_AND_SPEED_LOOP,  // 双环控制（角度+速度，更稳定）
                .reverse                 = false,  // 转向：输出轴端看CCW为正（按实际安装调整，反了就改true）
                .external_angle_feedback = FeedbackType::INTERNAL,  // 关键：用GM6020自身编码器（不用IMU）
                .external_speed_feedback = FeedbackType::INTERNAL    // 速度反馈也用电机内部数据
            },
            MotorPID{
                // 角度环PID（适配GM6020 1.2N·m扭矩，定位精度0.05°）
                .pid_angle_ = PIDController(
                    PIDConfig{
                        .Kp = 40.0f,                    // 比例增益
                        .Ki = 0.8f,                     // 积分增益
                        .Kd = 1.5f,                     // 微分增益
                        .MaxOutput = 30000.0f,          // 最大输出（匹配GM6020 CAN指令范围：-30000~30000）
                        .Deadband = 0.0f,               // 死区（未启用，设0）
                        .Improve = PIDImprovement::PID_Integral_Limit,  // 启用积分限幅
                        .IntegralLimit = 8000.0f,       // 积分限幅（防止积分饱和）
                        .CoefA = 0.0f,                  // 变速积分系数A（未启用，设0）
                        .CoefB = 0.0f,                  // 变速积分系数B（未启用，设0）
                        .Output_LPF_RC = 200.0f,        // 输出滤波RC系数（降低输出波动）
                        .Derivative_LPF_RC = 0.0f       // 微分滤波系数（未启用，设0）
                    }),
                // 速度环PID（适配GM6020最大320rpm转速）
                .pid_speed_ = PIDController(
                    PIDConfig{
                        .Kp = 25.0f,                    // 比例增益
                        .Ki = 1.2f,                     // 积分增益
                        .Kd = 0.2f,                     // 微分增益
                        .MaxOutput = 30000.0f,          // 最大输出（匹配CAN指令范围）
                        .Deadband = 0.0f,               // 死区（未启用，设0）
                        .Improve = PIDImprovement::PID_Integral_Limit,  // 启用积分限幅
                        .IntegralLimit = 1500.0f,       // 积分限幅（防止积分饱和）
                        .CoefA = 0.0f,                  // 变速积分系数A（未启用，设0）
                        .CoefB = 0.0f,                  // 变速积分系数B（未启用，设0）
                        .Output_LPF_RC = 8000.0f,       // 输出滤波RC系数
                        .Derivative_LPF_RC = 0.0f       // 微分滤波系数（未启用，设0）
                    }),
                .pid_angle_feedback_ptr_ = nullptr,  // 禁用外部角度反馈（用电机自身编码器）
                .pid_speed_feedback_ptr_ = nullptr   // 禁用外部速度反馈（用电机自身数据）
            });

        // Pitch轴GM6020初始化（适配GM6020，PID配置与PIDController结构体完全匹配）
        motor_pitch_ = std::make_shared<DJIMotor>(
            &hcan1,                                  // 若pitch轴接CAN2，改为&hcan2
            2,                                       // pitch轴电机ID（与拨码开关一致，不可与yaw轴冲突）
            MotorType::GM6020,
            MotorPIDSetting{
                .outer_loop              = CloseloopType::ANGLE_LOOP,
                .close_loop              = CloseloopType::ANGLE_AND_SPEED_LOOP,
                .reverse                 = true,   // pitch轴安装方向可能相反，默认true（按实际调整）
                .external_angle_feedback = FeedbackType::INTERNAL,  // 用GM6020自身编码器
                .external_speed_feedback = FeedbackType::INTERNAL
            },
            MotorPID{
                .pid_angle_ = PIDController(
                    PIDConfig{
                        .Kp = 38.0f,                    // 比例增益
                        .Ki = 0.7f,                     // 积分增益
                        .Kd = 1.2f,                     // 微分增益
                        .MaxOutput = 30000.0f,          // 最大输出（匹配CAN指令范围）
                        .Deadband = 0.0f,               // 死区（未启用，设0）
                        .Improve = PIDImprovement::PID_Integral_Limit,  // 启用积分限幅
                        .IntegralLimit = 7000.0f,       // 积分限幅（防止积分饱和）
                        .CoefA = 0.0f,                  // 变速积分系数A（未启用，设0）
                        .CoefB = 0.0f,                  // 变速积分系数B（未启用，设0）
                        .Output_LPF_RC = 200.0f,        // 输出滤波RC系数
                        .Derivative_LPF_RC = 0.0f       // 微分滤波系数（未启用，设0）
                    }),
                .pid_speed_ = PIDController(
                    PIDConfig{
                        .Kp = 22.0f,                    // 比例增益
                        .Ki = 1.0f,                     // 积分增益
                        .Kd = 0.15f,                    // 微分增益
                        .MaxOutput = 30000.0f,          // 最大输出（匹配CAN指令范围）
                        .Deadband = 0.0f,               // 死区（未启用，设0）
                        .Improve = PIDImprovement::PID_Integral_Limit,  // 启用积分限幅
                        .IntegralLimit = 1200.0f,       // 积分限幅（防止积分饱和）
                        .CoefA = 0.0f,                  // 变速积分系数A（未启用，设0）
                        .CoefB = 0.0f,                  // 变速积分系数B（未启用，设0）
                        .Output_LPF_RC = 8000.0f,       // 输出滤波RC系数
                        .Derivative_LPF_RC = 0.0f       // 微分滤波系数（未启用，设0）
                    }),
                .pid_angle_feedback_ptr_ = nullptr,  // 禁用外部角度反馈
                .pid_speed_feedback_ptr_ = nullptr   // 禁用外部速度反馈
            });

        // 初始化后强制使能电机（防止未使能导致不动）
        motor_yaw_->enable();
        motor_pitch_->enable();

        gimbal_cmd_sub_.bind("gimbal_cmd");
        LOGINFO("Gimbal", "GM6020 Yaw(PID1) + Pitch(PID2) Initialized (CAN:hcan1)");
    }

   // 控制任务：处理角度指令，适配GM6020编码器值
    void GimbalTask()
    {
        try
        {
            gimbal_cmd cmd = gimbal_cmd_sub_.pop();
            if (cmd.force_stop)
            {
                motor_yaw_->disable();
                motor_pitch_->disable();
                LOGINFO("Gimbal", "Motor stopped (force_stop)");
            }
            else
            {
                motor_yaw_->enable();
                motor_pitch_->enable();
    
                // 1. 限制角度范围（避免超出机械限位和编码器累计错误）
                float yaw_target_deg = std::clamp(cmd.yaw, -180.0f, 180.0f);    // yaw轴±180°（可根据机械结构调整）
                float pitch_target_deg = std::clamp(cmd.pitch, -90.0f, 90.0f);  // pitch轴±90°（防止机械碰撞）
    
                // 2. 核心转换：角度（°）→ GM6020编码器值（0-8191，对应0-360°）
                // GM6020编码器分辨率8192，1° = 8192/360 ≈ 22.755个编码器值
                float yaw_target_encoder = (yaw_target_deg / 360.0f) * 8192.0f;
                float pitch_target_encoder = (pitch_target_deg / 360.0f) * 8192.0f;
    
                // 3. 设定电机目标角度（若DJIMotor类用setRef接收目标值，替换为setRef）
                motor_yaw_->setTargetAngle(yaw_target_encoder);
                motor_pitch_->setTargetAngle(pitch_target_encoder);
    
                // 4. 打印日志：验证指令转换是否正确（调试用）
                char gimbal_log[256];
                snprintf(gimbal_log, sizeof(gimbal_log), 
                         "Target: Yaw=%.1f°→%.0f编码器值, Pitch=%.1f°→%.0f编码器值 | Current: Yaw=%d, Pitch=%d",
                         yaw_target_deg, yaw_target_encoder,
                         pitch_target_deg, pitch_target_encoder,
                         motor_yaw_->measure_.encoder,  // 电机当前编码器值（int类型→%d）
                         motor_pitch_->measure_.encoder);
                LOGINFO("Gimbal", gimbal_log);
            }
    
            // 5. 更新外部角度指针（编码器值→角度°，供外部读取）
            *gimbal_yaw_motor_angle_ptr_ = (motor_yaw_->measure_.encoder / 8192.0f) * 360.0f;
    
        }
        catch (umt::MessageError& e)
        {
            vTaskDelay(1);
        }
    }
private:
    umt::Subscriber<gimbal_cmd> gimbal_cmd_sub_;
    std::shared_ptr<DJIMotor>   motor_yaw_                  = nullptr;
    std::shared_ptr<DJIMotor>   motor_pitch_                = nullptr;
    std::shared_ptr<float>      gimbal_yaw_motor_angle_ptr_ = nullptr;
    attitude_t*                 imu_data_                   = nullptr;
};