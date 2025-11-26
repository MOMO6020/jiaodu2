/*
 * @Author       : Notch-FGJ mail.fgj.com@gmail.com
 * @Date         : 2025-04-07 10:59:49
 * @LastEditors  : Notch-FGJ mail.fgj.com@gmail.com
 * @LastEditTime : 2025-11-28 15:00:00
 * @FilePath     : \gxnu_hushi_ec\modules\motor\DJIMotor\DJIMotor.hpp
 * @Description  : 大疆电机驱动（修复编译错误）
 */
#pragma once
#include "../motor_def.hpp"
#include <array>
#include <unordered_map>

#include "bsp/can/stm32_can.hpp"
#include "modules/daemon/daemon.hpp"
#include "modules/motor/IMotor.hpp"

/**
 * @class DJIMotor
 * @brief 大疆电机类, 仅计算电机输出电流, 不控制电机, 使用MotorController类控制电机
 */
class DJIMotor : public IMotor
{
public:
    // 电机参数结构体（补充最大电流、CAN ID基础值）
    struct MotorParams {
        float reduction_ratio;  // 减速比
        float max_angle;        // 最大角度限制(度)
        float min_angle;        // 最小角度限制(度)
        float encoder_res;      // 编码器分辨率（线数）
        int16_t max_current;    // 最大输出电流（mA）
        uint16_t ctrl_id_base;  // 控制帧CAN ID基础值（GM6020=0x1FF，其他=0x200）
        uint16_t feedback_id_base; // 反馈帧CAN ID基础值（GM6020=0x204，其他=0x200）
    };

    struct Measure
    {
        uint16_t last_encoder   = 0;  ///< 上次编码器值(0-8191)
        uint16_t encoder        = 0;  ///< 编码器值(0-8191)
        int16_t  speed          = 0;  ///< 电机转速(RPM)
        int16_t  torque_current = 0;  ///< 电机电流
        uint8_t  temperature    = 0;  ///< 电机温度
        float    speed_dps      = 0;  ///< 电机转速(degree/s)
        float    angle          = 0;  ///< 电机单圈角度(degree)
        float    total_angle    = 0;  ///< 总转动角度(degree)
        int32_t  total_round    = 0;  ///< 电机转动圈数
    };

    DJIMotor(CAN_HandleTypeDef* _hcan,
             uint16_t           _motor_id,
             MotorType          _motor_type,
             MotorPIDSetting    _setting,
             MotorPID           _pid_config);
    
    void decode(const uint8_t* buf, const uint8_t len) override;  // 仅继承的虚函数加override
    void offlineCallback() override;  // 仅继承的虚函数加override
    int16_t calculateOutputCurrent() override;  // 仅继承的虚函数加override
    
    std::function<void()> calculateCallback = nullptr;  // 测试用回调函数

    static void DJIMotorControl();

    // 设置目标角度接口（应用层调用）
    void setTargetAngle(float target_angle);

    // 获取当前总角度（供应用层打印调试）
    float getCurrentAngle() const { return measure_.total_angle; }

    // 新增：获取输出电流（供调试日志）
    int16_t getOutputCurrent() const { return pid_out_; }

    // 新增：使能/禁用电机（不继承，去掉override）
    void enable() { enable_ = true; }
    void disable() { enable_ = false; }
    bool isEnabled() const { return enable_; }
    bool isOnline() const { return is_online_; }

    Measure measure_;  ///< 电机测量数据

private:
    uint8_t motor_id_;                    ///< 电机ID
    uint8_t motor_tx_group_ = 0;          ///< 电机发送组ID
    inline static std::array<DJIMotor*, MAX_DJIMOTOR_COUNT> registered_motors_{nullptr};  ///< 电机数组
    inline static uint8_t motor_count_ = 0;             ///< 电机数量

    MotorType motor_type_;          ///< 当前电机型号
    MotorPIDSetting setting_;       ///< 电机控制设置
    MotorPID pid_config_;           ///< PID配置
    float target_angle_ = 0.0f;     ///< 目标角度
    bool enable_ = false;           ///< 电机使能标志（新增）
    bool is_online_ = false;        ///< 电机在线标志（新增）
    int16_t pid_out_ = 0;           ///< PID输出电流（新增）
    
    // 电机参数（从参数表读取）
    float reduction_ratio_ = 1.0f;  ///< 减速比
    float max_angle_ = 180.0f;      ///< 最大角度限制（度）
    float min_angle_ = -180.0f;     ///< 最小角度限制（度）
    float encoder_res_ = 8192.0f;   ///< 编码器分辨率（线数）
    int16_t max_output_current_ = 20000;  ///< 最大输出电流（mA）
    uint16_t ctrl_can_id_ = 0;      ///< 控制帧CAN ID（计算后的值，如0x204）
    uint16_t feedback_can_id_ = 0;  ///< 反馈帧CAN ID（计算后的值，如0x209）

    // 电机参数表（更新：补充最大电流、CAN ID基础值）
    inline static const std::unordered_map<MotorType, MotorParams> motor_params_ = {
        {MotorType::GM6020, {19.0f,  180.0f, -180.0f, 8192.0f, 30000, 0x1FF, 0x204}},
        {MotorType::J4310,  {30.0f,   90.0f,  -90.0f, 8192.0f, 10000, 0x200, 0x200}},
        {MotorType::J8009,  {40.0f,  360.0f,    0.0f, 8192.0f, 20000, 0x200, 0x200}},
        {MotorType::M3508,  {19.0f,    0.0f,    0.0f, 8192.0f, 20000, 0x200, 0x200}},
        {MotorType::M2006,  {36.0f,    0.0f,    0.0f, 8192.0f, 10000, 0x200, 0x200}}
    };

    /**
     * @brief 存储DJI电机控制数据的静态数组
     * 分组说明：
     * CAN1: [0]: 0x200（M2006/M3508 1-4）, [1]: 0x1FF（M2006/M3508 5-8）, [2]: 0x1FE（GM6020 1-4）, [3]: 0x2FE（GM6020 5-8）
     * CAN2: [4]: 0x200, [5]: 0x1FF, [6]: 0x1FE, [7]: 0x2FE
     */
    inline static std::array<ICAN::ClassicPacket, 8> control_data_{
        // CAN1:
        ICAN::ClassicPacket{0x200, ICAN::Type::STANDARD, {0, 0, 0, 0, 0, 0, 0, 0}},
        ICAN::ClassicPacket{0x1FF, ICAN::Type::STANDARD, {0, 0, 0, 0, 0, 0, 0, 0}},
        ICAN::ClassicPacket{0x1FE, ICAN::Type::STANDARD, {0, 0, 0, 0, 0, 0, 0, 0}},
        ICAN::ClassicPacket{0x2FE, ICAN::Type::STANDARD, {0, 0, 0, 0, 0, 0, 0, 0}},
        // CAN2:
        ICAN::ClassicPacket{0x200, ICAN::Type::STANDARD, {0, 0, 0, 0, 0, 0, 0, 0}},
        ICAN::ClassicPacket{0x1FF, ICAN::Type::STANDARD, {0, 0, 0, 0, 0, 0, 0, 0}},
        ICAN::ClassicPacket{0x1FE, ICAN::Type::STANDARD, {0, 0, 0, 0, 0, 0, 0, 0}},
        ICAN::ClassicPacket{0x2FE, ICAN::Type::STANDARD, {0, 0, 0, 0, 0, 0, 0, 0}},
    };
};