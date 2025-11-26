/*
 * @Author       : Notch-FGJ mail.fgj.com@gmail.com
 * @Date         : 2025-03-30 21:26:02
 * @LastEditors  : Notch-FGJ mail.fgj.com@gmail.com
 * @LastEditTime : 2025-11-29 10:00:00
 * @FilePath     : \FrameworkA_FGJ\bsp\can\stm32_can.hpp
 * @Description  : CAN总线板级支持包（GM6020适配版，仅针对C板）
 */
#pragma once
#include "main.h"
#include "bsp/base_def.hpp"
#include <functional>
#include <memory>

class ICAN
{
public:
    /**
     * @enum Type
     * @brief CAN消息类型（GM6020仅使用STANDARD标准帧）
     */
    enum class Type : uint8_t
    {
        STANDARD = 0,        ///< 标准帧（GM6020控制/反馈均使用）
        EXTENDED = 1,        ///< 扩展帧（未使用）
        REMOTE_STANDARD = 2, ///< 远程标准帧（未使用）
        REMOTE_EXTENDED = 3, ///< 远程扩展帧（未使用）
    };

    /**
     * @struct ClassicPacket
     * @brief 经典CAN数据包结构体（GM6020固定8字节数据）
     * @details 该结构体用于存储经典CAN数据包的ID、类型和数据载荷
     */
    struct ClassicPacket
    {
        uint32_t id = 0;                     ///< 消息ID（GM6020控制ID：0x1FF/0x2FF，反馈ID：0x205~0x20B）
        Type type = Type::STANDARD;          ///< 消息类型（默认标准帧）
        uint8_t data[8] = {0};               ///< 数据载荷（GM6020固定8字节）
    };

    virtual ~ICAN() = default;

    /**
     * @brief 将CAN消息添加到发送队列（GM6020需发送8字节数据）
     * @param packet 要发送的数据包
     * @param timeout 超时时间(ms)
     * @return 操作结果
     */
    virtual ErrorCode transmit(const ClassicPacket &packet, uint32_t timeout = 1000) = 0;

    /**
     * @brief 添加对应RX_ID的回调函数（绑定GM6020反馈ID的解析函数）
     * @param rx_id 接收ID（GM6020反馈ID：0x204+电机ID）
     * @param callback 回调函数（DJIMotor::decode）
     */
    virtual ErrorCode setRxCallback(uint16_t rx_id, std::function<void(const uint8_t *, const uint8_t)> callback) = 0;
};

using CANHandle_t = std::shared_ptr<ICAN>;

/**
 * @brief STM32CAN初始化函数, 若未初始化则创建实例并返回
 * @param _hcan CAN句柄（&hcan1 或 &hcan2）
 * @return CANHandle_t STM32CAN对象的智能指针（供DJIMotor绑定回调和发送数据）
 */
CANHandle_t STM32CAN_GetInstance(CAN_HandleTypeDef *_hcan);

/**
 * @brief 初始化CAN1和CAN2实例（系统启动时调用）
 */
void STM32CAN_Init();