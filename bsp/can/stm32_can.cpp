/*
 * @Author       : Notch-FGJ mail.fgj.com@gmail.com
 * @Date         : 2025-03-30 22:11:35
 * @LastEditors  : Notch-FGJ mail.fgj.com@gmail.com
 * @LastEditTime : 2025-11-29 16:00:00
 * @FilePath     : \FrameworkA_FGJ\bsp\can\stm32_can.cpp
 * @Description  : CAN总线封装（GM6020适配版，修复编译错误）
 */
#include "stm32_can.hpp"
#include "bsp/log/log.hpp"
#include "bsp/system/time.hpp"
#include "cmsis_os2.h"
#include <cstdio>  // 包含snprintf所需头文件

class STM32CAN : public ICAN
{
public:
    STM32CAN(CAN_HandleTypeDef* _hcan) : hcan_(_hcan)
    {
        if (hcan_->Instance == CAN1)  // CAN1 - FIFO0
            fifo_ = CAN_RX_FIFO0;
        else if (hcan_->Instance == CAN2)  // CAN2 - FIFO1
            fifo_ = CAN_RX_FIFO1;
        else
            while (true)
            {
                LOGERROR("CAN", "CAN Instance Error");
                osDelay(1000);
            }

        // -------------- 核心修改1：初始化过滤器为掩码模式（匹配GM6020反馈ID范围）--------------
        CAN_FilterTypeDef can_filter;
        can_filter.FilterIdHigh         = 0x204 << 5;  // 基础ID：0x204（GM6020反馈ID=0x204+电机ID）
        can_filter.FilterIdLow          = 0x0000;
        can_filter.FilterMode           = CAN_FILTERMODE_IDMASK;  // 掩码模式（匹配ID范围）
        can_filter.FilterScale          = CAN_FILTERSCALE_32BIT;  // 32位过滤器
        can_filter.FilterMaskIdHigh     = 0xFFF << 5;  // 掩码：全1，精确匹配ID高11位（标准帧）
        can_filter.FilterMaskIdLow      = 0x0000;
        can_filter.FilterFIFOAssignment = fifo_;
        can_filter.FilterActivation     = ENABLE;
        can_filter.FilterBank           = (fifo_ == CAN_RX_FIFO0) ? 0 : 14;  // CAN1用过滤器0-13，CAN2用14-27
        can_filter.SlaveStartFilterBank = 14;

        if (HAL_CAN_ConfigFilter(hcan_, &can_filter) != HAL_OK)
        {
            LOGERROR("CAN", "CAN Filter Config Failed");
            while (true) osDelay(1000);
        }

        // 启动CAN并激活中断
        if (HAL_CAN_Start(hcan_) != HAL_OK)
        {
            LOGERROR("CAN", "CAN Start Failed");
            while (true) osDelay(1000);
        }

        // 只激活当前FIFO的接收中断和错误中断
        if (fifo_ == CAN_RX_FIFO0)
            HAL_CAN_ActivateNotification(hcan_, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR);
        else
            HAL_CAN_ActivateNotification(hcan_, CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_ERROR);
    }

    ~STM32CAN() override = default;

    ErrorCode transmit(const ClassicPacket& packet, uint32_t timeout = 1000) override
    {
        CAN_TxHeaderTypeDef tx_header;
        tx_header.DLC = sizeof(packet.data);  // GM6020固定8字节数据
        switch (packet.type)
        {
            case Type::STANDARD:
                tx_header.IDE = CAN_ID_STD;
                tx_header.RTR = CAN_RTR_DATA;
                break;
            case Type::EXTENDED:
                tx_header.IDE = CAN_ID_EXT;
                tx_header.RTR = CAN_RTR_DATA;
                break;
            case Type::REMOTE_STANDARD:
                tx_header.IDE = CAN_ID_STD;
                tx_header.RTR = CAN_RTR_REMOTE;
                break;
            case Type::REMOTE_EXTENDED:
                tx_header.IDE = CAN_ID_EXT;
                tx_header.RTR = CAN_RTR_REMOTE;
                break;
            default: return ErrorCode::INVALID;
        }
        tx_header.StdId              = (packet.type == Type::EXTENDED) ? 0 : packet.id;
        tx_header.ExtId              = (packet.type == Type::EXTENDED) ? packet.id : 0;
        tx_header.TransmitGlobalTime = DISABLE;

        uint64_t start_time = STM32TimeDWT::GetMilliseconds();
        while (HAL_CAN_GetTxMailboxesFreeLevel(hcan_) == 0)
        {
            if (STM32TimeDWT::GetMilliseconds() - start_time > timeout)
            {
                // 修复：格式符%X→%lX（适配uint32_t）
                char warn_msg[64];
                snprintf(warn_msg, sizeof(warn_msg), "Transmit Timeout (ID:0x%03lX)", packet.id);
                LOGWARNING("CAN", warn_msg);
                return ErrorCode::TIMEOUT;
            }
            osDelay(1);
        }

        if (HAL_CAN_AddTxMessage(hcan_, &tx_header, packet.data, &tx_mailbox_) != HAL_OK)
        {
            // 修复：格式符%X→%lX（适配uint32_t）
            char err_msg[64];
            snprintf(err_msg, sizeof(err_msg), "Transmit Failed (ID:0x%03lX)", packet.id);
            LOGERROR("CAN", err_msg);
            return ErrorCode::FAILED;
        }
        return ErrorCode::OK;
    }

    ErrorCode setRxCallback(uint16_t rx_id, std::function<void(const uint8_t*, const uint8_t)> callback) override
    {
        if (callback == nullptr)
        {
            LOGERROR("CAN", "Callback is nullptr");
            return ErrorCode::INVALID;
        }
        callbackMap[rx_id] = callback;
        // 修复：格式符%X→%lX（适配uint16_t，兼容长整数）
        char dbg_msg[64];
        snprintf(dbg_msg, sizeof(dbg_msg), "Set Rx Callback: ID=0x%03lX", rx_id);
        LOGDEBUG("CAN", dbg_msg);
        return ErrorCode::OK;
    }

    std::unordered_map<uint16_t, std::function<void(const uint8_t*, const uint8_t)>> callbackMap;  ///< 回调函数映射表

private:
    // -------------- 核心修改2：删除updateRxFilter函数（掩码模式无需动态更新过滤器）--------------

private:
    CAN_HandleTypeDef* hcan_;
    uint32_t           fifo_;
    uint32_t           tx_mailbox_;
};

// 全局CAN实例（初始化为nullptr）
std::shared_ptr<STM32CAN> __can_handle_1 = nullptr;
std::shared_ptr<STM32CAN> __can_handle_2 = nullptr;

// -------------- 核心修改3：修复CAN实例获取函数（返回已初始化的实例）--------------
CANHandle_t STM32CAN_GetInstance(CAN_HandleTypeDef* _hcan)
{
    if (_hcan == nullptr)
    {
        LOGERROR("CAN", "GetInstance: hcan is nullptr");
        return nullptr;
    }

    // 根据CAN实例选择对应的全局句柄
    auto& handle = (_hcan->Instance == CAN1) ? __can_handle_1 : __can_handle_2;
    // 若实例未初始化，先初始化
    if (handle == nullptr)
    {
        handle = std::make_shared<STM32CAN>(_hcan);
        // 修复：用snprintf替换std::format
        char info_msg[64];
        snprintf(info_msg, sizeof(info_msg), "CAN%s Instance Created", (_hcan->Instance == CAN1) ? "1" : "2");
        LOGINFO("CAN", info_msg);
    }
    return handle;
}

// 初始化两个CAN实例（在系统启动时调用）
void STM32CAN_Init()
{
    extern CAN_HandleTypeDef hcan1;
    extern CAN_HandleTypeDef hcan2;

    if (__can_handle_1 == nullptr)
    {
        __can_handle_1 = std::make_shared<STM32CAN>(&hcan1);
        LOGINFO("CAN", "CAN1 Init Success");
    }
    if (__can_handle_2 == nullptr)
    {
        __can_handle_2 = std::make_shared<STM32CAN>(&hcan2);
        LOGINFO("CAN", "CAN2 Init Success");
    }
}

// CAN1 FIFO0接收中断回调
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    if (hcan->Instance != CAN1 || __can_handle_1 == nullptr) return;

    CAN_RxHeaderTypeDef rx_header;
    uint8_t fifo0_buffer[8] = {0};
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, fifo0_buffer) != HAL_OK)
    {
        LOGERROR("CAN1", "FIFO0 Receive Failed");
        return;
    }

    // 查找对应ID的回调函数并执行（GM6020反馈ID=0x205~0x20B）
    auto it = __can_handle_1->callbackMap.find(rx_header.StdId);
    if (it != __can_handle_1->callbackMap.end())
    {
        it->second(fifo0_buffer, rx_header.DLC);
        // 修复：格式符%X→%lX（适配uint32_t）
        char rx0_msg[128];
        snprintf(rx0_msg, sizeof(rx0_msg), "Received ID:0x%03lX, Data:[0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X]",
                 rx_header.StdId,
                 fifo0_buffer[0], fifo0_buffer[1], fifo0_buffer[2], fifo0_buffer[3],
                 fifo0_buffer[4], fifo0_buffer[5], fifo0_buffer[6], fifo0_buffer[7]);
        LOGDEBUG("CAN1", rx0_msg);
    }
}

// CAN2 FIFO1接收中断回调
extern "C" void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    if (hcan->Instance != CAN2 || __can_handle_2 == nullptr) return;

    CAN_RxHeaderTypeDef rx_header;
    uint8_t fifo1_buffer[8] = {0};
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, fifo1_buffer) != HAL_OK)
    {
        LOGERROR("CAN2", "FIFO1 Receive Failed");
        return;
    }

    auto it = __can_handle_2->callbackMap.find(rx_header.StdId);
    if (it != __can_handle_2->callbackMap.end())
    {
        it->second(fifo1_buffer, rx_header.DLC);
        // 修复：格式符%X→%lX（适配uint32_t）
        char rx1_msg[128];
        snprintf(rx1_msg, sizeof(rx1_msg), "Received ID:0x%03lX, Data:[0x%02X,0x%02X,...]",
                 rx_header.StdId, fifo1_buffer[0], fifo1_buffer[1]);
        LOGDEBUG("CAN2", rx1_msg);
    }
}

// CAN错误回调
extern "C" void HAL_CAN_ErrorCallback(CAN_HandleTypeDef* hcan)
{
    static uint32_t last_error_tick = 0;
    if (osKernelGetTickCount() - last_error_tick > 1000)  // 1秒内只打印一次错误
    {
        // 修复：格式符%X→%lX（适配uint32_t）
        char err_can_msg[64];
        snprintf(err_can_msg, sizeof(err_can_msg), "CAN%s Error: 0x%08lX",
                 (hcan->Instance == CAN1) ? "1" : "2", hcan->ErrorCode);
        LOGERROR("CAN", err_can_msg);
        last_error_tick = osKernelGetTickCount();
    }
    // 修复：删除不存在的HAL_CAN_ClearError函数调用
    // HAL_CAN_ClearError(hcan);
}