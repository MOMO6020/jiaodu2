#include "app_main.h"
#include "app/chassis/chassis.hpp"
#include "app/robotcmd/robotcmd.hpp"
#include "bsp/system/mutex.hpp"
#include "cmsis_os.h"

#include "bsp/log/log.hpp"
#include "bsp/system/time.hpp"

#include "gimbal/gimbal.hpp"
#include "modules/imu/ins_task.h"
#include "modules/motor/motor.hpp"
#include "modules/motor/DJIMotor/DJIMotor.hpp"  // 包含DJIMotor头文件
#include <memory>
#include <cstdio>  // 新增：用于snprintf格式化字符串

// 新增：电机实例指针声明
std::shared_ptr<DJIMotor> gm6020 = nullptr;
std::shared_ptr<DJIMotor> j4310 = nullptr;
std::shared_ptr<DJIMotor> j8009 = nullptr;

// 原有变量声明
std::shared_ptr<float>    gimbal_yaw_motor_angle_ptr = nullptr;
std::shared_ptr<RobotCMD> robotcmd                   = nullptr;
std::shared_ptr<Chassis>  chassis                    = nullptr;
std::shared_ptr<Gimbal>   gimbal                     = nullptr;
attitude_t*               imu_data                   = nullptr;

// 新增：电机控制任务声明
void MotorPositionControlTask(void* arg);

// 原有任务声明
void RobotCMDTask(void* arg __attribute__((unused)));
void ChassisTask(void* arg __attribute__((unused)));
void INSTask(void* arg __attribute__((unused)));
void GimbalTask(void* arg __attribute__((unused)));

void app_main()
{
    taskENTER_CRITICAL();
    RTTLog::init();
    STM32TimeDWT::DWT_Init(168);
    LOGINFO("Robot", "System Init");
    STM32CAN_Init();
    StartDaemonTask();
    StartMotorControlTask();

    // 原有初始化逻辑
    gimbal_yaw_motor_angle_ptr = std::make_shared<float>(0.0f);
    chassis                    = std::make_shared<Chassis>();
    imu_data                   = INS_Init();
    robotcmd                   = std::make_shared<RobotCMD>(imu_data, gimbal_yaw_motor_angle_ptr);
    gimbal                     = std::make_shared<Gimbal>(imu_data, gimbal_yaw_motor_angle_ptr);

    // 新增：初始化目标电机（GM6020/J4310/J8009）
    extern CAN_HandleTypeDef hcan1;  // 假设使用CAN1总线，根据实际硬件修改
    
    // GM6020初始化（ID=5，角度环+速度环控制）
    gm6020 = std::make_shared<DJIMotor>(
        &hcan1, 5, MotorType::GM6020,
        MotorPIDSetting{
            .outer_loop = CloseloopType::ANGLE_LOOP,
            .close_loop = CloseloopType::ANGLE_AND_SPEED_LOOP,
            .reverse = false,
            .external_angle_feedback = FeedbackType::INTERNAL,  // 使用电机内置编码器
            .external_speed_feedback = FeedbackType::INTERNAL
        },
        MotorPID{
            .pid_angle_ = PIDController(PIDConfig{8.0f, 0.1f, 0.2f, 500.0f, 5.0f, PIDImprovement::PID_Integral_Limit, 30000.0f}),
            .pid_speed_ = PIDController(PIDConfig{5.0f, 0.05f, 0.1f, 16384.0f, 3.0f, PIDImprovement::PID_Integral_Limit, 30000.0f})
        }
    );

    // J4310初始化（ID=6）
    j4310 = std::make_shared<DJIMotor>(
        &hcan1, 6, MotorType::J4310,
        MotorPIDSetting{
            .outer_loop = CloseloopType::ANGLE_LOOP,
            .close_loop = CloseloopType::ANGLE_AND_SPEED_LOOP,
            .reverse = false,
            .external_angle_feedback = FeedbackType::INTERNAL,
            .external_speed_feedback = FeedbackType::INTERNAL
        },
        MotorPID{
            .pid_angle_ = PIDController(PIDConfig{6.0f, 0.08f, 0.15f, 500.0f, 3.0f, PIDImprovement::PID_Integral_Limit, 10000.0f}),
            .pid_speed_ = PIDController(PIDConfig{4.0f, 0.03f, 0.08f, 16384.0f, 2.0f, PIDImprovement::PID_Integral_Limit, 10000.0f})
        }
    );

    // J8009初始化（ID=7）
    j8009 = std::make_shared<DJIMotor>(
        &hcan1, 7, MotorType::J8009,
        MotorPIDSetting{
            .outer_loop = CloseloopType::ANGLE_LOOP,
            .close_loop = CloseloopType::ANGLE_AND_SPEED_LOOP,
            .reverse = false,
            .external_angle_feedback = FeedbackType::INTERNAL,
            .external_speed_feedback = FeedbackType::INTERNAL
        },
        MotorPID{
            .pid_angle_ = PIDController(PIDConfig{7.0f, 0.09f, 0.18f, 500.0f, 4.0f, PIDImprovement::PID_Integral_Limit, 20000.0f}),
            .pid_speed_ = PIDController(PIDConfig{4.5f, 0.04f, 0.09f, 16384.0f, 2.5f, PIDImprovement::PID_Integral_Limit, 20000.0f})
        }
    );

    // 创建原有任务
    xTaskCreate(RobotCMDTask, "RobotCMDTask", 1024, nullptr, osPriorityNormal, nullptr);
    xTaskCreate(ChassisTask, "ChassisTask", 1024, nullptr, osPriorityNormal, nullptr);
    xTaskCreate(INSTask, "INSTask", 1024, nullptr, osPriorityNormal, nullptr);
    xTaskCreate(GimbalTask, "GimbalTask", 1024, nullptr, osPriorityNormal, nullptr);
    
    // 新增：创建电机位置控制任务（优先级高于普通任务）
    xTaskCreate(MotorPositionControlTask, "MotorPosTask", 1024, nullptr, osPriorityAboveNormal, nullptr);

    taskEXIT_CRITICAL();
    LOGINFO("Robot", "Robot Init");
}

// 新增：电机位置控制任务（在此处修改目标角度）
void MotorPositionControlTask(void* arg __attribute__((unused)))
{
    // 等待电机初始化完成（避免空指针访问）
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // 使能电机
    if (gm6020) gm6020->enable();
    if (j4310) j4310->enable();
    if (j8009) j8009->enable();

    // 目标角度（可直接修改这些值，烧录后电机自动转到对应角度）
    const float gm6020_target_angle = 90.0f;    // GM6020目标角度：90度（范围±180度）
    const float j4310_target_angle  = 45.0f;    // J4310目标角度：45度（范围±90度）
    const float j8009_target_angle  = 180.0f;   // J8009目标角度：180度（范围0-360度）

    while (true)
    {
        // 设置目标角度（内部会进行软件限位检查）
        if (gm6020) gm6020->setTargetAngle(gm6020_target_angle);
        if (j4310) j4310->setTargetAngle(j4310_target_angle);
        if (j8009) j8009->setTargetAngle(j8009_target_angle);

        // 打印当前角度（调试用）：用snprintf格式化，适配LOGINFO仅2个参数的要求
        char log_msg[256];
        snprintf(log_msg, sizeof(log_msg), 
                 "GM6020: %.2f°, J4310: %.2f°, J8009: %.2f°",
                 gm6020 ? gm6020->getCurrentAngle() : 0.0f,
                 j4310 ? j4310->getCurrentAngle() : 0.0f,
                 j8009 ? j8009->getCurrentAngle() : 0.0f);
        LOGINFO("Motor", log_msg);

        vTaskDelay(10 / portTICK_PERIOD_MS);  // 10ms周期更新
    }
}

// 原有任务实现（保持不变）
void RobotCMDTask(void* arg __attribute__((unused)))
{
    while (true)
    {
        robotcmd->RobotCMDTask();
        vTaskDelay(2 / portTICK_PERIOD_MS);
    }
}

void ChassisTask(void* arg __attribute__((unused)))
{
    while (true)
    {
        chassis->ChassisTask();
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
}

void INSTask(void* arg __attribute__((unused)))
{
    while (true)
    {
        INS_Task();
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void GimbalTask(void* arg __attribute__((unused)))
{
    while (true)
    {
        gimbal->GimbalTask();
        vTaskDelay(2 / portTICK_PERIOD_MS);
    }
}