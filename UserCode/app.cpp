/**
 * @file    app.cpp
 * @author  syhanjin
 * @date    2026-02-01
 */
#include "app.hpp"

#include "chassis.hpp"
#include "cmsis_os2.h"
#include "device.hpp"
#include "dji.hpp"
#include "system.hpp"
#include "tim.h"
#include "Mecanum4.hpp"

size_t prescaler = 0;

extern "C" void TIM_Callback_1kHz(TIM_HandleTypeDef* htim)
{
    APP_Chassis_Update_1kHz();

    APP_Device_Update_1kHz();

    service::Watchdog::EatAll();
}

extern "C" void TIM_Callback_200Hz(TIM_HandleTypeDef* htim)
{
    APP_Chassis_Update_200Hz();
}

chassis::IChassis::Posture target;

bool                             relocalize_flag = false;
sensors::ops::ActionOPS::Posture relocalize_target;

/**
 * @brief Function implementing the initTask thread.
 * @param argument: Not used
 * @retval None
 */
extern "C" void Init(void* argument)
{
    /* 初始化代码 */
    APP_Device_Init();

    APP_Chassis_BeforeUpdate();

    // 启动定时器
    HAL_TIM_RegisterCallback(&htim6, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM_Callback_1kHz);
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_RegisterCallback(&htim13, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM_Callback_200Hz);
    HAL_TIM_Base_Start_IT(&htim13);

    // 等待各设备连接
    APP_Device_WaitConnections();

    /**
     * 等待各种东西更新
     */
    osDelay(2000);

    APP_Chassis_Init();

    osDelay(1000);

    osEventFlagsSet(systemEventHandle, SYSTEM_INITIALIZED);

    chassis::IChassis::Posture target_ = target;

    for (;;)
    {
        if (relocalize_flag)
        {
            relocalize_flag = false;
            sensor_ops->resetWorldCoordByPose(relocalize_target);
            chassis_->stop();
        }

        if (target_.x != target.x || target_.y != target.y || target_.yaw != target.yaw)
        {
            target_ = target;
            chassis_->setTargetPostureInWorld(target);
            chassis_->waitTrajectoryFinish();
        }

        osDelay(1);
    }

    /* 初始化完成后退出线程 */
    osThreadExit();
}