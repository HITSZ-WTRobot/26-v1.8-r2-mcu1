/**
 * @file    chassis.hpp
 * @author  syhanjin
 * @date    2026-02-02
 */
#pragma once
#include "Mecanum4.hpp"

#define CHASSIS_DEFAULT_TRANSLATION_LIMIT { .max_spd = 1.0f, .max_acc = 1.2f, .max_jerk = 2.0f }
#define CHASSIS_DEFAULT_ROTATION_LIMIT    { .max_spd = 180, .max_acc = 45, .max_jerk = 90 }

/**
 * 底盘对象
 */
extern chassis::Mecanum4* chassis_;

static void APP_Chassis_Update_200Hz()
{
    chassis_->profileUpdate(0.005);
}

static void APP_Chassis_Update_1kHz()
{
    static uint32_t prescaler_500Hz = 0;

    chassis_->feedbackUpdate();
    prescaler_500Hz++;
    if (prescaler_500Hz >= 2)
    {
        chassis_->errorUpdate();
        prescaler_500Hz = 0;
    }
    chassis_->controllerUpdate();
}

void APP_Chassis_BeforeUpdate();
void APP_Chassis_Init();