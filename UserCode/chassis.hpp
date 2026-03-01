/**
 * @file    chassis.hpp
 * @author  syhanjin
 * @date    2026-02-02
 */
#pragma once
#include "Mecanum4.hpp"
#include "Slave.hpp"

#define CHASSIS_DEFAULT_TRANSLATION_LIMIT { .max_spd = 1.0f, .max_acc = 1.2f, .max_jerk = 2.0f }
#define CHASSIS_DEFAULT_ROTATION_LIMIT    { .max_spd = 180, .max_acc = 45, .max_jerk = 90 }

/**
 * 底盘对象
 */
using Chassis = chassis::controller::Slave<chassis::Mecanum4, 500>;
extern Chassis* chassis_;

void APP_Chassis_Update_100Hz();
void APP_Chassis_Update_1kHz();
void APP_Chassis_BeforeUpdate();
void APP_Chassis_Init();