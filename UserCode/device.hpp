/**
 * @file    device.hpp
 * @author  syhanjin
 * @date    2026-02-02
 */
#pragma once

#include "ActionOPS.hpp"
#include "dji.hpp"
#include "HWT101CT.hpp"
#include "STP23L.hpp"

/**
 * 底盘使用的轮子
 * 大疆电机 3508, CAN1 ID: 1 ~ 4
 */
extern motors::DJIMotor* motor_wheel[4];

/**
 * 升降结构使用的电机
 * 大疆电机 3508, CAN1 ID: 5 ~ 6
 */
extern motors::DJIMotor* motor_elev[2];

/**
 * 前后滑行结构使用的电机
 * 大疆电机 3508, CAN2 ID: 1 ~ 2
 * TODO: 改到MCU2上
 */
extern motors::DJIMotor* motor_slide[2];

/**
 * 折叠结构使用的电机
 * 大疆电机 2006, CAN2 ID: 3 ~ 4
 */
extern motors::DJIMotor* motor_fold[2];

/**
 * 陀螺仪
 * HWT101CT， UART2
 */
extern sensors::gyro::HWT101CT* sensor_gyro_yaw;
#define DEVICE_SENSOR_GYRO_YAW_UART (&huart2)

/**
 * 码盘
 * Action-OPS, UART6
 */
extern sensors::ops::ActionOPS* sensor_ops;
#define DEVICE_SENSOR_OPS_UART (&huart6)

/**
 * 激光测距传感器
 * STP23L, UART3
 */
extern sensors::laser::STP23L* sensor_stp23l;
#define DEVICE_SENSOR_STP23L_UART (&huart3)

void APP_Device_Init();
void APP_Device_Update_1kHz();
bool APP_Device_isAllConnected();
void APP_Device_WaitConnections();