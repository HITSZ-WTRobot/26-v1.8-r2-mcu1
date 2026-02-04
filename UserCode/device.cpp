/**
 * @file    device.cpp
 * @author  syhanjin
 * @date    2026-02-02
 */

#include "device.hpp"
#include "ActionOPS.hpp"
#include "HWT101CT.hpp"
#include "STP23L.hpp"
#include "can.h"
#include "dji.hpp"
#include "usart.h"

motors::DJIMotor*        motor_wheel[4]; // 底盘轮子电机
motors::DJIMotor*        motor_elev[2];  // 升降电机
motors::DJIMotor*        motor_slide[2]; // 前后滑行电机
motors::DJIMotor*        motor_fold[2];  // 折叠电机
sensors::gyro::HWT101CT* sensor_gyro_yaw;
sensors::ops::ActionOPS* sensor_ops;
sensors::laser::STP23L*  sensor_stp23l;           // 激光测距（stp23l）
sensors::laser::DT35*    sensor_laser_dt35_left;  // 左侧激光测距（DT35）
sensors::laser::DT35*    sensor_laser_dt35_right; // 右侧激光测距（DT35)

sensors::laser::DT35Board* sensor_laser_dt35_board; // DT35 驱动板

UartRxSync_DefineCallback(sensor_gyro_yaw);
UartRxSync_DefineCallback(sensor_ops);
UartRxSync_DefineCallback(sensor_stp23l);
UartRxSync_DefineCallback(sensor_laser_dt35_board);

static void sensor_init()
{
    using namespace sensors;

    UartRxSync_RegisterCallback(sensor_gyro_yaw, DEVICE_SENSOR_GYRO_YAW_UART);
    sensor_gyro_yaw = new gyro::HWT101CT(DEVICE_SENSOR_GYRO_YAW_UART);

    UartRxSync_RegisterCallback(sensor_ops, DEVICE_SENSOR_OPS_UART);
    sensor_ops = new ops::ActionOPS(DEVICE_SENSOR_OPS_UART,
                                    { .x_offset   = -276.0f,
                                      .y_offset   = 0.0f,
                                      .yaw_offset = -90.0f,
                                      .yaw_car    = &sensor_gyro_yaw->getYaw() });

    UartRxSync_RegisterCallback(sensor_stp23l, DEVICE_SENSOR_STP23L_UART);
    sensor_stp23l = new laser::STP23L(DEVICE_SENSOR_STP23L_UART);

    sensor_laser_dt35_left = new laser::DT35({ .near = { .raw_data = 0, .distance = 0.0f },
                                               .far  = { .raw_data = 6452000, .distance = 7.4f },
                                               .k    = 1.0f });

    sensor_laser_dt35_right = new laser::DT35({ .near = { .raw_data = 14848, .distance = 0.073f },
                                                .far  = { .raw_data = 6474752, .distance = 0.423f },
                                                .k    = 0.0985765f });

    UartRxSync_RegisterCallback(sensor_laser_dt35_board, DEVICE_SENSOR_DT35_BOARD_UART);
    sensor_laser_dt35_board = new laser::DT35Board(DEVICE_SENSOR_DT35_BOARD_UART);
    sensor_laser_dt35_board->registerChannel(0, sensor_laser_dt35_right);
    sensor_laser_dt35_board->registerChannel(1, sensor_laser_dt35_left);

    // device enable
    if (!sensor_gyro_yaw->startReceive())
        Error_Handler();
    if (!sensor_ops->startReceive())
        Error_Handler();
    if (!sensor_stp23l->startReceive())
        Error_Handler();
    if (!sensor_laser_dt35_board->startReceive())
        Error_Handler();
}

static void can_init()
{
    // CAN 初始化
    motors::DJIMotor::CAN_FilterInit(&hcan1, 0);
    CAN_RegisterCallback(&hcan1, motors::DJIMotor::CANBaseReceiveCallback);
    motors::DJIMotor::CAN_FilterInit(&hcan2, 14);
    CAN_RegisterCallback(&hcan2, motors::DJIMotor::CANBaseReceiveCallback);

    // 注册 CAN 主回调，并启动 CAN
    HAL_CAN_RegisterCallback(&hcan1, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, CAN_Fifo0ReceiveCallback);
    CAN_Start(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_RegisterCallback(&hcan2, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, CAN_Fifo0ReceiveCallback);
    CAN_Start(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

constexpr motors::DJIMotor::Config motor_wheel_config[4] = {
    {
            .hcan    = &hcan1,
            .type    = motors::DJIMotor::Type::M3508_C620,
            .id1     = 1,
            .reverse = false,
    },
    {
            .hcan    = &hcan1,
            .type    = motors::DJIMotor::Type::M3508_C620,
            .id1     = 2,
            .reverse = true,
    },
    {
            .hcan    = &hcan1,
            .type    = motors::DJIMotor::Type::M3508_C620,
            .id1     = 3,
            .reverse = true,
    },
    {
            .hcan    = &hcan1,
            .type    = motors::DJIMotor::Type::M3508_C620,
            .id1     = 4,
            .reverse = false,
    },
};

static void wheel_motor_init()
{
    using namespace motors;
    for (size_t i = 0; i < 4; ++i)
        motor_wheel[i] = new DJIMotor(motor_wheel_config[i]);
}

constexpr motors::DJIMotor::Config motor_elevator_config[2] = {
    {
            .hcan    = &hcan1,
            .type    = motors::DJIMotor::Type::M3508_C620,
            .id1     = 5,
            .reverse = false, //

            // TODO: add external reduction ratio
    },
    {
            .hcan    = &hcan1,
            .type    = motors::DJIMotor::Type::M3508_C620,
            .id1     = 6,
            .reverse = true, // positive -> down, left side
    },
};

constexpr motors::DJIMotor::Config motor_slide_config[2] = {
    {
            .hcan    = &hcan2,
            .type    = motors::DJIMotor::Type::M3508_C620,
            .id1     = 1,
            .reverse = true,
    },
    {
            .hcan    = &hcan2,
            .type    = motors::DJIMotor::Type::M3508_C620,
            .id1     = 2,
            .reverse = false,
    },
};

constexpr motors::DJIMotor::Config motor_fold_config[2] = {
    {
            .hcan    = &hcan2,
            .type    = motors::DJIMotor::Type::M2006_C610,
            .id1     = 3,
            .reverse = false,
    },
    {
            .hcan    = &hcan2,
            .type    = motors::DJIMotor::Type::M2006_C610,
            .id1     = 4,
            .reverse = true,
    },
};

static void elevator_motor_init()
{
    using namespace motors;
    for (size_t i = 0; i < 2; ++i)
        motor_elev[i] = new DJIMotor(motor_elevator_config[i]);

    for (size_t i = 0; i < 2; ++i)
        motor_slide[i] = new DJIMotor(motor_slide_config[i]);

    for (size_t i = 0; i < 2; ++i)
        motor_fold[i] = new DJIMotor(motor_fold_config[i]);
}

void APP_Device_Init()
{
    sensor_init();

    can_init();

    wheel_motor_init();
    elevator_motor_init();
}

void APP_Device_Update_1kHz()
{
    motors::DJIMotor::SendIqCommand(&hcan1, motors::DJIMotor::IqSetCMDGroup::IqCMDGroup_1_4);
    motors::DJIMotor::SendIqCommand(&hcan1, motors::DJIMotor::IqSetCMDGroup::IqCMDGroup_5_8);
}

bool APP_Device_isAllConnected()
{
    bool all_connected = true;
    // check sensors
    all_connected &= sensor_gyro_yaw->isConnected();

    all_connected &= sensor_ops->isConnected();

    all_connected &= sensor_stp23l->isConnected();

    all_connected &= sensor_laser_dt35_board->isConnected();

    // check motors
    for (const auto& m : motor_wheel)
        all_connected &= m->isConnected();

    for (const auto& m : motor_elev)
        all_connected &= m->isConnected();

    for (const auto& m : motor_slide)
        all_connected &= m->isConnected();

    for (const auto& m : motor_fold)
        all_connected &= m->isConnected();

    return all_connected;
}

void APP_Device_WaitConnections()
{
    while (!APP_Device_isAllConnected())
        osDelay(1);
}