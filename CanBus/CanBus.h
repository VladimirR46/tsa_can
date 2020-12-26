#ifndef CANBUS_H
#define CANBUS_H

/* Network settings */
#define txSize 8
#define rxSize 8
#define CAN_ID 1
#define CAN_MASTER_ID 0

#include <stdio.h>
#include "mbed.h"
#include "Controller/Controller.h"

class CanBus
{
public:
    enum
    {
        MSG_MOTOR_ON = 0x88,
        MSG_MOTOR_OFF = 0x80,
        MSG_SET_CURRENT_1 = 0xA1,
        MSG_WRITE_PID_RAM,
        MSG_READ_PID,
        MSG_SET_POSITION,
        MSG_IMU_ACCEL
    };

    // class constructor
    CanBus(PinName rd, PinName td, int hz, Controller *controller);

    void onMsgReceived();

    // cmd function
    void set_motor_on(CANMessage &msg);
    void set_motor_off(CANMessage &msg);
    void set_current_1(CANMessage &msg);
    void write_pid_ram(CANMessage &msg);
    void read_pid_callback(CANMessage &msg);
    void set_position_callback(CANMessage &msg);
    void read_imu_accellerometer(CANMessage &msg);

    void unknown_command(CANMessage &msg);

    // class destructor
    ~CanBus();

private:
    CAN can;
    CANMessage rxMsg;
    CANMessage txMsg;

    Controller *controller;
};

#endif