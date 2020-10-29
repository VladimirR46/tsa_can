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
        MSG_MOTOR_ON,
        MSG_MOTOR_OFF,
        MSG_SET_CURRENT,
        MSG_WRITE_PID_RAM,
        MSG_READ_PID,
        MSG_SET_POSITION
    };

    // class constructor
    CanBus(PinName rd, PinName td, int hz, Controller *controller);

    void onMsgReceived();

    // cmd function
    void set_motor_on(CANMessage &msg);
    void set_motor_off(CANMessage &msg);
    void set_current_callback(CANMessage &msg);
    void write_pid_ram(CANMessage &msg);
    void read_pid_callback(CANMessage &msg);
    void set_position_callback(CANMessage &msg);

    // Utility functions
    static uint32_t get_node_id(uint32_t msgID);
    static uint32_t get_cmd_id(uint32_t msgID);

    // class destructor
    ~CanBus();

private:
    CAN can;
    CANMessage rxMsg;
    CANMessage txMsg;

    Controller *controller;
};

#endif