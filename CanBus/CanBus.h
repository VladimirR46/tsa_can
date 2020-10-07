#ifndef CANBUS_H
#define CANBUS_H

/* Network settings */
#define txSize 8
#define rxSize 8
#define CAN_ID 1
#define CAN_MASTER_ID 0

#include <stdio.h>
#include "mbed.h"
#include "Sensors/Sensors.h"
#include "EsconDriver/EsconDriver.h"

class CanBus
{
public:
    enum
    {
        MSG_MOTOR_ON,
        MSG_MOTOR_OFF,
        MSG_SET_CURRENT
    };

    // class constructor
    CanBus(PinName rd, PinName td, int hz, Sensors *sensors, EsconDriver *driver);

    void onMsgReceived();

    // cmd function
    void set_motor_on(CANMessage &msg);
    void set_motor_off(CANMessage &msg);
    void set_current_callback(CANMessage &msg);

    // Utility functions
    static uint32_t get_node_id(uint32_t msgID);
    static uint8_t get_cmd_id(uint32_t msgID);

    // class destructor
    ~CanBus();

private:
    CAN can;
    CANMessage rxMsg;
    CANMessage txMsg;

    Sensors *sensors;
    EsconDriver *driver;
};

#endif