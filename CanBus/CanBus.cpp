#include "CanBus.h"

// class constructor
CanBus::CanBus(PinName rd, PinName td, int hz, Controller *controller) : can(rd, td, hz),
                                                                         controller(controller)
{

    NVIC_SetPriority(CAN1_RX0_IRQn, 1);
    can.filter(CAN_ID, 0xFFE00000, CANStandard, 0); //0xFFE00004
    txMsg.id = CAN_MASTER_ID;
    txMsg.len = txSize;
    rxMsg.len = rxSize;
    can.attach(callback(this, &CanBus::onMsgReceived), CAN::RxIrq);

    printf("CanBus object was constructed.\n");
}

void CanBus::onMsgReceived()
{
    if (!can.read(rxMsg))
        return;

    if (rxMsg.id == CAN_ID)
    {
        switch (rxMsg.data[0])
        {
        case MSG_MOTOR_ON:
            set_motor_on(rxMsg);
            break;
        case MSG_MOTOR_OFF:
            set_motor_off(rxMsg);
            break;
        case MSG_SET_CURRENT_1:
            set_current_1(rxMsg);
            break;
        case MSG_WRITE_PID_RAM:
            write_pid_ram(rxMsg);
            break;
        case MSG_READ_PID:
            read_pid_callback(rxMsg);
            break;
        case MSG_SET_POSITION:
            set_position_callback(rxMsg);
            break;
        case MSG_IMU_ACCEL:
            read_imu_accellerometer(rxMsg);
            break;
        default:
            unknown_command(rxMsg);
            break;
        }
    }
}

void CanBus::unknown_command(CANMessage &msg)
{
    txMsg.id = msg.id;
    memcpy(txMsg.data, msg.data, msg.len);
    can.write(txMsg);
}

void CanBus::read_imu_accellerometer(CANMessage &msg)
{
    /*
    txMsg.id = CAN_ID << NUM_CMD_ID_BITS;
    txMsg.id += MSG_IMU_ACCEL;

    float Accel[3]; // x/y/z accel register data stored here
    controller->sensors->getIMUAccel(&Accel[0]);

    uint32_t floatBytes;
    std::memcpy(&floatBytes, &Accel[2], sizeof floatBytes);
    txMsg.data[0] = floatBytes;
    txMsg.data[1] = floatBytes >> 8;
    txMsg.data[2] = floatBytes >> 16;
    txMsg.data[3] = floatBytes >> 24;

    can.write(txMsg);
    */
}
void CanBus::set_position_callback(CANMessage &msg)
{
    /*
    txMsg.id = CAN_ID << NUM_CMD_ID_BITS;
    txMsg.id += MSG_SET_POSITION;

    uint32_t currentBytes = (msg.data[3] << 24) | (msg.data[2] << 16) | (msg.data[1] << 8) | msg.data[0];
    static_assert(sizeof(currentBytes) == sizeof(controller->parameters.kp));
    std::memcpy(&controller->parameters.p_des, &currentBytes, sizeof currentBytes);

    // reply
    uint16_t m_e = controller->sensors->getMotorCountOneTurn(); // 0    ... +1024
    int16_t m_count = controller->sensors->getMotorTurnCount(); // -512 ... +512
    uint16_t lin_e = controller->sensors->getLinearCounter();   // 0    ... +4096
    uint16_t f_sensor = controller->sensors->getForceU16();     // 0    ... 65535
    uint16_t m_current = controller->driver->getCurrentU16();   // 0    ... 65535

    txMsg.data[0] = m_e & 0xFF;
    txMsg.data[1] = ((m_e & 0x300) >> 8) + ((m_count & 0x3F) << 2);
    txMsg.data[2] = ((m_count & 0x3C0) >> 6) + ((lin_e & 0x0F) << 4);
    txMsg.data[3] = (lin_e & 0xFF0) >> 4;
    txMsg.data[4] = *(uint8_t *)(&f_sensor); // f_sensor & 0xFF;
    txMsg.data[5] = *((uint8_t *)(&f_sensor) + 1);
    txMsg.data[6] = m_current & 0xFF;
    txMsg.data[7] = (m_current & 0xFF00) >> 8;

    can.write(txMsg);
    */
}

void CanBus::read_pid_callback(CANMessage &msg)
{
    /* 
    txMsg.id = CAN_ID << NUM_CMD_ID_BITS;
    txMsg.id += MSG_READ_PID;

    uint32_t floatBytes;
    std::memcpy(&floatBytes, &controller->parameters.kp, sizeof floatBytes);
    txMsg.data[0] = floatBytes;
    txMsg.data[1] = floatBytes >> 8;
    txMsg.data[2] = floatBytes >> 16;
    txMsg.data[3] = floatBytes >> 24;

    floatBytes;
    std::memcpy(&floatBytes, &controller->parameters.kd, sizeof floatBytes);
    txMsg.data[4] = floatBytes;
    txMsg.data[5] = floatBytes >> 8;
    txMsg.data[6] = floatBytes >> 16;
    txMsg.data[7] = floatBytes >> 24;

    can.write(txMsg);
    */
}

void CanBus::write_pid_ram(CANMessage &msg)
{
    /*
    txMsg.id = CAN_ID << NUM_CMD_ID_BITS;
    txMsg.id += MSG_WRITE_PID_RAM;

    uint32_t currentBytes = (msg.data[3] << 24) | (msg.data[2] << 16) | (msg.data[1] << 8) | msg.data[0];
    static_assert(sizeof(currentBytes) == sizeof(controller->parameters.kp));
    std::memcpy(&controller->parameters.kp, &currentBytes, sizeof currentBytes);

    currentBytes = (msg.data[7] << 24) | (msg.data[6] << 16) | (msg.data[5] << 8) | msg.data[4];
    static_assert(sizeof(currentBytes) == sizeof(controller->parameters.kd));
    std::memcpy(&controller->parameters.kd, &currentBytes, sizeof currentBytes);

    can.write(txMsg);
    */
}

void CanBus::set_motor_on(CANMessage &msg)
{
    txMsg.id = msg.id;
    memcpy(txMsg.data, msg.data, msg.len);

    controller->driver->motorEnable();
    can.write(txMsg);
}

void CanBus::set_motor_off(CANMessage &msg)
{
    txMsg.id = msg.id;
    memcpy(txMsg.data, msg.data, msg.len);

    controller->driver->motorDisable();
    can.write(txMsg);
}

void CanBus::set_current_1(CANMessage &msg)
{
    txMsg.id = msg.id;
    txMsg.data[0] = msg.data[0];

    int16_t current = (msg.data[5] << 8) | msg.data[4];

    // reply
    uint16_t encoder = controller->sensors->getMotorCountOneTurn(); // 0    ... +1024
    uint16_t iq = controller->driver->getCurrentU16();              // 0    ... 65535

    txMsg.data[1] = 0x00;
    txMsg.data[2] = *(uint8_t *)(&iq);
    txMsg.data[3] = *((uint8_t *)(&iq) + 1);
    txMsg.data[4] = 0x00;
    txMsg.data[5] = 0x00;
    txMsg.data[6] = *(uint8_t *)(&encoder);
    txMsg.data[7] = *((uint8_t *)(&encoder) + 1);

    can.write(txMsg);
}

// class destructor
CanBus::~CanBus()
{
    printf("CanBus object was destructed.\n");
}