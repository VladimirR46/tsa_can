#include "CanBus.h"

static constexpr uint8_t NUM_NODE_ID_BITS = 6;
static constexpr uint8_t NUM_CMD_ID_BITS = 11 - NUM_NODE_ID_BITS;

// class constructor
CanBus::CanBus(PinName rd, PinName td, int hz, Controller *controller) : can(rd, td, hz),
                                                                         controller(controller)
{

    NVIC_SetPriority(CAN1_RX0_IRQn, 1);
    can.filter(CAN_ID, 0xFFE00004, CANStandard, 0);
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

    // nodeID | CMD
    // 6 bits | 5 bits
    uint32_t nodeID = get_node_id(rxMsg.id);
    uint32_t cmd = get_cmd_id(rxMsg.id);

    if (nodeID == CAN_ID)
    {
        switch (cmd)
        {
        case MSG_MOTOR_ON:
            set_motor_on(rxMsg);
            break;
        case MSG_MOTOR_OFF:
            set_motor_off(rxMsg);
            break;
        case MSG_SET_CURRENT:
            set_current_callback(rxMsg);
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
        default:
            break;
        }
    }
}

void set_position_callback(CANMessage &msg)
{
    txMsg.id = CAN_ID << NUM_CMD_ID_BITS;
    txMsg.id += MSG_MOTOR_ON;

    uint32_t currentBytes = (msg.data[3] << 24) | (msg.data[2] << 16) | (msg.data[1] << 8) | msg.data[0];
    static_assert(sizeof(currentBytes) == sizeof(controller->parameters.kp));
    std::memcpy(&controller->parameters.p_des, &currentBytes, sizeof currentBytes);

    can.write(txMsg);
}

void CanBus::read_pid_callback(CANMessage &msg)
{
    txMsg.id = CAN_ID << NUM_CMD_ID_BITS;
    txMsg.id += MSG_MOTOR_ON;

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
}

void CanBus::write_pid_ram(CANMessage &msg)
{
    txMsg.id = CAN_ID << NUM_CMD_ID_BITS;
    txMsg.id += MSG_MOTOR_ON;

    uint32_t currentBytes = (msg.data[3] << 24) | (msg.data[2] << 16) | (msg.data[1] << 8) | msg.data[0];
    static_assert(sizeof(currentBytes) == sizeof(controller->parameters.kp));
    std::memcpy(&controller->parameters.kp, &currentBytes, sizeof currentBytes);

    currentBytes = (msg.data[7] << 24) | (msg.data[6] << 16) | (msg.data[5] << 8) | msg.data[4];
    static_assert(sizeof(currentBytes) == sizeof(controller->parameters.kd));
    std::memcpy(&controller->parameters.kd, &currentBytes, sizeof currentBytes);

    can.write(txMsg);
}

void CanBus::set_motor_on(CANMessage &msg)
{
    txMsg.id = CAN_ID << NUM_CMD_ID_BITS;
    txMsg.id += MSG_MOTOR_ON;

    controller->driver->motorEnable();
    can.write(txMsg);
}

void CanBus::set_motor_off(CANMessage &msg)
{
    txMsg.id = CAN_ID << NUM_CMD_ID_BITS;
    txMsg.id += MSG_MOTOR_OFF;

    controller->driver->motorDisable();
    can.write(txMsg);
}

void CanBus::set_current_callback(CANMessage &msg)
{
    txMsg.id = CAN_ID << NUM_CMD_ID_BITS;
    txMsg.id += MSG_SET_CURRENT;

    uint32_t currentBytes = (msg.data[3] << 24) | (msg.data[2] << 16) | (msg.data[1] << 8) | msg.data[0];
    float current = 0.0f;
    static_assert(sizeof(currentBytes) == sizeof(controller->parameters.t_ff));
    std::memcpy(&controller->parameters.t_ff, &currentBytes, sizeof currentBytes);

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
}

uint32_t CanBus::get_node_id(uint32_t msgID)
{
    return (msgID >> NUM_CMD_ID_BITS); // Upper 6 or more bits
}

uint8_t CanBus::get_cmd_id(uint32_t msgID)
{
    return (msgID & 0x01F); // Bottom 5 bits
}

// class destructor
CanBus::~CanBus()
{
    printf("CanBus object was destructed.\n");
}