#include "CanBus.h"

static constexpr uint8_t NUM_NODE_ID_BITS = 6;
static constexpr uint8_t NUM_CMD_ID_BITS = 11 - NUM_NODE_ID_BITS;

// class constructor
CanBus::CanBus(PinName rd, PinName td, int hz, Sensors *sensors, EsconDriver *driver) : can(rd, td, hz),
                                                                                        sensors(sensors),
                                                                                        driver(driver)
{
    NVIC_SetPriority(CAN1_RX0_IRQn, 3);
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
        default:
            break;
        }
    }
}

void CanBus::set_motor_on(CANMessage &msg)
{
    txMsg.id = CAN_ID << NUM_CMD_ID_BITS;
    txMsg.id += MSG_MOTOR_ON;

    driver->motorEnable();
    can.write(txMsg);
}

void CanBus::set_motor_off(CANMessage &msg)
{
    txMsg.id = CAN_ID << NUM_CMD_ID_BITS;
    txMsg.id += MSG_MOTOR_OFF;

    driver->motorDisable();
    can.write(txMsg);
}

void CanBus::set_current_callback(CANMessage &msg)
{
    txMsg.id = CAN_ID << NUM_CMD_ID_BITS;
    txMsg.id += MSG_SET_CURRENT;

    uint32_t currentBytes = (msg.data[3] << 24) | (msg.data[2] << 16) | (msg.data[1] << 8) | msg.data[0];
    float current = 0.0f;
    static_assert(sizeof(currentBytes) == sizeof(current));
    std::memcpy(&current, &currentBytes, sizeof currentBytes);
    driver->setCurrent(current);

    // reply
    float m_pos = 22.2;  //sensors->getMotorPosition();
    float l_pos = 33.44; //sensors->getLinearPosition();

    uint32_t floatBytes;
    static_assert(sizeof(m_pos) == sizeof(current));
    std::memcpy(&floatBytes, &m_pos, sizeof floatBytes);
    txMsg.data[0] = floatBytes;
    txMsg.data[1] = floatBytes >> 8;
    txMsg.data[2] = floatBytes >> 16;
    txMsg.data[3] = floatBytes >> 24;

    static_assert(sizeof(l_pos) == sizeof(current));
    std::memcpy(&floatBytes, &l_pos, sizeof floatBytes);
    txMsg.data[4] = floatBytes;
    txMsg.data[5] = floatBytes >> 8;
    txMsg.data[6] = floatBytes >> 16;
    txMsg.data[7] = floatBytes >> 24;

    //ThisThread::sleep_for(1ms);
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