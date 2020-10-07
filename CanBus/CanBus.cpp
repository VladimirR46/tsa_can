#include "CanBus.h"

static constexpr uint8_t NUM_NODE_ID_BITS = 6;
static constexpr uint8_t NUM_CMD_ID_BITS = 11 - NUM_NODE_ID_BITS;

// class constructor
CanBus::CanBus(PinName rd, PinName td, int hz, Sensors *sensors, EsconDriver *driver) : can(rd, td, hz),
                                                                                        sensors(sensors),
                                                                                        driver(driver)
{
    //NVIC_SetPriority(CAN1_RX0_IRQn, 3);
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
    uint16_t lin_c = sensors->getLinearCounter();
    uint16_t m_e = 3234;
    uint8_t m_count = 250;
    uint16_t f_sensor = sensors->getForceU16();
    uint16_t m_current = driver->getCurrentU16();

    txMsg.data[0] = lin_c & 0xFF;
    txMsg.data[1] = ((lin_c & 0xF00) >> 8) + ((m_e & 0x0F) << 4);
    txMsg.data[2] = (m_e & 0xFF0) >> 4;
    txMsg.data[3] = m_count;
    txMsg.data[4] = f_sensor & 0xFF;
    txMsg.data[5] = (f_sensor & 0xFF00) >> 8;
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