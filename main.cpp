#include "mbed.h"
//using namespace std::chrono;

#include "EsconDriver/EsconDriver.h"
#include "CanBus/CanBus.h"
#include "Controller/Controller.h"
#include "TimEncoders/Nucleo_Encoder_16_bits.h"

// DRIVER //
PinName driverDigitalPins[] = {
    PE_3, // motor enable output: 0 - disable, 1 - enable
    LED1, // motor enable LED
    PE_2, // motor direction output: 0 - CW, 1 - CCW
    LED2  // motor direction LED
};

PinName driverAnalogPins[] = {
    PC_3, // motor current input
    PC_2, // motor filtered current input
    PA_4  // motor setpoint output
};

/* Initialize the EsconDriver */
EsconDriver driver(driverDigitalPins, driverAnalogPins);

/* Initialize the motor and linear encoders */
Nucleo_Encoder_16_bits motor_encoder(TIM4);
Nucleo_Encoder_16_bits linear_encoder(TIM3);

// ForceSensor
AnalogIn force_sensor(PF_9);
Sensors sensors(&motor_encoder, &linear_encoder, &force_sensor);

Controller controller(&sensors, &driver);

CanBus can(PB_8, PB_9, 1000000, &controller);

Ticker ticker;

// 40 kHz
void TickerCallback()
{
  controller.update();
}

int main()
{
  ticker.attach(&TickerCallback, 25us);

  while (1)
  {
  }
}