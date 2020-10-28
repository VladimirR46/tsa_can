#include "mbed.h"
#include "EsconDriver/EsconDriver.h"
#include "CanBus/CanBus.h"
#include "Controller/Controller.h"
#include "TimEncoders/Nucleo_Encoder_16_bits.h"

#define M_PI 3.14159265358979323846

BufferedSerial pc(USBTX, USBRX, 115200); // tx, rx

FileHandle *mbed::mbed_override_console(int fd)
{
  return &pc;
}

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

Controller controller(&sensors, &driver, PG_0);

CanBus can(PB_8, PB_9, 1000000, &controller);

Ticker ticker;

int main()
{

  ticker.attach(callback(&controller, &Controller::update), 50us); // 25us

  while (1)
  {
  }
}