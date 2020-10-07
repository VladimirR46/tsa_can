#include "mbed.h"
using namespace std::chrono;

#include "EsconDriver/EsconDriver.h"
#include "CanBus/CanBus.h"
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

CanBus can(PB_8, PB_9, 1000000, &sensors, &driver);

Ticker timer1;
Ticker timer2;

uint32_t RRRR = 0;

void attime1()
{
  RRRR++;
}

void attime2()
{
  printf("Count: %d \n", RRRR);
  RRRR = 0;
}

int main()
{

  timer1.attach(&attime1, 25us);
  timer2.attach(&attime2, 1.0);

  while (1)
  {
  }
}