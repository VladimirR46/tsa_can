#ifndef ESCONDRIVER_H
#define ESCONDRIVER_H

#include "mbed.h"
#include <stdio.h>

class EsconDriver
{
public:
    // class constructor
    EsconDriver(PinName *driverDigitalPins, PinName *driverAnalogPins);
    // set the motor parameters
    void setParameters(float *mot_params);
    // enable the motor
    void motorEnable();
    // disable the motor
    void motorDisable();
    // get actual motor current
    float getCurrent();
    // get filtered motor current
    float getFilteredCurrent();
    // set the direction of the motor rotation
    void setDirection(float current);
    // set the motor current
    void setCurrent(float current);
    // set the motor torque
    void setTorque(float torque);
    // class destructor
    ~EsconDriver();

    // TODO driver.getCurrent_i16(int current_count) | Do we actually need it?
    // TODO driver.getTorque(float torque_value) | What exactly should this function return?

private:
    // digital driver pins
    DigitalOut mot_en;      // motor enable output: 0 - disable, 1 - enable
    DigitalOut mot_en_led;  // motor enable LED
    DigitalOut mot_dir;     // motor direction output: 0 - CW, 1 - CCW  (TODO check the correctness)
    DigitalOut mot_dir_led; // motor direction LED

    // analog driver pins
    AnalogIn mot_cur_in;     // motor current input
    AnalogIn mot_cur_flt_in; // motor filtered current input
    AnalogOut mot_set_out;   // motor setpoint output

    // motor parameters
    float set_scale = 0.0; // Current scale we use to set the desired current using setCurrent() function
    float get_scale = 0.0; // Current scale we use to read the actual current value using getCurrent() function
    float k_m = 0.0;       // Torque constant (mNm/A)
};

#endif