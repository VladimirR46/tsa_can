#include "EsconDriver.h"

// class constructor
EsconDriver::EsconDriver(PinName *driverDigitalPins, PinName *driverAnalogPins) : mot_en(driverDigitalPins[0]),
                                                                                  mot_en_led(driverDigitalPins[1]),
                                                                                  mot_dir(driverDigitalPins[2]),
                                                                                  mot_dir_led(driverDigitalPins[3]),

                                                                                  mot_cur_in(driverAnalogPins[0]),
                                                                                  mot_cur_flt_in(driverAnalogPins[1]),
                                                                                  mot_set_out(driverAnalogPins[2])
{
    EsconDriver::motorDisable(); // TODO | Do we actually need this line?

    printf("EsconDriver object was constructed.\n");
}

// set the motor parameters
void EsconDriver::setParameters(float *mot_params)
{
    set_scale = mot_params[0];
    get_scale = mot_params[1];
    k_m = mot_params[2];
}

void EsconDriver::motorEnable()
{
    mot_en = 1;
    mot_en_led = mot_en; // turn on led if motor is enabled
}

void EsconDriver::motorDisable()
{
    mot_en = 0;
    mot_en_led = mot_en; // turn off led if motor is disabled
}

// get actual motor current
float EsconDriver::getCurrent()
{
    return get_scale * (mot_cur_in - 0.5f); // mot_cur_in provide the float value in range [0, 1]
}

// get filtered motor current
float EsconDriver::getFilteredCurrent()
{
    return get_scale * (mot_cur_flt_in - 0.5f); // mot_cur_flt_in provide the float value in range [0, 1]
}

// set the direction of the motor rotation
void EsconDriver::setDirection(float current)
{
    if (current > 0.)
    {
        mot_dir = 1;
        mot_dir_led = mot_dir; // turn on led if motor rotates CCW
    }
    else
    {
        mot_dir = 0;
        mot_dir_led = mot_dir; // turn off led if motor rotates CW
    }
}

// set the motor current
void EsconDriver::setCurrent(float current)
{
    float current_abs = abs(current);
    setDirection(current);
    mot_set_out = /*set_scale * */ current_abs;
}

// set the motor torque
void EsconDriver::setTorque(float torque)
{
    float current = torque / k_m;
    setCurrent(current);
}

// class destructor
EsconDriver::~EsconDriver()
{
    printf("EsconDriver object was destructed.\n");
}