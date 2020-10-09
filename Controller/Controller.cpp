#include "Controller.h"

// class constructor
Controller::Controller(Sensors *sensors, EsconDriver *driver) : sensors(sensors),
                                                                driver(driver)
{
    printf("Controller object was constructed.\n");
    reset_param();
}

void Controller::reset_param()
{
    parameters.p_des = 0;
    parameters.v_des = 0;
    parameters.kp = 0;
    parameters.kd = 0;
    parameters.t_ff = 0;
}

void Controller::control()
{
    float current_ref = parameters.kp * (parameters.p_des - sensors->getMotorPosition()) + parameters.kd * (parameters.v_des - sensors->getMotorSpeed(PERIOD)) + parameters.t_ff;
    driver->setCurrent(current_ref);
}

void Controller::update()
{
    control();
}

// class destructor
Controller::~Controller()
{
    printf("Controller object was destructed.\n");
}
