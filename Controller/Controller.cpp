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
    parameters.kp = 0.0;
    parameters.kd = 0.0;
    parameters.t_ff = 0.0;
}

void Controller::control()
{
    float p_error = parameters.p_des - sensors->getMotorPosition();
    float v_error = parameters.v_des - sensors->getMotorSpeed(0.00005f);

    float current_ref = parameters.kp * p_error + parameters.kd * v_error + parameters.t_ff;

    //printf("p_des: %.4f \n", parameters.p_des);
    //printf("kp: %.5f kd: %.5f e: %.4f cur_p: %.4f cur: %.4f \n", parameters.kp, parameters.kd, error, sensors->getMotorSpeed(0.025), current_ref);
    if (fabs(current_ref) > 0.5)
        current_ref = 0.0;
    driver->setCurrent(current_ref);

    //printf("kp: %f p_des %f m_pos %f kd %f v_des %f m_v %f t_ff %f \n", parameters.kp, parameters.p_des, sensors->getMotorPosition(), parameters.kd, parameters.v_des, sensors->getMotorSpeed(1.0 / HZ), parameters.t_ff);
}

void Controller::update()
{
    //sensors->readIMU();
    control();
}

// class destructor
Controller::~Controller()
{
    printf("Controller object was destructed.\n");
}
