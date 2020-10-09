#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <stdio.h>
#include <cmath>
#include "Sensors/Sensors.h"
#include "EsconDriver/EsconDriver.h"

#define HZ 40000
#define PERIOD (1 / HZ)

typedef struct
{
    float p_des, v_des, kp, kd, t_ff;
} ControllerStruct;

class Controller
{
public:
    Sensors *sensors;
    EsconDriver *driver;
    ControllerStruct parameters;

    // class constructor
    Controller(Sensors *sensors, EsconDriver *driver);
    // set the controller parameters

    void reset_param();

    void control();

    void update();

    // class destructor
    ~Controller();

private:
};

#endif