#include "Sensors.h"

// class constructor
Sensors::Sensors(Nucleo_Encoder_16_bits *motor_encoder,
                 Nucleo_Encoder_16_bits *linear_encoder,
                 AnalogIn *force_sensor) : motor_encoder(motor_encoder),
                                           linear_encoder(linear_encoder),
                                           force_sensor(force_sensor)
{
    printf("Sensors object was constructed.\n");
}

void Sensors::setMotorScale(float scale)
{
    mot_enc_scale = scale;
    printf("New motor encoder scale = %f\n", mot_enc_scale);
}

float Sensors::getMotorPosition(void)
{
    mot_enc_count = motor_encoder->GetCounter(); // get motor encoder counts
    mot_pos_prev = mot_pos_curr;
    mot_pos_curr = mot_enc_count * mot_enc_scale; // convert counts to rad

    return mot_pos_curr;
}

float Sensors::getMotorSpeed(float period_s)
{
    mot_vel = (mot_pos_curr - mot_pos_prev) / period_s; // in rad/s

    return mot_vel;
}

void Sensors::setLinearScale(float scale)
{
    lin_enc_scale = scale;
    printf("New linear encoder scale = %f\n", lin_enc_scale);
}

float Sensors::getLinearPosition(void)
{
    lin_enc_count = linear_encoder->GetCounter(); // get motor encoder counts
    lin_pos_prev = lin_pos_curr;
    lin_pos_curr = lin_enc_count * lin_enc_scale; // convert counts to mm

    return lin_pos_curr;
}

float Sensors::getLinearSpeed(float period_s)
{
    lin_vel = (lin_pos_curr - lin_pos_prev) / period_s; // in rad/s

    return lin_vel;
}

void Sensors::setForceScale(float scale)
{
    force_scale = scale;
    printf("New force sensor scale = %f\n", force_scale);
}

float Sensors::getForce(void)
{
    force_data = force_sensor->read(); // read data from force sensor
    force_value = force_scale * (force_data - force_bias) + weight_offset; // get force in N

    return force_value;
}

void Sensors::calibrateForce(int measurements_number)
{
    float force_data_sum = 0;
    for (float i = 0; i < measurements_number; i++)
        force_data_sum += force_sensor->read();
    force_bias = force_data_sum / measurements_number;
    printf("Estimated force bias = %f\n", force_bias);
}

void Sensors::getDLPF(){
}



// class destructor
Sensors::~Sensors()
{
    printf("Sensors object was destructed.\n");
}