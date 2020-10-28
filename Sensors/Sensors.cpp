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
    f_buff[f_index] = mot_pos_curr;

    if (f_index >= FILTER_SAMPLE - 1)
        f_index = 0;
    else
        f_index++;

    mot_vel = (mot_pos_curr - f_buff[f_index]) / (FILTER_SAMPLE * period_s);

    //mot_vel = (mot_pos_curr - mot_pos_prev) / period_s; // in rad/s

    return mot_vel;
}

uint16_t Sensors::getMotorCountOneTurn()
{
    return (abs(motor_encoder->GetCounter()) / 4) % mot_enc_cpt;
}

int16_t Sensors::getMotorTurnCount() // 9 bit
{
    int32_t motor_count = motor_encoder->GetCounter();
    int16_t count = motor_count / (mot_enc_cpt * 4);
    if (motor_count < 0)
        count -= 1;
    return count;
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

uint16_t Sensors::getLinearCounter(void)
{
    return linear_encoder->GetCounter();
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
    force_data = force_sensor->read();                                     // read data from force sensor
    force_value = force_scale * (force_data - force_bias) + weight_offset; // get force in N

    return force_value;
}

uint16_t Sensors::getForceU16()
{
    return force_sensor->read_u16();
}

void Sensors::calibrateForce(int measurements_number)
{
    float force_data_sum = 0;
    for (float i = 0; i < measurements_number; i++)
        force_data_sum += force_sensor->read();
    force_bias = force_data_sum / measurements_number;
    printf("Estimated force bias = %f\n", force_bias);
}

void Sensors::getDLPF()
{
}

// class destructor
Sensors::~Sensors()
{
    printf("Sensors object was destructed.\n");
}