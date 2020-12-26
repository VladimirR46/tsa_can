#ifndef SENSORS_H
#define SENSORS_H

#define FILTER_SAMPLE 32

#include <stdio.h>
// #include <cmath>

#include "TimEncoders/Nucleo_Encoder_16_bits.h"
#include "MPU9250.h"

// TODO: This class should collect certaint sensors, decorate them with function getReadings() ...
// add scales biases and etc

// TODO: Implement DLPF feature
//       Implement getDerivative() feature
//       Implement getIntegral() feature
//       Implement setScale, setBias, setSensor feature
//       Implement addSensor member function that should copy the instance of some class (IMU/Encoder/Force sensor) ...

class Sensors
{
public:
    // class constructor
    Sensors(Nucleo_Encoder_16_bits *motor_encoder,
            Nucleo_Encoder_16_bits *linear_encoder,
            AnalogIn *force_sensor /*,
            MPU9250 *imu*/
    );

    // imu
    void readIMU();
    void getIMUAccel(float *acell);

    // update motor encoder scale
    void setMotorScale(float scale);
    // read the data from the motor encoder
    float getMotorPosition(void);
    // calculate the motor speed using differences
    float getMotorSpeed(float period_s);

    uint16_t getMotorCountOneTurn();
    int16_t getMotorTurnCount();

    // update linear encoder scale
    void setLinearScale(float scale);
    // read the data from the linear encoder
    float getLinearPosition(void);
    // calculate the linear speed using differences
    float getLinearSpeed(float period_s);
    // Counter
    uint16_t getLinearCounter(void);

    // update force sensor scale
    void setForceScale(float scale);
    // read the data from the force sensor
    float getForce(void);
    // uint16_t
    uint16_t getForceU16();
    // calibrate force sensor
    void calibrateForce(int duration);
    // digital low pass filter
    void getDLPF();

    // class destructor
    ~Sensors();

private:
    Nucleo_Encoder_16_bits *motor_encoder;
    Nucleo_Encoder_16_bits *linear_encoder;
    AnalogIn *force_sensor;
    MPU9250 *imu;

    // Linear encoder settings
    int mot_enc_cpt = 1024; // counts per turn
    float mot_enc_scale = 2.0 * M_PI / (mot_enc_cpt * 4.0);
    int mot_enc_count = 0;
    int mot_enc_bias = 0;   // in counts
    float mot_pos_curr = 0; // in rad
    float mot_pos_prev = 0; // in rad
    float mot_vel = 0;      //in rad/s

    // Motor Encoder Settings
    int lin_enc_cpi = 360; // counts per inch
    float lin_enc_scale = 25.4 / (lin_enc_cpi * 4);
    int lin_enc_count = 0;
    int lin_enc_bias = 0;   // in counts
    float lin_pos_curr = 0; // in mm
    float lin_pos_prev = 0; // in mm
    float lin_vel = 0;      //in mm/s

    // Force Sensor Settings
    float force_scale = 94.315;
    float force_bias = 0.032;
    float weight_offset = 3.0;
    float force_data = 0;
    float force_value = 0; // in N

    float f_buff[FILTER_SAMPLE] = {0.0f};
    uint8_t f_index = 0;
};

#endif