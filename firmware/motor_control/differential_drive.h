#ifndef DIFFERENTIAL_DRIVE_H
#define DIFFERENTIAL_DRIVE_H

#include "motor_with_encoder.h"

class DifferentialDrive {

public:
    DifferentialDrive(float update_period);

    void setOutput(float forward_velocity, float angular_velocity);

    void pulseLeftEncoder();
    void pulseRightEncoder();

    void updateState();

    float* getPulsesPerRadian();
    float* getWheelRadius();
    float* getWheelbaseLength();

    float* getVelocitykP();
    float* getVelocitykI();
    float* getVelocitykD();

    float* getAngularkP();
    float* getAngularkI();
    float* getAngularkD();

private:
    MotorWithEncoder left_motor_;
    MotorWithEncoder right_motor_;

    float update_period_ = 0.02f;
    float pulses_per_radian_ = 600f;
    float wheel_radius_ = 0.1f;
    float wheelbase_length_ = 0.4f;

    float forward_velocity_setpoint_;
    float angular_velocity_setpoint_;

    float computeVelocityPID_(float velocity_setpoint, float velocity_current);
    float velocity_kP_ = 0.01;
    float velocity_kI_ = 0.01;
    float velocity_kD_ = 0;
    float velocity_integrator_ = 0;
    float velocity_previous_error_ = 0;

    float computeAngularPID_(float angular_setpoint, float angular_current);
    float angular_kP_ = 0.01;
    float angular_kI_ = 0.01;
    float angular_kD_ = 0;
    float angular_integrator_ = 0;
    float angular_previous_error_ = 0;

    float pulsesToRadians_(int pulses);
};

inline DifferentialDrive::DifferentialDrive(float update_period) : update_period_(update_period) {}

inline void DifferentialDrive::setOutput(float forward_velocity, float angular_velocity) {
    forward_velocity_setpoint_ = forward_velocity;
    angular_velocity_setpoint_ = angular_velocity;
}

inline void DifferentialDrive::pulseLeftEncoder() {
    left_motor_.pulseEncoder();
}

inline void DifferentialDrive::pulseRightEncoder() {
    right_motor_.pulseEncoder();
}

inline void DifferentialDrive::updateState(float& delta_x_out, float& delta_y_out, float& delta_theta_out) {
    float left_motor_angular_distance = left_motor_.getPulses() / pulses_per_radian_;
    float right_motor_angular_distance = right_motor_.getPulses() / pulses_per_radian_;

    float distance_estimate = (wheel_radius_ / 2f) * (right_motor_angular_distance + left_motor_angular_distance);
    float rotation_estimate = (wheel_radius_ / wheelbase_length_) * (right_motor_angular_distance - left_motor_angular_distance);

    float velocity_estimate = distance_estimate / update_period_;
    float angular_estimate = rotation_estimate / update_period_;

    float velocity_control = computeVelocityPID_(forward_velocity_setpoint_, velocity_estimate);
    float angular_control = computeAngularPID_(angular_velocity_setpoint_, angular_estimate);

    left_motor_.setOutput(velocity_control - angular_control);
    right_motor_.setOutput(velocity_control + angular_control);

    delta_x_out = distance_estimate * cos(rotation_estimate);
    delta_y_out = distance_estimate * sin(rotation_estimate);
    delta_theta_out = rotation_estimate;
}

inline float DifferentialDrive::pulsesToRadians_(int pulses) {
    return (int)(pulses);
}

inline float DifferentialDrive::computeVelocityPID_(float velocity_setpoint, float velocity_current) {
    float velocity_error = velocity_setpoint - velocity_current;

    velocity_integrator_ += velocity_error * update_period_;
    float velocity_derivative = (velocity_error - velocity_previous_error_) / update_period_;

    velocity_previous_error_ = velocity_error;

    return velocity_kP_ * velocity_error + velocity_kI_ * velocity_integrator_ + velocity_kD_ * velocity_derivative;
}

inline float DifferentialDrive::computeAngularPID_(float angular_setpoint, float angular_current) {
    float angular_error = angular_setpoint - angular_current;

    angular_integrator_ += angular_error * update_period_;
    float angular_derivative = (angular_error - angular_previous_error_) / update_period_;

    angular_previous_error_ = angular_error;

    return angular_kP_ * angular_error + angular_kI_ * angular_integrator_ + angular_kD_ * angular_derivative;
}

inline float* DifferentialDrive::getPulsesPerRadian() {
    return &pulses_per_radian_;
}

inline float* DifferentialDrive::getWheelRadius() {
    return &wheel_radius_;
}

inline float* DifferentialDrive::getWheelbaseLength() {
    return &wheelbase_length_;
}

inline float* DifferentialDrive::getVelocitykP() {
    return &velocity_kP_;
}

inline float* DifferentialDrive::getVelocitykI() {
    return &velocity_kI_;
}

inline float* DifferentialDrive::getVelocitykD() {
    return &velocity_kD_;
}

inline float* DifferentialDrive::getAngularkP() {
    return &angular_kP_;
}

inline float* DifferentialDrive::getAngularkI() {
    return &angular_kI_;
}

inline float* DifferentialDrive::getAngularkD() {
    return &angular_kD_;
}

#endif