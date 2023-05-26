#ifndef DIFFERENTIAL_DRIVE_H
#define DIFFERENTIAL_DRIVE_H

#include "motor_with_encoder.h"
#include "common.h"

class DifferentialDrive {

public:
    DifferentialDrive(MotorWithEncoder left_motor, MotorWithEncoder right_motor, float update_period);

    void setup();

    void setOutput(float forward_velocity, float angular_velocity);

    void pulseLeftEncoder();
    void pulseRightEncoder();

    void updateState(float& delta_x_out, float& delta_y_out, float& delta_theta_out);

    void getSetpoints(PIDSetpoints& pid_setpoints);
    void getControl(PIDControl& pid_control);

    float* getUpdatePeriod();
    float* getPulsesPerRadian();
    float* getWheelRadius();
    float* getWheelbaseLength();
    float* getSlewRateLimit();
    float* getLeftEncoderFactor();
    float* getRightEncoderFactor();

    float* getVelocitykP();
    float* getVelocitykI();
    float* getVelocitykD();
    float* getVelocitykF();

    float* getAngularkP();
    float* getAngularkI();
    float* getAngularkD();
    float* getAngularkF();

private:
    MotorWithEncoder left_motor_;
    MotorWithEncoder right_motor_;

    float update_period_ = 0.025f;
    float pulses_per_radian_ = 600.0f * 20.0f / 16.8f;
    float wheel_radius_ = 0.135f;
    float wheelbase_length_ = 0.45f;
    float left_encoder_factor_ = 1.00f;
    float right_encoder_factor_ = 1.01f;

    float forward_velocity_setpoint_;
    float angular_velocity_setpoint_;

    float velocity_estimate;
    float angular_estimate;

    float left_motor_output;
    float right_motor_output;

    float computeVelocityPID_(float velocity_setpoint, float velocity_current);
    float velocity_kP_ = 0.1;
    float velocity_kI_ = 1.0;
    float velocity_kD_ = 0;
    float velocity_kF_ = 0.15;
    float velocity_integrator_ = 0;
    float velocity_previous_error_ = 0;

    float computeAngularPID_(float angular_setpoint, float angular_current);
    float angular_kP_ = 0.1;
    float angular_kI_ = 0.4;
    float angular_kD_ = 0;
    float angular_kF_ = 0.15;
    float angular_integrator_ = 0;
    float angular_previous_error_ = 0;

    float pulsesToRadians_(int pulses);

    float slewLimit_(float current_output, float desired_output);
    float slew_rate_limit_ = 0.05;
};

inline DifferentialDrive::DifferentialDrive(MotorWithEncoder left_motor, MotorWithEncoder right_motor, float update_period)
    : update_period_(update_period), left_motor_(left_motor), right_motor_(right_motor) {}

inline void DifferentialDrive::setup() {
  left_motor_.setup();
  right_motor_.setup();
}

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
    float left_motor_angular_distance = left_motor_.getPulses() / pulses_per_radian_ * left_encoder_factor_;
    float right_motor_angular_distance = right_motor_.getPulses() / pulses_per_radian_ * right_encoder_factor_;

    float distance_estimate = (wheel_radius_ / 2.0f) * (right_motor_angular_distance + left_motor_angular_distance);
    float rotation_estimate = (wheel_radius_ / wheelbase_length_) * (right_motor_angular_distance - left_motor_angular_distance);

    velocity_estimate = distance_estimate / update_period_;
    angular_estimate = rotation_estimate / update_period_;

    float velocity_control = computeVelocityPID_(forward_velocity_setpoint_, velocity_estimate);
    float angular_control = computeAngularPID_(angular_velocity_setpoint_, angular_estimate);

    if (abs(forward_velocity_setpoint_) < 0.05 && abs(angular_velocity_setpoint_) < 0.05 && abs(velocity_estimate) < 0.05 && abs(angular_estimate) < 0.05) {
      // estop fix
      velocity_integrator_ = 0;
      angular_integrator_ = 0;

      left_motor_output = 0;
      right_motor_output = 0;
    } else {
      left_motor_output = slewLimit_(left_motor_output, velocity_control - angular_control);
      right_motor_output = slewLimit_(right_motor_output, velocity_control + angular_control);
    }

    left_motor_.setOutput(left_motor_output);
    right_motor_.setOutput(right_motor_output);

    float estimated_theta = delta_theta_out + 0.5 * rotation_estimate;
    delta_x_out += distance_estimate * cos(estimated_theta);
    delta_y_out += distance_estimate * sin(estimated_theta);
    delta_theta_out += rotation_estimate;
}

inline float DifferentialDrive::pulsesToRadians_(int pulses) {
    return (int)(pulses);
}

inline float DifferentialDrive::slewLimit_(float current_output, float desired_output) {
    if (desired_output - current_output > slew_rate_limit_) {
      return current_output + slew_rate_limit_;
    } else if (desired_output - current_output < -slew_rate_limit_) {
      return current_output - slew_rate_limit_;
    } else {
      return desired_output;
    }
}

inline float DifferentialDrive::computeVelocityPID_(float velocity_setpoint, float velocity_current) {
    float velocity_error = velocity_setpoint - velocity_current;

    velocity_integrator_ += velocity_error * update_period_;
    float velocity_derivative = (velocity_error - velocity_previous_error_) / update_period_;

    velocity_previous_error_ = velocity_error;

    return velocity_kP_ * velocity_error + velocity_kI_ * velocity_integrator_ + velocity_kD_ * velocity_derivative + velocity_kF_ * velocity_setpoint;
}

inline float DifferentialDrive::computeAngularPID_(float angular_setpoint, float angular_current) {
    float angular_error = angular_setpoint - angular_current;

    angular_integrator_ += angular_error * update_period_;
    float angular_derivative = (angular_error - angular_previous_error_) / update_period_;

    angular_previous_error_ = angular_error;

    return angular_kP_ * angular_error + angular_kI_ * angular_integrator_ + angular_kD_ * angular_derivative + angular_kF_ * angular_setpoint;
}

inline float* DifferentialDrive::getUpdatePeriod() {
    return &update_period_;
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

inline float* DifferentialDrive::getLeftEncoderFactor() {
    return &left_encoder_factor_;
}

inline float* DifferentialDrive::getRightEncoderFactor() {
    return &right_encoder_factor_;
}

inline float* DifferentialDrive::getSlewRateLimit() {
    return &slew_rate_limit_;
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

inline float* DifferentialDrive::getVelocitykF() {
    return &velocity_kF_;
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

inline float* DifferentialDrive::getAngularkF() {
    return &angular_kF_;
}

inline void DifferentialDrive::getSetpoints(PIDSetpoints& pid_setpoints) {
  pid_setpoints.forward_current = velocity_estimate * 1000;
  pid_setpoints.forward_setpoint = forward_velocity_setpoint_ * 1000;
  pid_setpoints.angular_current = angular_estimate * 1000;
  pid_setpoints.angular_setpoint = angular_velocity_setpoint_ * 1000;
}

inline void DifferentialDrive::getControl(PIDControl& pid_control) {
  pid_control.left_motor_output = left_motor_output * 1000;
  pid_control.right_motor_output = right_motor_output * 1000;
}

#endif