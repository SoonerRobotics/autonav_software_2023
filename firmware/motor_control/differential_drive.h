#ifndef DIFFERENTIAL_DRIVE_H
#define DIFFERENTIAL_DRIVE_H

#include "motor_with_encoder.h"

class DifferentialDrive {

public:
    DifferentialDrive();

    void setOutput(float forward_velocity, float angular_velocity);

    void pulseLeftEncoder();
    void pulseRightEncoder();

    void updateState();

private:
    MotorWithEncoder left_motor_;
    MotorWithEncoder right_motor_;

    float forward_velocity_setpoint_;
    float angular_velocity_setpoint_;

    float computeVelocityPID_();
    float computeAngularPID_();
};

inline DifferentialDrive::DifferentialDrive() {

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

inline void DifferentialDrive::updateState() {
    float velocity_control = computeVelocityPID_();
    float angular_control = computeAngularPID_();

    left_motor_.setOutput(velocity_control - angular_control);
    right_motor_.setOutput(velocity_control + angular_control);
}

inline float DifferentialDrive::computeVelocityPID_() {
    // Compute kinematics

    // Update motors
}

inline float DifferentialDrive::computeAngularPID_() {
    // Compute kinematics

    // Update motors
}

#endif