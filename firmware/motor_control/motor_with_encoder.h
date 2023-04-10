#ifndef MOTOR_WITH_ENCODER_H
#define MOTOR_WITH_ENCODER_H

#include <stdio.h>

class MotorWithEncoder {

public:
    MotorWithEncoder(uint8_t pin_PWM_Control, uint8_t pin_EncoderA, uint8_t pin_EncoderB, bool reversed);

    void setup();

    void pulseEncoder();
    int getPulses();

    void setOutput(float control);
    void setMinOutput(float minControl);

private:
    float minControl_ = 0.01f;
    float maxControl_ = 0.90f;

    Servo servoController_;

    const uint8_t pin_PWM_Control_;
    const uint8_t pin_EncoderA_;
    const uint8_t pin_EncoderB_;
    const bool reversed_;

    int encoder_pulses_ = 0;
    int previous_Encoder_State_ = 0;
};

inline MotorWithEncoder::MotorWithEncoder(uint8_t pin_PWM_Control, uint8_t pin_EncoderA, uint8_t pin_EncoderB, bool reversed)
    : pin_PWM_Control_(pin_PWM_Control), pin_EncoderA_(pin_EncoderA), pin_EncoderB_(pin_EncoderB), reversed_(reversed) {}

inline void MotorWithEncoder::setup() {
    servoController_.attach(pin_PWM_Control_, 1000, 2000);
}

/*
    Set motor output on a scale of -1.0 to 1.0
*/
inline void MotorWithEncoder::setOutput(float control) {
    
    int servoOut = 90; // Default to braked

    if (control > 1) {
      control = 1;
    }

    if (control < -1) {
      control = -1;
    }

    if (reversed_) {
        control = -control;
    }

    if (abs(control) > maxControl_) { 
      control = abs(control) / control * maxControl_;
    }

    // If we are greater than our min control signal, set the servo output
    if ((control > 0 && control > minControl_) || (control < 0 && control < -minControl_)) {
        servoOut = 90 + control * 85;
    }

    servoController_.write(servoOut);
}


inline void MotorWithEncoder::pulseEncoder() {
    int encoderA = digitalRead(pin_EncoderA_);
    int encoderB = digitalRead(pin_EncoderB_);

    int current_Encoder_State_ = (encoderA << 1) | (encoderB);

    //11->00->11->00 is counter clockwise rotation or "forward".
    if ((previous_Encoder_State_ == 0x3 && current_Encoder_State_ == 0x0) || (previous_Encoder_State_ == 0x0 && current_Encoder_State_ == 0x3)) {
      encoder_pulses_++;
    }
    //10->01->10->01 is clockwise rotation or "backward".
    else if ((previous_Encoder_State_ == 0x2 && current_Encoder_State_ == 0x1) || (previous_Encoder_State_ == 0x1 && current_Encoder_State_ == 0x2)) {
      encoder_pulses_--;
    }

    previous_Encoder_State_ = current_Encoder_State_;
}

inline int MotorWithEncoder::getPulses() {
    int temp = encoder_pulses_;
    encoder_pulses_ = 0;

    if (reversed_) {
        temp = -temp;
    }

    return temp;
}


#endif