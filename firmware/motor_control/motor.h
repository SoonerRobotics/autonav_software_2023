#ifndef MOTOR_H
#define MOTOR_H

#include <Servo.h>

#define ServoMinWidth 1000
#define ServoMaxWidth 2000

#define MOTOR_UPDATE_RATE 100            // Frequency that motor PID is updated (Hz)
#define MAX_SPEED 2.2f                   // (m/s)
#define PULSES_PER_REV (600.0f * 12 * 2 * 24 / 16)  // (revs)
//#define LINEAR_PER_REV 0.2054f // Wheel radius (m) old
#define LINEAR_PER_REV 0.2667f  // Wheel radius (m)
#define MILLIS_TO_FULL 250      // Milliseconds to go from 0 output speed to 1
#define LPIIR_DECAY 0.1f        // Decay rate of low pass filter on velocity

#define INCREMENT_AMT (1000.0f / (MILLIS_TO_FULL * MOTOR_UPDATE_RATE))

#define PREV_MASK 0x1  // Mask for the previous state in determining direction of rotation.
#define CURR_MASK 0x2  // Mask for the current state in determining direction of rotation.
#define INVALID 0x3    // XORing two states where both bits have changed.

#define DEADZONE 5

class motor {
  int PWM_Pin;

  bool disabled = LOW;
  int defaultOnZero = HIGH;

  float targetSpeed = 0.0f;
  float currentOutput = 0.0f;
  float currentSpeed = 0.0f;
  float speedEstimate = 0.0f;

  // PID
  float integrator = 0.0f;
  float error = 0.0f;
  // states
  float prevError = 0.0f;
  float lastState = 0.0f;

  int prevState_;
  int currState_;
  volatile int pulses = 0;

  volatile int delta = 0;

  // equations
  float kp = 0.05f;
  float ki = 0.0f;
  float kd = 0.0f;
  bool reverse = false;

  Servo motorServo;
public:
  motor(int PWM_Motor, bool reverse = false) {
    this->PWM_Pin = PWM_Motor;
    this->reverse = reverse;
  }

  void  setup() {
    this->motorServo.attach(this->PWM_Pin, ServoMinWidth, ServoMaxWidth);
  }

  void output(float speed) {
    if (this->reverse) {
      //speed = -speed;
    }
    this->targetSpeed = speed;
  }
  //stolen from igvc 21 / QEI library
  void pulse(int left, int right) {
    //Serial.printf("left %d, right %d \n", left, right);
    currState_ = (left << 1) | (right);

    //11->00->11->00
    //11->00->11->00 is counter clockwise rotation or "forward".
    if ((prevState_ == 0x3 && currState_ == 0x0) || (prevState_ == 0x0 && currState_ == 0x3)) {
      this->pulses++;
      this->delta++;
    }
    //10->01->10->01 is clockwise rotation or "backward".
    else if ((prevState_ == 0x2 && currState_ == 0x1) || (prevState_ == 0x1 && currState_ == 0x2)) {
      this->pulses--;
      this->delta--;
    }

    prevState_ = currState_;
    if (currentOutput < 0.01f && currentOutput > -0.01f) {  // break
    }
  }
  void brake() {
    this->motorServo.write(90);
  }
  int out2servo(float out) {

    if (this->reverse)
      out = -out;
    out = out / MAX_SPEED;
    int servoOut;
    //Serial.printf("servo in %f \n",out);
    if (out < 0.01f && out > -0.01f) {  // break
      servoOut = 90;
    } else if (currentOutput > 0.0f) {  // forward
      out = out * 85;
      servoOut = (int)(90 + DEADZONE + out);
    } else  //backward
    {
      out = out * 85;
      servoOut = (int)(90 - DEADZONE + out);
    }
    //Serial.printf("servo out %f \n",(float)(servoOut -90)/ 180);
    return servoOut;
  }

  void update() {
    //printf("pulses %d", this->pulses);
    float instantaneousSpeed = this->pulses / (float)PULSES_PER_REV * 2.0 * PI * LINEAR_PER_REV * MOTOR_UPDATE_RATE;
    if (this->reverse) {
      instantaneousSpeed = -instantaneousSpeed;
    }
    //Serial.printf("Instantaneuos speed %f \n", instantaneousSpeed);
    this->speedEstimate += (1.0f - LPIIR_DECAY) * (instantaneousSpeed - this->speedEstimate);  // Low pass filter our speed estimate
    float pidOUT = this->updatePID(this->targetSpeed, this->speedEstimate);
    //Serial.printf("pid output %f \n", pidOUT);
    currentOutput += pidOUT;

    currentOutput = clamp(currentOutput, -MAX_SPEED, MAX_SPEED);
    //Serial.println("curr output");
    //Serial.println(currentOutput);


    // "Fixes the estop problem" - noah
    if (this->pulses < 10 && abs(currentOutput) > 0.01 && abs(this-> targetSpeed) < 0.01) {
        this->integrator = 0;
        this->speedEstimate = 0;
        this-> lastState = 0;
        currentOutput = 0;
    }

    this->pulses = 0;
    int servoOut = out2servo(currentOutput);
    motorServo.write(servoOut);
  }
  float getDistance() {
    float temp = this->delta / PULSES_PER_REV * LINEAR_PER_REV * PI * 2.0 * 2.0;
    this->delta = 0;
    return temp;
  }

  float getSpeedEstimate() {
    //Serial.printf("target speed: %f \n", this->targetSpeed);

    return this->speedEstimate;
  }

  int getPulses() {
    return this->pulses;
  }

  motor& operator=(float v) {
    output(v);
    return *this;
  }
  // Adapted (Stolen) from RobotLib :)
  float updatePID(float target_state, float cur_state) {
    // Declare local variables
    float P, I, D;
    float result;
    float slope;
    float dt;

    // Get the time step
    dt = 1.0f / MOTOR_UPDATE_RATE;

    // Calculate error
    this->error = target_state - cur_state;
    //Serial.printf("error %f \n", this->error);

    // Integrate error using trapezoidal Riemann sums
    this->prevError = target_state - this->lastState;
    this->integrator += 0.5f * (this->error + this->prevError) * dt;

    // Find the slope of the error curve using secant approximation
    slope = (cur_state - this->lastState) / dt;

    // Apply PID gains
    P = this->kp * this->error;
    I = this->ki * this->integrator;
    D = this->kd * slope;
    I = clamp(I, -INCREMENT_AMT * .25, INCREMENT_AMT * .25);
    // Sum P, I, D to get the result of the equation
    // Bind the output if needed
    result = clamp(P + I + D, -INCREMENT_AMT, INCREMENT_AMT);

    // Update timing and increment to the next state
    this->lastState = cur_state;

    // Return the PID result
    return result;
  }

  static float clamp(float val, float min, float max) {
    if (val > max) {
      return max;
    } else if (val < min) {
      return min;
    }
    return val;
  }

  void resetError() {
    this->error = 0.0f;
    this->prevError = 0.0f;
    this->lastState = 0.0f;
  }
};

#endif