#ifndef MOTOR_COMMON_H
#define MOTOR_COMMON_H

//in meters
float DIAMETER_FROM_CENTER_WHEEL = 0.138;
float DISTANCE_BETWEEN_WHEELS = 0.44;

uint32_t SPEED_SCALE_FACTOR = 1000;
uint32_t ODOM_SCALE_FACTOR = 10000;


uint32_t ODOM_OUT_ID = 14; 
uint32_t ODOM_OUT_LEN = 6;
typedef struct{
    unsigned int eStop : 1;
    unsigned int mStop : 1;
    unsigned int mStart : 1;
}robotStatus_t;

typedef  struct{
      short xn;
      short yn;
      short on;
}distance;

typedef struct{
  short setpoint_forward_velocity;
  short setpoint_angular_velocity;
} MotorCommand;

typedef  struct{
      short forward_current;
      short forward_setpoint;
      short angular_current;
      short angular_setpoint;
} PIDSetpoints;

typedef  struct{
      short left_motor_output;
      short right_motor_output;
} PIDControl;

#endif