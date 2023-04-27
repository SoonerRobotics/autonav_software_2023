#include <Servo.h>
#include "time.h"
#include <ACAN2515.h>
#include "differential_drive.h"
#include "motor_with_encoder.h"
#include "common.h"

//LEDs
const int ledPin = LED_BUILTIN;
const int CANLED = 0;
const int LED0 = 1;

//Encoder Pins
const int encoderLeftA = 3;
const int encoderLeftB = 2;
const int encoderRightA = 5;
const int encoderRightB = 4;
//Motor PWM pins
const int leftMotorPwmPin = 16;
const int rightMotorPwmPin = 14;

const int estopPin = 27;

//SPI pins
static const byte MCP2515_SCK = 10;   // SCK input of MCP2515
static const byte MCP2515_MOSI = 11;  // SDI input of MCP2515
static const byte MCP2515_MISO = 8;   // SDO output of MCP2515
static const byte MCP2515_CS = 9;   // CS input of MCP2515
static const byte MCP2515_INT = 7;  // INT output of MCP2515

ACAN2515 can(MCP2515_CS, SPI1, MCP2515_INT);

TickTwo timer1(setFiveMilliSecFlag, 5);      //TODO: CHANGE 5 ms PLACE HOLDERS
TickTwo timer2(setTwentyMilliSecFlag, 20);      //TODO: CHANGE 10 ms PLACE HOLDERS
TickTwo timer3(setFiftyMilliSecFlag, 50);  //TODO: CHANGE 50 ms PLACE HOLDERS 
TickTwo timer4(setFiveHundMilliSecFlag, 500);  //TODO: CHANGE 500 ms

static const uint32_t QUARTZ_FREQUENCY = 8UL * 1000UL * 1000UL;  // 8 MHz

CANMessage frame;
CANMessage outFrame;

robotStatus_t roboStatus;
distance motorDistances;
MotorCommand motorCommand;

// motor leftMotor(leftMotorPwmPin, true);
// motor rightMotor(rightMotorPwmPin, false);

MotorWithEncoder leftMotor(leftMotorPwmPin, encoderLeftA, encoderLeftB, true);
MotorWithEncoder rightMotor(rightMotorPwmPin, encoderRightA, encoderRightB, false);
DifferentialDrive drivetrain(leftMotor, rightMotor, 0.02);

void configureCan();
void printCanMsg();
void updateMsgData();
void updateLeft();
void updateRight();
void sendCanOdomMsgOut();
void resetDelta();

bool SEND_CAN_ODOM = false;
bool canBlinky = false;

short collisonBoxDist = 60;

float delta_x = 0;
float delta_y = 0;
float delta_theta = 0;

bool isSafe = true;

void setup() {
  delay(50);

  drivetrain.setup();

  pinMode(encoderRightA, INPUT);
  pinMode(encoderRightB, INPUT);
  pinMode(encoderLeftA, INPUT);
  pinMode(encoderLeftB, INPUT);

  attachInterrupt(encoderLeftA, updateLeft, CHANGE);
  attachInterrupt(encoderRightA, updateRight, CHANGE);

  timer1.start();
  timer2.start();
  timer3.start();
  timer4.start();
}
void setup1(){
  Serial.begin(9600);
  delay(50);
  configureCan();

  pinMode(LED0, OUTPUT);
  pinMode(CANLED, OUTPUT);
  pinMode(estopPin, INPUT);
}
void loop() {
  updateTimers();
  if (TWENTY_MS_FLAG) {

    drivetrain.updateState(delta_x, delta_y, delta_theta);

    TWENTY_MS_FLAG = false;
  }
  if (FIFTY_MS_FLAG ) {
    SEND_CAN_ODOM = true; //flag to send CAN out

    FIFTY_MS_FLAG = false;
  }
  if(FIVE_HUND_MS_FLAG){
    FIVE_HUND_MS_FLAG = false;
  }
}
void loop1(){
  if (SEND_CAN_ODOM) {
    sendCanOdomMsgOut();
    resetDelta();
    SEND_CAN_ODOM = false;
  }
}
void configureCan() {
  SPI1.setSCK(MCP2515_SCK);
  SPI1.setTX(MCP2515_MOSI);
  SPI1.setRX(MCP2515_MISO);
  SPI1.setCS(MCP2515_CS);
  SPI1.begin();


  Serial.println("Configure ACAN2515");
  ACAN2515Settings settings(QUARTZ_FREQUENCY, 100UL * 1000UL);  // CAN bit rate 100 kb/s
  settings.mRequestedMode = ACAN2515Settings::NormalMode ; // Select Normal mode
  const uint16_t errorCode = can.begin(settings, onCanRecieve );
  if (errorCode == 0) {
  }
  else{
    Serial.print("Error: ");
    Serial.print(errorCode);
  }
}
void onCanRecieve() {
  can.isr();
  can.receive(frame);  
  switch (frame.id) {
    case 10:
      motorCommand = *(MotorCommand*)(frame.data); 
      if(isSafe){
        drivetrain.setOutput((float)motorCommand.setpoint_forward_velocity / SPEED_SCALE_FACTOR, (float)motorCommand.setpoint_angular_velocity / SPEED_SCALE_FACTOR);
      }
      else if(isSafe = false && (float)motorCommand.setpoint_forward_velocity < 0.0){
        drivetrain.setOutput((float)motorCommand.setpoint_forward_velocity / SPEED_SCALE_FACTOR, (float)motorCommand.setpoint_angular_velocity / SPEED_SCALE_FACTOR);
      }
      break;
    case 20:
      Serial.print(frame.data[1]);
      Serial.print(frame.data[2]);
      Serial.println(frame.data[3]);
      if((frame.data[1] < collisonBoxDist || frame.data[2] < collisonBoxDist || frame.data[3] < collisonBoxDist) & 
        motorCommand.setpoint_forward_velocity > 0.0){
        drivetrain.setOutput(0.0, (float)motorCommand.setpoint_angular_velocity / SPEED_SCALE_FACTOR);
        isSafe = false;
      }
      else{
        isSafe = true;

      }
      break;
  }
}
void sendCanOdomMsgOut(){
  outFrame.id = ODOM_OUT_ID;
  outFrame.len = ODOM_OUT_LEN;

  motorDistances.xn = delta_x * ODOM_SCALE_FACTOR;
  motorDistances.yn = delta_y * ODOM_SCALE_FACTOR;
  motorDistances.on = delta_theta * ODOM_SCALE_FACTOR;

  memcpy(outFrame.data, &motorDistances, ODOM_OUT_LEN );
  const bool ok = can.tryToSend (outFrame) ;

}
void printCanMsg(CANMessage frame) {
  Serial.print("  id: ");
  Serial.println(frame.id, HEX);
  Serial.println(frame.len);
  Serial.print("  data: ");
  for (int x = 0; x < frame.len; x++) {
    Serial.print(frame.data[x], HEX);
    Serial.print(":");
  }
  Serial.println("");
}
void updateLeft(){
  drivetrain.pulseLeftEncoder();
}
void updateRight(){
  drivetrain.pulseRightEncoder();
}
void resetDelta(){
  motorDistances.xn = 0;
  motorDistances.yn = 0;
  motorDistances.on = 0;
  delta_x = 0;
  delta_y = 0;
  delta_theta = 0;
}
void updateTimers() {
  timer1.update();
  timer2.update();
  timer3.update();
  timer4.update();
}

