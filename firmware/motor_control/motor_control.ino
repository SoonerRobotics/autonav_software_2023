#include <Servo.h>
#include "time.h"
#include <ACAN2515.h>
#include "differential_drive.h"
#include "motor_with_encoder.h"
#include "common.h"
#include <CONBus.h> 
#include <CANBusDriver.h>

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

TickTwo motor_update_timer(setMotorUpdateFlag, 25);

static const uint32_t QUARTZ_FREQUENCY = 8UL * 1000UL * 1000UL;  // 8 MHz

CANMessage frame;
CANMessage outFrame;

// Setup CONBus variables
CONBus::CONBus conbus;
CONBus::CANBusDriver conbus_can(conbus, 0x10); // device id 0x10 (16)

robotStatus_t roboStatus;
distance motorDistances;
MotorCommand motorCommand;

// motor leftMotor(leftMotorPwmPin, true);
// motor rightMotor(rightMotorPwmPin, false);

MotorWithEncoder leftMotor(leftMotorPwmPin, encoderLeftA, encoderLeftB, true);
MotorWithEncoder rightMotor(rightMotorPwmPin, encoderRightA, encoderRightB, false);
DifferentialDrive drivetrain(leftMotor, rightMotor, 0.025);

void configureCan();
void printCanMsg();
void updateMsgData();
void updateLeft();
void updateRight();
void sendCanOdomMsgOut();
void resetDelta();

int motor_updates_in_deltaodom = 0;
uint32_t motor_updates_between_deltaodom = 3;
bool canBlinky = false;

bool useObstacleAvoidance = false;
uint32_t collisonBoxDist = 20;
bool isDetectingObstacle = false;

bool sendStatistics = true;

float delta_x = 0;
float delta_y = 0;
float delta_theta = 0;

float desired_forward_velocity;
float desired_angular_velocity;

void setup() {
  delay(50);

  drivetrain.setup();

  pinMode(encoderRightA, INPUT);
  pinMode(encoderRightB, INPUT);
  pinMode(encoderLeftA, INPUT);
  pinMode(encoderLeftB, INPUT);

  attachInterrupt(encoderLeftA, updateLeft, CHANGE);
  attachInterrupt(encoderRightA, updateRight, CHANGE);

  motor_update_timer.start();
}
void setup1(){
  Serial.begin(9600);
  delay(50);
  configureCan();

  pinMode(LED0, OUTPUT);
  pinMode(CANLED, OUTPUT);
  pinMode(estopPin, INPUT);

  conbus.addReadOnlyRegister(0x00, drivetrain.getUpdatePeriod());
  conbus.addRegister(0x01, drivetrain.getPulsesPerRadian());
  conbus.addRegister(0x02, drivetrain.getWheelRadius());
  conbus.addRegister(0x03, drivetrain.getWheelbaseLength());
  conbus.addRegister(0x04, drivetrain.getSlewRateLimit());
  conbus.addRegister(0x05, drivetrain.getLeftEncoderFactor());
  conbus.addRegister(0x06, drivetrain.getRightEncoderFactor());

  conbus.addRegister(0x10, drivetrain.getVelocitykP());
  conbus.addRegister(0x11, drivetrain.getVelocitykI());
  conbus.addRegister(0x12, drivetrain.getVelocitykD());
  conbus.addRegister(0x13, drivetrain.getVelocitykF());

  conbus.addRegister(0x20, drivetrain.getAngularkP());
  conbus.addRegister(0x21, drivetrain.getAngularkI());
  conbus.addRegister(0x22, drivetrain.getAngularkD());
  conbus.addRegister(0x23, drivetrain.getAngularkF());

  conbus.addRegister(0x30, &useObstacleAvoidance);
  conbus.addRegister(0x31, &collisonBoxDist);

  conbus.addRegister(0x40, &sendStatistics);

  conbus.addRegister(0x50, &motor_updates_between_deltaodom);
}
void loop() {
  updateTimers();
  if (MOTOR_UPDATE_FLAG) {

    drivetrain.updateState(delta_x, delta_y, delta_theta);
    motor_updates_in_deltaodom++;

    MOTOR_UPDATE_FLAG = false;
  }
}
void loop1(){

  if (motor_updates_in_deltaodom >= motor_updates_between_deltaodom) {
    motor_updates_in_deltaodom = 0;
    sendCanOdomMsgOut();
    resetDelta();
  }

  if (conbus_can.isReplyReady()) {
    conbus_can.peekReply(outFrame.id, outFrame.len, outFrame.data);

    bool success = can.tryToSend(outFrame);

    if (success) {
      conbus_can.popReply();
    }
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

  if (!can.available()) {
    return;
  }

  can.receive(frame);  

  conbus_can.readCanMessage(frame.id, frame.data);

  switch (frame.id) {
    case 10:
      motorCommand = *(MotorCommand*)(frame.data);     //Noah made me cry. I dont know what they did but I dont like it one bit - Jorge

      desired_forward_velocity = (float)motorCommand.setpoint_forward_velocity / SPEED_SCALE_FACTOR;
      desired_angular_velocity = (float)motorCommand.setpoint_angular_velocity / SPEED_SCALE_FACTOR;

      if (useObstacleAvoidance && isDetectingObstacle && desired_forward_velocity > 0) {
        desired_forward_velocity = 0;
      }

      drivetrain.setOutput(desired_forward_velocity, desired_angular_velocity);
      break;
    case 20:
      isDetectingObstacle = (frame.data[1] < collisonBoxDist || frame.data[2] < collisonBoxDist || frame.data[3] < collisonBoxDist);

      if (useObstacleAvoidance && isDetectingObstacle && desired_forward_velocity > 0) {
        desired_forward_velocity = 0;
      }

      drivetrain.setOutput(desired_forward_velocity, desired_angular_velocity);
      break;
  }
}

PIDSetpoints pid_setpoints;
PIDControl pid_controls;

void sendCanOdomMsgOut(){
  outFrame.id = ODOM_OUT_ID;
  outFrame.len = ODOM_OUT_LEN;

  motorDistances.xn = delta_x * ODOM_SCALE_FACTOR;
  motorDistances.yn = delta_y * ODOM_SCALE_FACTOR;
  motorDistances.on = delta_theta * ODOM_SCALE_FACTOR;

  memcpy(outFrame.data, &motorDistances, ODOM_OUT_LEN );
  const bool ok = can.tryToSend (outFrame) ;

  if (sendStatistics) {
    drivetrain.getSetpoints(pid_setpoints);
    outFrame.id = 50;
    outFrame.len = 8;
    memcpy(outFrame.data, &pid_setpoints, 8);
    can.tryToSend(outFrame);

    drivetrain.getControl(pid_controls);
    outFrame.id = 51;
    outFrame.len = 4;
    memcpy(outFrame.data, &pid_controls, 4);
    can.tryToSend(outFrame);
  }

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
  motor_update_timer.update();
}

