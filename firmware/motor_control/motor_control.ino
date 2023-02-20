#include <Servo.h>
#include "TickTwo.h"
#include <ACAN2515.h>
#include "motor.h"
#include "common.h"

const int ledPin = LED_BUILTIN;
const int CANLED = 0;
const int LED0 = 1;

const int encoderLeftA = 3;
const int encoderLeftB = 2;

const int encoderRightA = 5;
const int encoderRightB = 4;

const int leftMotorPwmPin = 16;
const int rightMotorPwmPin = 14;

const int estopPin = 27;


static const byte MCP2515_SCK = 10;   // SCK input of MCP2515
static const byte MCP2515_MOSI = 11;  // SDI input of MCP2515
static const byte MCP2515_MISO = 8;   // SDO output of MCP2515

static const byte MCP2515_CS = 9;   // CS input of MCP2515
static const byte MCP2515_INT = 7;  // INT output of MCP2515

ACAN2515 can(MCP2515_CS, SPI1, MCP2515_INT);
static const uint32_t QUARTZ_FREQUENCY = 8UL * 1000UL * 1000UL;  // 8 MHz

CANMessage frame;
CANMessage outFrame;

robotStatus_t roboStatus;
distance motorDistances;
motorCommand motoCommand;


motor leftMotor(leftMotorPwmPin, true);
motor rightMotor(rightMotorPwmPin, false);

void configureCan();
void printCanMsg();
void updateMsgData();
void updateLeft();
void updateRight();
void sendCanOdomMsgOut();
void resetDelta(float& dxn, float& dyn, float& don);

bool ledState;
int counterUS;

float dxn = 0;
float dyn = 0;
float don = 0;

bool blinky = false;


short leftSpeedSetpoint;
short rightSpeedSetpoint;


TickTwo timer1(setFiveMilliSecFlag, 5);      //TODO: CHANGE 5 ms PLACE HOLDERS
TickTwo timer2(setTenMilliSecFlag, 10);      //TODO: CHANGE 10 ms PLACE HOLDERS
TickTwo timer3(setFiftyMilliSecFlag, 50);  //TODO: CHANGE 50 ms PLACE HOLDERS CURRENTLY 1 SEC for debugging

static uint32_t gReceivedFrameCount = 0;


void setup() {
  Serial.begin(9600);
  delay(50);

  // NOTE: SPI 1 must be configured before the PWM is initilized (in this case CAN before left motor setup)
  configureCan();

  leftMotor.setup();
  rightMotor.setup();

  pinMode(LED0, OUTPUT);
  pinMode(CANLED, OUTPUT);
  pinMode(estopPin, INPUT_PULLDOWN);
  pinMode(encoderRightA, INPUT);
  pinMode(encoderRightB, INPUT);
  pinMode(encoderLeftA, INPUT);
  pinMode(encoderLeftB, INPUT);

  attachInterrupt(encoderLeftA, updateLeft, CHANGE);
  attachInterrupt(encoderRightA, updateRight, CHANGE);

  timer1.start();
  timer2.start();
  timer3.start();
}
void loop() {
  updateTimers();
  
  if (FIVE_MS_FLAG) {  //TODO: CHANGE PLACE HOLDERS
    FIVE_MS_FLAG = false;
  }
  if (TEN_MS_FLAG) {
    leftMotor.update();
    rightMotor.update();


    // Serial.println("Right speed");
    // Serial.println(rightMotor.getSpeedEstimate());
    // Serial.println("Left speed");
    // Serial.println(leftMotor.getSpeedEstimate());

    FIVE_MS_FLAG = false;
  }
  if (FIFTY_MS_FLAG ) {
    float left_distance = leftMotor.getDistance();
    float right_distance = rightMotor.getDistance();
    
    dxn = dxn + (left_distance + right_distance) / 2 * cos(don);
    dyn = dyn + (left_distance + right_distance) / 2 * sin(don); 
    don = don + (right_distance - left_distance) * DIAMETER_FROM_CENTER_WHEEL / DISTANCE_BETWEEN_WHEELS; //diametrer of center of wheel and diameter between wheel TODO: no magic numbers

    motorDistances.xn = (short)(dxn * SPEED_SCALE_FACTOR);
    motorDistances.yn = (short)(dyn * SPEED_SCALE_FACTOR);
    motorDistances.on = (short)(don * SPEED_SCALE_FACTOR);

    //sendCanOdomMsgOut();

    resetDelta(dxn, dyn, don);
    
    blinky = !blinky;
    digitalWrite(LED0, blinky);

    Serial.print("Right speed: ");
    Serial.println(right_distance);
    Serial.print("Left speed: ");
    Serial.println(left_distance);
    FIFTY_MS_FLAG = false;
  }
}

void updateTimers() {
  timer1.update();
  timer2.update();
  timer3.update();
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
    Serial.print("Actual bit rate: ");
    Serial.print(settings.actualBitRate());
  } else {
    Serial.print(errorCode);
  }
}

void onCanRecieve() {
  can.isr();
  can.receive(frame);  
  switch (frame.id) {
    case 0:
      Serial.print("ESTOP \n");
      roboStatus.eStop = 1;
      roboStatus.mStop = 1;
      roboStatus.mStart = 0;
      leftSpeedSetpoint = 0;
      rightSpeedSetpoint = 0;

      break;
    case 1:
      roboStatus.eStop = 1;
      roboStatus.mStop = 0;
      roboStatus.mStart = 0;
      leftSpeedSetpoint = 0;
      rightSpeedSetpoint = 0;

      break;

    case 10:
      Serial.print("10 ID \n");
      roboStatus.eStop = 0;
      roboStatus.mStop = 0;
      roboStatus.mStart = 1;
      motoCommand = *(motorCommand*)(frame.data);     //Noah made me cry. I dont know what they did but I dont like it one bit - Jorge
      leftMotor = motoCommand.setpointLeft/(float)SPEED_SCALE_FACTOR;
      rightMotor = motoCommand.setpointRight/(float)SPEED_SCALE_FACTOR;
      // leftSpeedSetpoint = (frame.data[0] << 8 & 0xFF00) | frame.data[1];
      // rightSpeedSetpoint = (frame.data[2] << 8 & 0xFF00) | frame.data[3];
      
      break;
  }
  Serial.print("LEFT: ");
  Serial.println(leftSpeedSetpoint);
  Serial.print("RIGHT: ");
  Serial.println(rightSpeedSetpoint);
  printCanMsg(frame);
}

void sendCanOdomMsgOut(){
  outFrame.id = ODOM_OUT_ID;
  outFrame.len = ODOM_OUT_LEN;
  memcpy(outFrame.data, &motorDistances, 6 );

  const bool ok = can.tryToSend (frame) ;
  if(ok){
    Serial.print ("Sent: Odom Data");
  }
  else{
    Serial.println ("Send failure") ;
  }

}
void printCanMsg(CANMessage frame) {
  Serial.print("  id: ");
  Serial.println(frame.id, HEX);
  Serial.print("  ext: ");
  Serial.println(frame.ext);
  Serial.print("  rtr: ");
  Serial.println(frame.rtr);
  Serial.print("  len: ");
  Serial.println(frame.len);
  Serial.print("  data: ");
  for (int x = 0; x < frame.len; x++) {
    Serial.print(frame.data[x], HEX);
    Serial.print(":");
  }
  Serial.println("");
}
void updateLeft(){
  //Serial.printf("Left update \n");
  leftMotor.pulse(digitalRead(encoderLeftA),digitalRead(encoderLeftB));
}
void updateRight(){
  //Serial.printf("Right update \n");
  rightMotor.pulse(digitalRead(encoderRightA),digitalRead(encoderRightB));
}
void resetDelta(float &dxn, float &dyn, float &don){
  dxn = 0;
  dyn = 0;
  don = 0;
  motorDistances.xn = 0;
  motorDistances.yn = 0;
  motorDistances.on = 0;
}
void updateMsgData() {
}

