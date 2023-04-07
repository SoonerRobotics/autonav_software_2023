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
bool canBlinky = false;


short leftSpeedSetpoint;
short rightSpeedSetpoint;


TickTwo timer1(setFiveMilliSecFlag, 5);      //TODO: CHANGE 5 ms PLACE HOLDERS
TickTwo timer2(setTenMilliSecFlag, 10);      //TODO: CHANGE 10 ms PLACE HOLDERS
TickTwo timer3(setFiftyMilliSecFlag, 50);  //TODO: CHANGE 50 ms PLACE HOLDERS 
TickTwo timer4(setFiveHundMilliSecFlag, 500);  //TODO: CHANGE 500 ms


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
  pinMode(estopPin, INPUT);
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
void loop() {
  updateTimers();
  
  if (FIVE_MS_FLAG) {  //TODO: CHANGE PLACE HOLDERS
    FIVE_MS_FLAG = false;
  }
  if (TEN_MS_FLAG) {
    leftMotor.update();
    rightMotor.update();

    float left_distance = leftMotor.getDistance();
    float right_distance = rightMotor.getDistance();

    don = don + (right_distance - left_distance) * DIAMETER_FROM_CENTER_WHEEL / DISTANCE_BETWEEN_WHEELS * 10; //diametrer of center of wheel and diameter between wheel
    dxn = dxn + (left_distance + right_distance) / 2 * cos(don);
    dyn = dyn + (left_distance + right_distance) / 2 * sin(don); 

    motorDistances.xn = motorDistances.xn + (short)(dxn * ODOM_SCALE_FACTOR);
    motorDistances.yn = motorDistances.yn + (short)(dyn * ODOM_SCALE_FACTOR);
    motorDistances.on = motorDistances.on + (short)(don * ODOM_SCALE_FACTOR);

    // outFrame.id = 73;
    // outFrame.len = 8;
    // memcpy(outFrame.data, &motorDistances, 6 );
    //printCanMsg(outFrame);
  
    // Serial.println("Right speed");
    // Serial.println(rightMotor.getSpeedEstimate());
    // Serial.println("Left speed");
    // Serial.println(leftMotor.getSpeedEstimate());

    TEN_MS_FLAG = false;
  }
  if (FIFTY_MS_FLAG ) {

    // Serial.print("Left Dist: ");
    // Serial.println(left_distance);

    // Serial.print("Right Dist: ");
    // Serial.println(right_distance);
    
    sendCanOdomMsgOut();

    resetDelta(dxn, dyn, don);

    // Serial.print("Right speed: ");
    // Serial.println(right_distance);
    // Serial.print("Left speed: ");
    // Serial.println(left_distance);    

    FIFTY_MS_FLAG = false;
  }
  if(FIVE_HUND_MS_FLAG){
    FIVE_HUND_MS_FLAG = false;
  }
}

void updateTimers() {
  timer1.update();
  timer2.update();
  timer3.update();
  timer4.update();
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
  // canBlinky = !canBlinky;
  // digitalWrite(CANLED, canBlinky);

  can.isr();
  can.receive(frame);  
  switch (frame.id) {
    case 0:
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
      roboStatus.eStop = 0;
      roboStatus.mStop = 0;
      roboStatus.mStart = 1;
      motoCommand = *(motorCommand*)(frame.data);     //Noah made me cry. I dont know what they did but I dont like it one bit - Jorge
      leftMotor = ((float)motoCommand.setpointLeft)/(float)SPEED_SCALE_FACTOR;
      rightMotor = ((float)motoCommand.setpointRight)/(float)SPEED_SCALE_FACTOR;
      break;
  }
}

void sendCanOdomMsgOut(){
  outFrame.id = ODOM_OUT_ID;
  outFrame.len = ODOM_OUT_LEN;
  memcpy(outFrame.data, &motorDistances, ODOM_OUT_LEN );

  //printCanMsg(outFrame);

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

