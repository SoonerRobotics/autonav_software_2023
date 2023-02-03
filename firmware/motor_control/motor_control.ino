#include "TickTwo.h"
#include <ACAN2515.h>

const int ledPin = LED_BUILTIN;

const int encoderLeftB = 3;
const int encoderLeftA = 2;
const int encoderRightA = 4;
const int encoderRightB = 5; 

const int leftMotorPwmPin = 12;
const int rightMotorPwmPin = 14;

const int estopPin = 27;

static const byte MCP2515_SCK  = 10 ; // SCK input of MCP2515
static const byte MCP2515_MOSI = 11; // SDI input of MCP2515
static const byte MCP2515_MISO = 8 ; // SDO output of MCP2515

static const byte MCP2515_CS  = 9 ;  // CS input of MCP2515
static const byte MCP2515_INT = 7 ;  // INT output of MCP2515

motor leftMotor(leftMotorPin, true);
motor rightMotor(rightMotorPin, false);

//TODO: CAN DEFINE

void setFiveMilliSecFlag();
void setTenMilliSecFlag();
void setFiftyMilliSecFlag();
void blink();

bool ledState;
int counterUS;

TickTwo timer1(setFiveMilliSecFlag, 5); //TODO: CHANGE 5 ms PLACE HOLDERS
TickTwo timer2(setTenMilliSecFlag, 10); //TODO: CHANGE 10 ms PLACE HOLDERS
TickTwo timer3(setFiftyMilliSecFlag, 50); //TODO: CHANGE 50 ms PLACE HOLDERS

bool FIVE_MS_FLAG = false; //TODO : flag to keep track of 5 ms flag
bool TEN_MS_FLAG = false; //TODO : flag to keep track of 10 ms flag
bool FIFTY_MS_FLAG = false; //TODO: flag to keep track of 50 ms flag


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  delay(2000);
  timer1.start();
  timer2.start();
  timer3.start();
  }

void loop() {
  updateTimers();

  if(FIVE_MS_FLAG){ //TODO: CHANGE PLACE HOLDERS

    UpdateLeft();
    UpdateRight();

    FIVE_MS_FLAG = false;
  }
  if(TEN_MS_FLAG){

    ReadCAN();
    UpdatePID();

    FIVE_MS_FLAG = false;
  }
  if(FIFTY_MS_FLAG){

    FIFTY_MS_FLAG = false;
    
  }

void updateTimers() {
  timer1.update();
  timer2.update();
  timer3.update();
  }
void setFiveMilliSecFlag() {
  FIVE_MS_FLAG = true;
  }

void setTenMilliSecFlag() {
  TEN_MS_FLAG = true;
  }

void setFiftyMilliSecFlag() {
  FIFTY_MS_FLAG = true;
  }

void blink() {
  digitalWrite(LED_BUILTIN, ledState);
  ledState = !ledState;
  }
