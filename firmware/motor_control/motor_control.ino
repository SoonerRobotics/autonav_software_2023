#include "TickTwo.h"
#include <ACAN2515.h>
#include "motor.h"

const int ledPin = LED_BUILTIN;
const int CANLED = 0;
const int LED0 = 1;

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

ACAN2515 can (MCP2515_CS, SPI1, MCP2515_INT) ;
static const uint32_t QUARTZ_FREQUENCY = 8UL * 1000UL * 1000UL ; // 8 MHz
const int CanMsgLen = 8;

// motor leftMotor(leftMotorPin, true);
// motor rightMotor(rightMotorPin, false);

void configureCan();
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
  pinMode(LED0, OUTPUT);
  digitalWrite (LED0, HIGH);
  pinMode(CANLED, OUTPUT);
  digitalWrite (CANLED, HIGH);

  Serial.begin(9600);
  delay(2000);
  timer1.start();
  timer2.start();
  timer3.start();
  }

void loop() {
  updateTimers();
  digitalWrite (LED0, LOW);
  digitalWrite (CANLED, LOW);


  if(FIVE_MS_FLAG){ //TODO: CHANGE PLACE HOLDERS
    digitalWrite (LED0, HIGH);
    delay(100);
    digitalWrite (LED0, LOW);

    // UpdateLeft();
    // UpdateRight();

    FIVE_MS_FLAG = false;
  }
  if(TEN_MS_FLAG){

    // ReadCAN();
    // UpdatePID();

    FIVE_MS_FLAG = false;
  }
  if(FIFTY_MS_FLAG){

    
    digitalWrite (CANLED, HIGH);
    delay(100);
    digitalWrite (LED0, LOW);
    FIFTY_MS_FLAG = false;
    
  }
}

void updateTimers() {
  timer1.update();
  timer2.update();
  timer3.update();
  }

void configureCan(){

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
