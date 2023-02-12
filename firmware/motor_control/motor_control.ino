#include "TickTwo.h"
#include <ACAN2515.h>
#include "motor.h"
#include "common.h" 

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

CANMessage frame;

// motor leftMotor(leftMotorPin, true);
// motor rightMotor(rightMotorPin, false);

void configureCan();
void printCanMsg();
void updateMsgData();
void setFiveMilliSecFlag();
void setTenMilliSecFlag();
void setFiftyMilliSecFlag();
void blink();

bool ledState;
int counterUS;

short leftSpeedSetpoint;
short rightSpeedSetpoint;


TickTwo timer1(setFiveMilliSecFlag, 5); //TODO: CHANGE 5 ms PLACE HOLDERS
TickTwo timer2(setTenMilliSecFlag, 10); //TODO: CHANGE 10 ms PLACE HOLDERS
TickTwo timer3(setFiftyMilliSecFlag, 1000); //TODO: CHANGE 50 ms PLACE HOLDERS

bool FIVE_MS_FLAG = false; //TODO : flag to keep track of 5 ms flag
bool TEN_MS_FLAG = false; //TODO : flag to keep track of 10 ms flag
bool FIFTY_MS_FLAG = false; //TODO: flag to keep track of 50 ms flag

static uint32_t gReceivedFrameCount = 0 ;


void setup() {
  pinMode(LED0, OUTPUT);
  digitalWrite (LED0, HIGH);
  pinMode(CANLED, OUTPUT);
  digitalWrite (CANLED, HIGH);

  Serial.begin(9600);
  while (!Serial) {
    delay (50) ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
  }
  configureCan();

  timer1.start();
  timer2.start();
  timer3.start();
  }

void loop() {
  updateTimers();
  digitalWrite (LED0, LOW);
  digitalWrite (CANLED, LOW);

  if(FIVE_MS_FLAG){ //TODO: CHANGE PLACE HOLDERS

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
    if (can.receive (frame)) {
      printCanMsg(frame);
    }
    FIFTY_MS_FLAG = false;
    
  }
}

void updateTimers() {
  timer1.update();
  timer2.update();
  timer3.update();
}

void configureCan(){
  SPI1.setSCK (MCP2515_SCK);
  SPI1.setTX (MCP2515_MOSI);
  SPI1.setRX (MCP2515_MISO);
  SPI1.setCS (MCP2515_CS); 
  SPI1.begin();


  Serial.println ("Configure ACAN2515") ;
  ACAN2515Settings settings (QUARTZ_FREQUENCY, 100UL * 1000UL) ; // CAN bit rate 100 kb/s
  const uint16_t errorCode = can.begin (settings,  canISR) ;
  if(errorCode == 0){
    Serial.print ("Actual bit rate: ") ;
    Serial.print (settings.actualBitRate ()) ;
  }


} 

void canISR(){
  can.isr();
  can.receive(frame);
  switch(frame.id){
    case 0:
    Serial.print("ESTOP \n");
    leftSpeedSetpoint = (short)frame.data[0]<<8 | (short)frame.data[1];
    rightSpeedSetpoint = (short)frame.data[2]<<8 | (short)frame.data[3];

    break;
    
    case 10:
    Serial.print("10 ID \n");
    leftSpeedSetpoint = (frame.data[0]<<8 & 0xFF00) | frame.data[1];
    rightSpeedSetpoint = (frame.data[2]<<8 & 0xFF00) | frame.data[3];
    break;

  }
  Serial.print("LEFT: ");
  Serial.println(leftSpeedSetpoint);
  Serial.print("RIGHT: ");
  Serial.println(rightSpeedSetpoint);
  //printCanMsg(frame);

}
void printCanMsg(CANMessage frame){
    Serial.print ("  id: ");Serial.println (frame.id,HEX);
    Serial.print ("  ext: ");Serial.println (frame.ext);
    Serial.print ("  rtr: ");Serial.println (frame.rtr);
    Serial.print ("  len: ");Serial.println (frame.len);
    Serial.print ("  data: ");
    for(int x=0;x<frame.len;x++) {
      Serial.print (frame.data[x],HEX); Serial.print(":");
    }
    Serial.println ("");
}

void updateMsgData(){

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
