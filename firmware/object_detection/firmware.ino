#ifndef ARDUINO_ARCH_RP2040
#error "Select a Raspberry Pi Pico board"
#endif

//——————————————————————————————————————————————————————————————————————————————

#include <ACAN2515.h>
#include <NewPing.h>
static const byte MCP2515_SCK  = 18 ; // SCK input of MCP2515
static const byte MCP2515_MOSI = 19 ; // SDI input of MCP2515
static const byte MCP2515_MISO = 16 ; // SDO output of MCP2515
static const byte MCP2515_CS  = 17 ;  // CS input of MCP2515 (adapt to your design)
static const byte MCP2515_INT = 26 ;  // INT output of MCP2515 (adapt to your design)
#define TRIGGER_PINJ1 1
#define ECHO_PINJ1 0
#define TRIGGER_PINJ4 5
#define ECHO_PINJ4 4
#define TRIGGER_PINJ5 7
#define ECHO_PINJ5 6
#define MAX_DISTANCE 40
#define LEDJ1 2
#define LEDJ4 11
#define LEDJ5 27
int distanceJ1[4];
int distanceJ4[4];
int distanceJ5[4];
int distanceJ1Median;
int distanceJ4Median;
int distanceJ5Median;

// NewPing setup of pins and maximum distance
NewPing sonarJ1(TRIGGER_PINJ1, ECHO_PINJ1, MAX_DISTANCE); 
NewPing sonarJ4(TRIGGER_PINJ4, ECHO_PINJ4, MAX_DISTANCE); 
NewPing sonarJ5(TRIGGER_PINJ5, ECHO_PINJ5, MAX_DISTANCE); 
ACAN2515 can (MCP2515_CS, SPI, MCP2515_INT) ;
static const uint32_t QUARTZ_FREQUENCY = 8UL * 1000UL * 1000UL ; // 8 MHz
void setup() {
  pinMode(LEDJ1, OUTPUT);
  pinMode(LEDJ4, OUTPUT);
  pinMode(LEDJ5, OUTPUT);
  Serial.begin(9600);
  SPI.setSCK(MCP2515_SCK);
  SPI.setTX(MCP2515_MOSI);
  SPI.setRX(MCP2515_MISO);
  SPI.setCS(MCP2515_CS);
  //--- Begin SPI
  SPI.begin () ;
  //--- Configure ACAN2515
  Serial.println ("Configure ACAN2515") ;
  ACAN2515Settings settings (QUARTZ_FREQUENCY, 100UL * 1000UL) ; // CAN bit rate 100 kb/s
  const uint16_t errorCode = can.begin (settings, [] { can.isr () ; }) ;
  if (errorCode == 0) {
    Serial.print ("Bit Rate prescaler: ") ;
    Serial.println (settings.mBitRatePrescaler) ;
    Serial.print ("Propagation Segment: ") ;
    Serial.println (settings.mPropagationSegment) ;
    Serial.print ("Phase segment 1: ") ;
    Serial.println (settings.mPhaseSegment1) ;
    Serial.print ("Phase segment 2: ") ;
    Serial.println (settings.mPhaseSegment2) ;
    Serial.print ("SJW: ") ;
    Serial.println (settings.mSJW) ;
    Serial.print ("Triple Sampling: ") ;
    Serial.println (settings.mTripleSampling ? "yes" : "no") ;
    Serial.print ("Actual bit rate: ") ;
    Serial.print (settings.actualBitRate ()) ;
    Serial.println (" bit/s") ;
    Serial.print ("Exact bit rate ? ") ;
    Serial.println (settings.exactBitRate () ? "yes" : "no") ;
    Serial.print ("Sample point: ") ;
    Serial.print (settings.samplePointFromBitStart ()) ;
    Serial.println ("%") ;
  } else {
    Serial.print ("Configuration error 0x") ;
    Serial.println (errorCode, HEX) ;
  }

}
static uint32_t gBlinkLedDate =0;
void loop() {
  digitalWrite(LEDJ1, HIGH);
  digitalWrite(LEDJ4, HIGH);
  digitalWrite(LEDJ5, HIGH);
  CANMessage frame;
  frame.id = 0x14;
  frame.data[0] = 0;
  frame.data[1] = distanceJ1Median;
  frame.data[2] = distanceJ4Median;
  frame.data[3] = distanceJ5Median;
  frame.len = 4;
  if (gBlinkLedDate < millis ()){
    gBlinkLedDate += 10;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN));
  }
  distanceJ1[0] = sonarJ1.ping_cm();
  distanceJ1[1] = sonarJ1.ping_cm();
  distanceJ1[2] = sonarJ1.ping_cm();
  distanceJ1[3] = sonarJ1.ping_cm();
  distanceJ1[4] = sonarJ1.ping_cm();
  if (distanceJ1[0] == 0) {
    distanceJ1[0]= 255;
  }
    if (distanceJ1[1] == 0) {
    distanceJ1[1]= 255;
  }
    if (distanceJ1[2] == 0) {
    distanceJ1[2]= 255;
  }
    if (distanceJ1[3] == 0) {
    distanceJ1[3]= 255;
  }
    if (distanceJ1[4] == 0) {
    distanceJ1[4]= 255;
  }
  distanceJ1Median = median(distanceJ1, 5);
  distanceJ4[0] = sonarJ4.ping_cm();
  distanceJ4[1] = sonarJ4.ping_cm();
  distanceJ4[2] = sonarJ4.ping_cm();
  distanceJ4[3] = sonarJ4.ping_cm();
  distanceJ4[4] = sonarJ4.ping_cm();
    if (distanceJ4[0] == 0) {
    distanceJ4[0]= 255;
  }
    if (distanceJ4[1] == 0) {
    distanceJ4[1]= 255;
  }
    if (distanceJ4[2] == 0) {
    distanceJ4[2]= 255;
  }
    if (distanceJ4[3] == 0) {
    distanceJ4[3]= 255;
  }
    if (distanceJ4[4] == 0) {
    distanceJ4[4]= 255;
  }
  distanceJ4Median = median(distanceJ4, 5);
  distanceJ5[0] = sonarJ5.ping_cm();
  distanceJ5[1] = sonarJ5.ping_cm();
  distanceJ5[2] = sonarJ5.ping_cm();
  distanceJ5[3] = sonarJ5.ping_cm();
  distanceJ5[4] = sonarJ5.ping_cm();
    if (distanceJ5[0] == 0) {
    distanceJ5[0]= 255;
  }
    if (distanceJ5[1] == 0) {
    distanceJ5[1]= 255;
  }
    if (distanceJ5[2] == 0) {
    distanceJ5[2]= 255;
  }
    if (distanceJ5[3] == 0) {
    distanceJ5[3]= 255;
  }
    if (distanceJ5[4] == 0) {
    distanceJ5[4]= 255;
  }
  distanceJ5Median = median(distanceJ5, 5);
  const bool ok = can.tryToSend(frame);
  if (ok) {
    Serial.println("Sent");
  }
  //Serial.println(distanceJ1[0]);
  //Serial.println(distanceJ1[1]);
  //Serial.println(distanceJ1[2]);
  //Serial.println(distanceJ1[3]);
  //Serial.println(distanceJ1[4]);
  Serial.println(distanceJ1Median);
  Serial.println(distanceJ4Median);
  Serial.println(distanceJ5Median);
}
const int WINDOW_SIZE = 5;
int data[WINDOW_SIZE];

// Define a variable to keep track of the current position in the array
int pos = 0;
int median(int* values, int n) {
  // Sort the values in ascending order
  for (int i = 0; i < n-1; i++) {
    for (int j = i+1; j < n; j++) {
      if (values[i] > values[j]) {
        int tmp = values[i];
        values[i] = values[j];
        values[j] = tmp;
      }
    }
  }

  // Return the median value
  if (n % 2 == 0) {
    return (values[n/2-1] + values[n/2]) / 2;
  } else {
    return values[n/2];
  }
}

// Function to update the filter with a new data point
int updateFilter(int newData) {
  // Add the new data point to the array
  data[pos] = newData;

  // Increment the position variable and wrap around if necessary
  pos = (pos + 1) % WINDOW_SIZE;

  // Calculate the median of the data points
  return median(data, WINDOW_SIZE);
}
