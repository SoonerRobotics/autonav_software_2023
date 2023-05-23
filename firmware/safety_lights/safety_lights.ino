#include <SPI.h>
#include <ACAN2515.h>
#include <Adafruit_NeoPixel.h>
#include "ArduinoJson.h"

// Config

static const int BLINK_PERIOD_MS = 500;
static const int DEFAULT_BRIGHTNESS = 50;

// Definitions

static const int NUM_COLOR_LEDS = 61;
static const int COLOR_PIN = 22;
static const int E_STOP_PIN = 11;
static const int WHITE_PIN = 18;

static const byte MCP2515_SCK = 2; // SCK input of MCP2515
static const byte MCP2515_MOSI = 3; // SDI input of MCP2515
static const byte MCP2515_MISO = 0; // SDO output of MCP2517

static const byte MCP2515_CS = 1; // CS input of MCP2515
static const byte MCP2515_INT = 4; // INT output of MCP2515

Adafruit_NeoPixel strip(NUM_COLOR_LEDS, COLOR_PIN, NEO_GRB);

// Start as mobility start and not autonomuos
bool is_estopped = false;
bool is_mobility_stopped = false;
bool is_autonomous = false;
bool is_eco = false;
int current_brightness = DEFAULT_BRIGHTNESS;

// Json packet
StaticJsonDocument<256> json_in;

// Default to fading purple as a "connecting" state
int color_mode = 2;
int current_color = strip.Color(0,255,255);

typedef struct SafetyLightsMessage { 
    uint8_t autonomous: 1;
    uint8_t eco: 1;
    uint8_t mode: 6;
    uint8_t brightness;
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} SafetyLightsMessage;

SafetyLightsMessage last_CAN_message;

ACAN2515 can(MCP2515_CS, SPI, MCP2515_INT);

static const uint32_t QUARTZ_FREQUENCY = 8UL * 1000UL * 1000UL; // 8 MHZ

uint32_t colorSelector(int num){ //0-15 to color
  switch(num){
    case 0: //black
      return strip.Color(0,0,0);
    case 1: //red
      return strip.Color(0,255,0);
    case 2: //orange
      return strip.Color(127,255,0);
    case 3: //yellow
      return strip.Color(255,255,0);
    case 4: //lime
      return strip.Color(255,127,0);
    case 5: //green
      return strip.Color(255,0,0);
    case 6: //cyan-green
      return strip.Color(255,0,127);
    case 7: //cyan
      return strip.Color(255,0,255);
    case 8: //light blue
      return strip.Color(127,0,255);
    case 9: //blue
      return strip.Color(0,0,255);
    case 10: //violet
      return strip.Color(0,127,255);
    case 11: //purple
      return strip.Color(0,255,255);
    case 12: //pink
      return strip.Color(0,255,127);
    case 13: //white green
      return strip.Color(255,127,127);
    case 14: //white red
      return strip.Color(127,255,127);
    case 15: //white
      return strip.Color(225,255,255);
    default:
      return strip.Color(0,0,0);
  }
}

void modeSelector(){ // RGB mode selector
  switch(color_mode){
    case 0: //Solid
      colorSolid();
      break;
    case 1: //Flash
      colorFlash();
      break;
    case 2: //Fade
      colorFade();
      break;
    default:
      colorSolid();
      break;
   }
}

CANMessage frame;

void onCanRecieve() {
  can.isr();
}

void setup() {
  Serial.begin (115200);
  delay(2000);

  SPI.setSCK(MCP2515_SCK);
  SPI.setTX(MCP2515_MOSI);
  SPI.setRX(MCP2515_MISO);
  SPI.setCS(MCP2515_CS);
  SPI.begin();
  
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(WHITE_PIN, OUTPUT);
  pinMode(E_STOP_PIN, INPUT);
  digitalWrite(LED_BUILTIN, HIGH);
    
  strip.begin();
  strip.show();
  strip.setBrightness(current_brightness);

  analogWriteFreq(100);

  Serial.println("Configure ACAN2515");
  ACAN2515Settings settings(QUARTZ_FREQUENCY, 100UL * 1000UL);  // CAN bit rate 100 kb/s

  settings.mRequestedMode = ACAN2515Settings::NormalMode ; // Select Normal mode
  const uint16_t errorCode = can.begin(settings, onCanRecieve );
  if (errorCode == 0) {
    Serial.println("ACAN Configured???");
  }
  else{
    Serial.print("Error: ");
    Serial.print(errorCode);
  }

}

void loop() {

  // Serial.println("test");

  is_estopped = !digitalRead(E_STOP_PIN);

  if (Serial.available()) {
    // Read the JSON document from the "link" serial port
    DeserializationError err = deserializeJson(json_in, Serial);

    if (err == DeserializationError::Ok) 
    {
      int json_red = json_in["red"].as<int>();
      int json_green = json_in["green"].as<int>();
      int json_blue = json_in["blue"].as<int>();

      is_autonomous = json_in["autonomous"].as<bool>();
      is_eco = json_in["eco"].as<bool>();
      color_mode = json_in["mode"].as<int>();
      current_brightness = json_in["brightness"].as<int>();
      current_color = strip.Color(json_green, json_red, json_blue);
    } 
    else 
    {
      // Flush all bytes in the "link" serial port buffer
      while (Serial.available() > 0)
        Serial.read();
    }
  }

  if (can.available()) {
    can.receive(frame);
    // Serial.print("Received CAN ");
    // Serial.println(frame.id);

    switch (frame.id) {
      case 1: // Mobility stop
        is_mobility_stopped = true;
      case 9: // Mobility start
        is_mobility_stopped = false;
      case 13: // Safety Lights
        last_CAN_message = *(SafetyLightsMessage*)frame.data;

        is_autonomous = last_CAN_message.autonomous;
        is_eco = last_CAN_message.eco;
        color_mode = last_CAN_message.mode;
        current_brightness = last_CAN_message.brightness;
        current_color = strip.Color(last_CAN_message.green, last_CAN_message.red, last_CAN_message.blue);

        // Serial.print(last_CAN_message.mode);
        // Serial.print(", ");
        // Serial.print(last_CAN_message.red);
        // Serial.print(", ");
        // Serial.print(last_CAN_message.green);
        // Serial.print(", ");
        // Serial.print(last_CAN_message.blue);
        // Serial.print(", ");
        // Serial.println(last_CAN_message.brightness);
    }
  }

  whiteFlash();
  modeSelector();

  // delay(10); // Just so we aren't looping too fast
}

void whiteFlash() {
  if (!is_autonomous || is_estopped || is_mobility_stopped) {

    if (!is_eco) {
      digitalWrite(WHITE_PIN, LOW);
    } else {
      analogWrite(WHITE_PIN, 128);
    }

    return;
  }
  if (!is_eco) {
    digitalWrite(WHITE_PIN, (millis() / BLINK_PERIOD_MS) % 2);
  } else {
    analogWrite(WHITE_PIN,((millis() / BLINK_PERIOD_MS) % 2) * 128);
  }
}

void colorSolid() { // LED strip solid effect
  strip.setBrightness(current_brightness);
  strip.fill(current_color);
  strip.show();
}

void colorFlash(){ // LED strip flashing effect
  strip.setBrightness(current_brightness);
  if ((millis() / BLINK_PERIOD_MS) % 2 == 0){
    strip.fill(current_color);
    strip.show();
  }
  else{
    colorClear();
  }
}

void colorFade(){ // LED strip fading effect
  strip.setBrightness(abs(sin(millis() / (float)BLINK_PERIOD_MS)) * current_brightness);
  strip.fill(current_color);
  strip.show();
}

void colorClear(){ // Clear LED strip
  strip.fill(0);
  strip.show();
}
