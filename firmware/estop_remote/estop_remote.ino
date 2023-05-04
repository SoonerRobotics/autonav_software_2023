
#include <SPI.h>
#include <RH_RF95.h>
#include <LiquidCrystal_PCF8574.h>
#include <Wire.h>
#include "estop_common.h"

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

#define ESTOP_BUTTON_PIN 10
#define MOBSTOP_BUTTON_PIN 11
#define MOBSTART_BUTTON_PIN 12
#define RF95_FREQ 915.0

#define LED_PIN 13
#define VBATPIN A7

RH_RF95 rf95(RFM95_CS, RFM95_INT);

LiquidCrystal_PCF8574 lcd(0x27); // set the LCD address to 0x27 for a 16 chars and 2 line display

// Parameters
float battery_min_voltage = 3.4f;
float battery_max_voltage = 3.75f;
int screen_refresh_period_ms = 250;
int heartbeat_period_ms = 250;
int handshake_period_ms = 500;
int message_resend_period_ms = 500;
int max_resends = 2;

// Storage variables
unsigned long last_display_update = 0;
unsigned long last_heartbeat = 0;
unsigned long last_handshake_attempt = 0;
unsigned long last_received_message = 0;
unsigned long last_sent_message = 0;
int current_resends = 0;

int led_status = 0;
float battery_percentage_smoothed = -1.0f;
uint8_t expecting_response_id = MSG_NONE_ID;
bool is_connected = false;

volatile int signal_to_send = NONE_SIGNAL;

RadioPacket outgoing_message;

// Errors
static const int NO_ERROR = 0;
static const int ERROR_WRONG_RESPONSE_ID = 1;
static const int ERROR_WRONG_SIGNAL_RECEIVED = 2;
volatile int error_state = NO_ERROR;

void estopButtonInterrupt()
{
  signal_to_send = ESTOP_SIGNAL;
}

void mobilityStopButtonInterrupt()
{
  if (signal_to_send == NONE_SIGNAL) {
    signal_to_send = MOB_STOP_SIGNAL;
  }
}

void mobilityStartButtonInterrupt()
{
  if (signal_to_send == NONE_SIGNAL) {
    signal_to_send = MOB_START_SIGNAL;
  }
}


void setup()
{
  Serial.begin(115200);

  // display init to blank screen
  lcd.begin(16, 2); // initialize the lcd
  lcd.setBacklight(255);
  lcd.home();
  lcd.clear();
  lcd.noBlink();
  lcd.noCursor();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, led_status);

  // reset rfm95
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  // init rfm95
  rf95.init();
  rf95.setFrequency(RF95_FREQ);
  rf95.setTxPower(23, false);

  // button interrupt setup
  pinMode(ESTOP_BUTTON_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ESTOP_BUTTON_PIN), estopButtonInterrupt, FALLING);

  pinMode(MOBSTOP_BUTTON_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(MOBSTOP_BUTTON_PIN), mobilityStopButtonInterrupt, FALLING);

  pinMode(MOBSTART_BUTTON_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(MOBSTART_BUTTON_PIN), mobilityStartButtonInterrupt, FALLING);

  strncpy(outgoing_message.password, GLOBAL_PASSWORD, sizeof(GLOBAL_PASSWORD));

}

void updateBatteryDisplay() {
  // Battery status
  float battery_volage = analogRead(VBATPIN);
  battery_volage *= 2;    // we divided by 2, so multiply back
  battery_volage *= 3.3;  // Multiply by 3.3V, our reference voltage
  battery_volage /= 1024; // convert to voltage

  float estimated_battery_percentage = (battery_volage - battery_min_voltage) / (battery_max_voltage - battery_min_voltage) * 100.0f; // lerp between bat min and bat max

  if (estimated_battery_percentage > 100) {
    estimated_battery_percentage = 100;
  }

  if (estimated_battery_percentage < 0) {
    estimated_battery_percentage = 0;
  }

  // Smooth out battery percentage by averaging it with the last value
  if (battery_percentage_smoothed < 0) {
    battery_percentage_smoothed = estimated_battery_percentage;
  } else {
    battery_percentage_smoothed = 0.7 * battery_percentage_smoothed + 0.3 * estimated_battery_percentage;
  }

  // Allow space for the "1" in "100"
  if (battery_percentage_smoothed < 99.5) {
    lcd.setCursor(13, 0);
  } else {
    lcd.setCursor(12, 0);
  }
 
  // Print leading 0
  if (battery_percentage_smoothed < 10) {
    lcd.print("0");
  }

  lcd.print(battery_percentage_smoothed, 0);
  lcd.print("%");
}

void updateDisplay() {
  lcd.home();
  lcd.clear();

  if (error_state != NO_ERROR) {
    lcd.print("ERROR!");
    lcd.setCursor(0, 1);
    lcd.print("Code: ");
    lcd.print((float)error_state, 0);

    updateBatteryDisplay();
    return;
  }

  if (is_connected && signal_to_send != NONE_SIGNAL) {
    lcd.print("Connected.   ");
    lcd.setCursor(0, 1);
    lcd.print("Sending: ");
    lcd.print((float)signal_to_send, 0);

    updateBatteryDisplay();
    return;
  }

  // Connection status
  if (is_connected) {
    lcd.print("Connected.   ");
    if (current_resends > 0) {
        lcd.setCursor(0, 1);
        lcd.print("Resends: ");
        lcd.print((float)current_resends, 0);
    }
  } else {
    lcd.print("Waiting...");
  }

  updateBatteryDisplay();
}

void reachedError(int error_type) {
    error_state = error_type;
    signal_to_send = ESTOP_SIGNAL;

    outgoing_message.id = MSG_SIGNAL_ID;
    outgoing_message.signal = signal_to_send;

    rf95.send((uint8_t*)&outgoing_message, sizeof(outgoing_message));
    rf95.waitPacketSent();}

void loop()
{

  if (rf95.available())
  {
    // Blink LED on receive
    led_status = !led_status;
    digitalWrite(LED_PIN, led_status);

    uint8_t buf[sizeof(RadioPacket)];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len))
    {
      // Convert packet to character array to determine message contents
      RadioPacket incoming_message = *(RadioPacket*)buf;

      // Check password
      if (strcmp(GLOBAL_PASSWORD, incoming_message.password) == 0) {

        last_received_message = millis();

        // Clear expecting response
        if (incoming_message.id == expecting_response_id) {
            expecting_response_id = MSG_NONE_ID;
            current_resends = 0;
        } else {
            // Error! Relay responded with wrong response!
            // EStop due to unknown state!
            reachedError(ERROR_WRONG_RESPONSE_ID);
        }

        // Handshake reply
        if (!is_connected && incoming_message.id == MSG_HANDSHAKE_REPLY_ID) {
            is_connected = true;
        }

        // Signal reply
        if (incoming_message.id == MSG_SIGNAL_REPLY_ID) {
            if (incoming_message.signal == signal_to_send) {
                signal_to_send = NONE_SIGNAL;
            } else {
                // Error! Relay responded with wrong signal!
                // EStop due to unknown state!
                reachedError(ERROR_WRONG_SIGNAL_RECEIVED);
            }
        }
      }
    }
  }

  // Update display regularly, except when we are expecting a response
  if ((expecting_response_id == MSG_NONE_ID || expecting_response_id == MSG_HANDSHAKE_REPLY_ID) && (millis() - last_display_update) > screen_refresh_period_ms) {
    last_display_update = millis();
    updateDisplay();
  }

//   // Mark us as disconnected if we haven't received an ACK in a while
//   if (is_connected && (millis() - last_received_message) > connection_timeout_ms) {
//     is_connected = false;
//   }

  // Resend messages
  if (is_connected && expecting_response_id != MSG_NONE_ID && (millis() - last_sent_message) > message_resend_period_ms) {
    // Disconnect after certain number of tries
    if (current_resends >= max_resends) {
        is_connected = false;
        expecting_response_id = MSG_NONE_ID;
        signal_to_send = NONE_SIGNAL;
    }

    current_resends += 1;

    // Update the display to reflect resend
    updateDisplay();

    rf95.send((uint8_t*)&outgoing_message, sizeof(outgoing_message));
    rf95.waitPacketSent();

    last_sent_message = millis();
  }

  // Send a signal if we have one waiting
  if (is_connected && expecting_response_id == MSG_NONE_ID && signal_to_send != NONE_SIGNAL) {
    outgoing_message.id = MSG_SIGNAL_ID;
    outgoing_message.signal = signal_to_send;

    rf95.send((uint8_t*)&outgoing_message, sizeof(outgoing_message));
    rf95.waitPacketSent();

    last_sent_message = millis();
    expecting_response_id = MSG_SIGNAL_REPLY_ID;
  }

  // Send a hearbeat periodically if we haven't received an ACK in a while
  if (is_connected && expecting_response_id == MSG_NONE_ID && (millis() - last_heartbeat) > heartbeat_period_ms) {
    last_heartbeat = millis();
    outgoing_message.id = MSG_INIT_HEARTBEAT_ID;

    rf95.send((uint8_t*)&outgoing_message, sizeof(outgoing_message));
    rf95.waitPacketSent();

    last_sent_message = millis();
    expecting_response_id = MSG_HEARTBEAT_REPLY_ID;
  }

  // Send a hearbeat periodically if we are not connected
  if (!is_connected && (millis() - last_handshake_attempt) > handshake_period_ms) {
    last_handshake_attempt = millis();
    outgoing_message.id = MSG_INIT_HANDSHAKE_ID;

    rf95.send((uint8_t*)&outgoing_message, sizeof(outgoing_message));
    rf95.waitPacketSent();

    last_sent_message = millis();
    expecting_response_id = MSG_HANDSHAKE_REPLY_ID;
  }
}