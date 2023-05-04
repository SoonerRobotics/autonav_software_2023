#include <ACAN2515.h>
#include <ACAN2515Settings.h>
#include <ACAN2515_Buffer16.h>
#include <ACANBuffer.h>
#include <CANMessage.h>
#include <MCP2515ReceiveFilters.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <RHSoftwareSPI.h>

#include "estop_common.h"

#define RADIO_RESET 7
#define RADIO_MISO 8
#define RADIO_CS 9
#define RADIO_SCK 10
#define RADIO_MOSI 11
#define RADIO_INT 12

#define CAN_MISO 16
#define CAN_CS 17
#define CAN_SCK 18
#define CAN_MOSI 19
#define CAN_INT 20

#define PICO_OUT 26

RHSoftwareSPI spi;
RH_RF95 rf95(RADIO_CS, RADIO_INT, spi);
ACAN2515 can(CAN_CS, SPI, CAN_INT);

// Parameters
static const int receive_timeout_ms = 2000;


const uint32_t QUARTZ = 8 * 1000 * 1000;
bool ESTOP = false;
bool isConnected = false;
unsigned long lastAck = 0;
unsigned long lastHandshake = 0;

unsigned long lastReceivedMessageMillis = 0;

RadioPacket toSend;
CANMessage frame;

void setup() {

  pinMode(RADIO_RESET, INPUT);
  
  Serial.begin(115200);

  pinMode(PICO_OUT, OUTPUT);
  digitalWrite(PICO_OUT, HIGH);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  delay(2000);

  pinMode(RADIO_RESET, OUTPUT);
  digitalWrite(RADIO_RESET, LOW);
  delay(10);
  digitalWrite(RADIO_RESET, HIGH);
  delay(10);

  SPI.setSCK(CAN_SCK);
  SPI.setTX(CAN_MOSI);
  SPI.setRX(CAN_MISO);
  SPI.setCS(CAN_CS);

  // SPI1.setSCK(RADIO_SCK);
  // SPI1.setTX(RADIO_MOSI);
  // SPI1.setRX(RADIO_MISO);
  // SPI1.setCS(RADIO_CS);
  spi.setPins(RADIO_MISO, RADIO_MOSI, RADIO_SCK);

  SPI.begin();

  ACAN2515Settings settings(QUARTZ, 100 * 1000);
  const uint16_t error = can.begin(settings, [] { can.isr () ; });
  if (error == 0) {
    Serial.print("Bit Rate: ");
    Serial.print(settings.actualBitRate());
  }
  else {
    Serial.print(error);
  }

  if (!rf95.init()) {
    Serial.println("Radio Initialization Failure");
  }
  rf95.setFrequency(915.0);
  rf95.setTxPower(23, false);

  lastHandshake = millis();
  strncpy(toSend.password, GLOBAL_PASSWORD, sizeof(GLOBAL_PASSWORD));

}

void loop() {

  if (ESTOP) {
    return;
  }

  if (rf95.available()) {

    uint8_t buf[sizeof(RadioPacket)];
    uint8_t len = sizeof(buf);

      if (rf95.recv(buf, &len)) {

        RadioPacket msg = *(RadioPacket*)buf;

        if (strcmp(GLOBAL_PASSWORD, msg.password) == 0) {

            lastReceivedMessageMillis = millis();

            // Signal
            if (msg.id == MSG_SIGNAL_ID) {

              if (msg.signal == ESTOP_SIGNAL) {

                digitalWrite(PICO_OUT, LOW);
                ESTOP = true;
                digitalWrite(LED_BUILTIN, HIGH);

                frame.id = 0x0;
                can.tryToSend(frame);
              }

              if (msg.signal == MOB_STOP_SIGNAL) {
                frame.id = 0x1;
                can.tryToSend(frame);
              }

              if (msg.signal == MOB_START_SIGNAL) {
                frame.id = 0x9;
                can.tryToSend(frame);
              }

              toSend.id = MSG_SIGNAL_REPLY_ID;
              toSend.signal = msg.signal;
              rf95.send((uint8_t*)&toSend, sizeof(toSend));
              rf95.waitPacketSent();

            }

            // Handshake
            if (!isConnected && msg.id == MSG_INIT_HANDSHAKE_ID) {
              isConnected = true;

              toSend.id = MSG_HANDSHAKE_REPLY_ID;
              rf95.send((uint8_t*)&toSend, sizeof(toSend));
              rf95.waitPacketSent();
            }

            // Heartbeat
            if (isConnected && msg.id == MSG_INIT_HEARTBEAT_ID) {
              toSend.id = MSG_HEARTBEAT_REPLY_ID;
              rf95.send((uint8_t*)&toSend, sizeof(toSend));
              rf95.waitPacketSent();
            }
        }
      }
  }

  // EStop if we haven't received anything in a while
  if (isConnected && (millis() - lastReceivedMessageMillis) > receive_timeout_ms) {

    digitalWrite(PICO_OUT, LOW);
    ESTOP = true;
    digitalWrite(LED_BUILTIN, HIGH);

    frame.id = 0x0;
    can.tryToSend(frame);

    isConnected = false;
  }
}
