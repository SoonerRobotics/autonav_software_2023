#include <ACAN2515.h>
#include <ACAN2515Settings.h>
#include <ACAN2515_Buffer16.h>
#include <ACANBuffer.h>
#include <CANMessage.h>
#include <MCP2515ReceiveFilters.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <RHSoftwareSPI.h>

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

#define GLOBAL_PASSWORD "CRABCAKES"

RHSoftwareSPI spi;
RH_RF95 rf95(RADIO_CS, RADIO_INT, spi);
ACAN2515 can(CAN_CS, SPI, CAN_INT);


const uint32_t QUARTZ = 8 * 1000 * 1000;
bool ESTOP = false;
bool isConnected = false;
unsigned long lastAck = 0;
unsigned long lastHandshake = 0;

struct RadioPacket {
  unsigned char id;
  char password[sizeof(GLOBAL_PASSWORD)];
  char message[16];
};

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

  if (rf95.available()) {

    uint8_t buf[sizeof(RadioPacket)];
    uint8_t len = sizeof(buf);

      if (rf95.recv(buf, &len)) {

        RadioPacket msg = *(RadioPacket*)buf;

        if (strcmp(GLOBAL_PASSWORD, msg.password) == 0) {

            if (msg.id == 0) {

              if (msg.message[0] == 1) {

                digitalWrite(PICO_OUT, LOW);

                frame.id = 0x0;
                const bool ok = can.tryToSend(frame);

                if (ok) {
                  Serial.println("ESTOP");
                  ESTOP = true;
                  digitalWrite(LED_BUILTIN, HIGH);
                }

              }

              if (msg.message[0] == 2) {
                frame.id = 0x1;
                const bool ok = can.tryToSend(frame);
                if (ok) {
                  Serial.println("MOBILITY STOP");
                }
              }

              if (msg.message[0] == 3) {
                frame.id = 0x9;
                const bool ok = can.tryToSend(frame);
                if (ok) {
                  Serial.println("MOBILITY START");
                }
              }

              toSend.id = 9;
              lastAck = millis();
              rf95.send((uint8_t*)&toSend, sizeof(toSend));
              rf95.waitPacketSent();

            }

            if (!isConnected && msg.id == 2) {
              Serial.println("Handshake Ack");
              isConnected = true;
              lastAck = millis();
            }

            if (isConnected && msg.id == 3) {
              Serial.println("Heartbeat");
              toSend.id = 4;
              lastAck = millis();
              rf95.send((uint8_t*)&toSend, sizeof(toSend));
              rf95.waitPacketSent();
            }
        }
      }
  }

  if (!isConnected && !ESTOP && (millis() - lastHandshake) > 400) {

    lastHandshake = millis();
    toSend.id = 1;
    rf95.send((uint8_t*)&toSend, sizeof(toSend));
    rf95.waitPacketSent();

  }

  if (isConnected && (millis() - lastAck) > 5000) {

    digitalWrite(PICO_OUT, LOW);
    digitalWrite(LED_BUILTIN, HIGH);
    frame.id = 0x0;
    const bool ok = can.tryToSend(frame);
    if (ok) {
      Serial.println("ESTOP (5 seconds)");
    }
    isConnected = false;
    ESTOP = true;
    
  }
}
