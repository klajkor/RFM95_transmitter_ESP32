/*
    Wiring:
    ESP32       –  RFM95/96

    Gpio14/D14  – RST
    Gpio05/D05  – NSS (Chip Select)
    Gpio18/D18  – SCK
    Gpio23/D23  – MOSI
    Gpio19/D19  – MISO        
    Gpio02/D02  – DIO0 (INT)
    
    VCC         – 3.3V
    GND         – GND

*/

#include <Arduino.h>
#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS 5
#define RFM95_RST 14
#define RFM95_INT 2

#define RF95_FREQ 868.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);

void GPIO_init(void);
void RFM95_TX_init(void);

void setup()
{
  Serial.begin(115200);
  GPIO_init();
  RFM95_TX_init();
}

int16_t packetnum = 0; // packet counter

void loop()
{
  Serial.println("Sending to RF95_server");
  
  char radiopacket[32] = "RFM95 test - #      ";
  itoa(packetnum++, radiopacket + 13, 10);
  Serial.print("Sending ");
  Serial.println(radiopacket);
  radiopacket[31] = 0;

  Serial.println("Sending...");
  delay(10);
  rf95.send((uint8_t *)radiopacket, 20);

  Serial.println("Waiting for packet to complete...");
  delay(10);
  rf95.waitPacketSent();
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  Serial.println("Waiting for reply...");
  delay(10);
  if (rf95.waitAvailableTimeout(1000))
  {
    if (rf95.recv(buf, &len))
    {
      Serial.print("Got reply: ");
      Serial.println((char *)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
  else
  {
    Serial.println("No reply from receiver");
  }
  yield();
  delay(1000);
  yield();
  delay(1000);
}

void GPIO_init(void)
{
  delay(1000);
  pinMode(RFM95_CS, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  Serial.println("GPIO init done");
  delay(1000);
  yield();
  delay(2000);
}

void RFM95_TX_init(void)
{
  Serial.println();

  Serial.println("Gateway Module starting…");

  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init())
  {

    Serial.println("LoRa radio init failed");

    while (1)
    {
      yield();
    }
  }

  Serial.println("LoRa radio init OK!");

  if (!rf95.setFrequency(RF95_FREQ))
  {

    Serial.println("setFrequency failed");

    while (1)
    {
      yield();
    }
  }

  Serial.print("Set Freq to: ");
  Serial.println(RF95_FREQ);
  rf95.setTxPower(23, false);
}
