/*
    LoRa board Wiring:
    ESP32       –  RFM95/96

    Gpio14/D14  – RST
    Gpio05/D05  – NSS (Chip Select)
    Gpio18/D18  – SCK
    Gpio23/D23  – MOSI
    Gpio19/D19  – MISO        
    Gpio02/D02  – DIO0 (INT)
    
    VCC         – 3.3V
    GND         – GND

    SSD1306 I2C OLED display:
    ESP32  - display

    GPIO21 - SDA
    GPIO22 - SCL

*/

#include <Arduino.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <Wire.h>
//#include <Adafruit_GFX.h>
#include <SSD1306Ascii.h>
#include <SSD1306AsciiWire.h>
//#include <Adafruit_SSD1306.h>

#define OLED_I2C_ADDR 0x3C /* OLED module I2C address */
#define OLED_SDA_GPIO 21
#define OLED_SCL_GPIO 22

//Set up OLED display
SSD1306AsciiWire oled_ssd1306_display;

#define RFM95_CS 5
#define RFM95_RST 14
#define RFM95_INT 2

#define RF95_FREQ 868.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);

void GPIO_init(void);
void RFM95_TX_init(void);
void Ssd1306_Oled_Init(void);

void setup()
{
  Serial.begin(115200);
  GPIO_init();
  Ssd1306_Oled_Init();
  RFM95_TX_init();
}

int16_t packetnum = 0; // packet counter
int16_t Last_RSSI = 0;

void loop()
{
  oled_ssd1306_display.clear();
  oled_ssd1306_display.setRow(0);
  oled_ssd1306_display.setCol(0);
  Serial.println("Sending to RF95_server");
  char radiopacket[32] = "RFM95 test # N      ";
  itoa(packetnum++, radiopacket + 13, 10);
  Serial.print("Sending ");
  Serial.println(radiopacket);
  radiopacket[31] = 0;

  //Serial.println("Sending...");
  delay(10);
  rf95.send((uint8_t *)radiopacket, 20);

  //Serial.println("Waiting for packet to complete...");
  delay(10);
  rf95.waitPacketSent();
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  Serial.println("Message sent, waiting for reply...");
  oled_ssd1306_display.println(radiopacket);
  delay(10);
  if (rf95.waitAvailableTimeout(1000))
  {
    if (rf95.recv(buf, &len))
    {
      Last_RSSI = rf95.lastRssi();
      Serial.print(">> Got reply: ");
      Serial.print((char *)buf);
      Serial.print("  RSSI: ");
      Serial.println(Last_RSSI, DEC);
      oled_ssd1306_display.print("ACK OK, RSSI: ");
      oled_ssd1306_display.println(Last_RSSI, DEC);
    }
    else
    {
      Serial.println("ACK Receive failed");
      oled_ssd1306_display.println("ACK failed");
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
  yield();
  delay(1000);
  yield();
  delay(1000);
}

void GPIO_init(void)
{
  delay(500);
  pinMode(RFM95_CS, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  Serial.println("GPIO init done");
  delay(500);
  yield();
  delay(500);
}

void RFM95_TX_init(void)
{
  oled_ssd1306_display.clear();
  oled_ssd1306_display.setRow(0);
  oled_ssd1306_display.setCol(0);
  Serial.println();
  Serial.println("Gateway Module starting…");
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  while (!rf95.init())
  {
    Serial.println("LoRa radio init failed");
    oled_ssd1306_display.println("LoRa radio init failed");
    while (1)
    {
      yield();
    }
  }
  Serial.println("LoRa radio init OK!");
  oled_ssd1306_display.println("LoRa radio init OK!");
  delay(100);
  if (!rf95.setFrequency(RF95_FREQ))
  {
    Serial.println("setFrequency failed");
    oled_ssd1306_display.println("setFrequency failed");
    while (1)
    {
      yield();
    }
  }
  Serial.print("Set Freq to: ");
  Serial.println(RF95_FREQ);
  rf95.setTxPower(23, false);
  Serial.println("RFM95 init done");
  oled_ssd1306_display.println("RFM95 init done");
  delay(100);
}

void Ssd1306_Oled_Init(void) {
  Wire.begin();
  oled_ssd1306_display.begin(&Adafruit128x32, OLED_I2C_ADDR);
  oled_ssd1306_display.clear();
  oled_ssd1306_display.setFont(X11fixed7x14);
  oled_ssd1306_display.setRow(0);
  oled_ssd1306_display.println(F("RFM95W"));
  oled_ssd1306_display.println(F("Transmitter"));
  Serial.println("OLED init done");
  yield();
  delay(500);
  yield();
  delay(500);
}
