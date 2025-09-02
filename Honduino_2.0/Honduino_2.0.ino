#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>

// ---------- OLED Setup ----------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ---------- K-line pins ----------
#define K_RX 3
#define K_TX 2
SoftwareSerial KLine(K_RX, K_TX); // RX, TX

#define MAX_FRAME 64
byte frameBuf[MAX_FRAME];
int frameLen = 0;

// ---------- ECU Flags ----------
struct ECUFlag {
  const char* name;
  uint16_t addr;
  uint8_t bytePos;
  uint8_t bitPos;
};

ECUFlag ecuFlags[] = {
  {"Starter", 0x2108, 0, 0},
  {"Aircon", 0x2108, 1, 0},
  {"Brake", 0x2108, 3, 0},
  {"ParkNeutral", 0x2108, 4, 0},
  {"VtecPres", 0x2108, 7, 0},
  {"SCS", 0x2109, 3, 0},
  {"VTS", 0x210A, 2, 0},
  {"MainRelay", 0x210B, 0, 0},
  {"AcClutch", 0x210B, 1, 0},
  {"O2Heater1", 0x210B, 2, 0},
  {"CEL", 0x210B, 5, 0},
  {"O2Heater2", 0x210B, 7, 0},
  {"AltC", 0x210C, 0, 0},
  {"FanC", 0x210C, 1, 0},
  {"IAB", 0x210C, 2, 0},
  {"Econo", 0x210C, 7, 0},
  {"Mount", 0x210D, 3, 0},
  {"ClosedLoop", 0x210F, 0, 0}
};
const int numFlags = sizeof(ecuFlags)/sizeof(ecuFlags[0]);

// ---------- Helper Functions ----------
void printTimestamp(Print &p){
  unsigned long ms = millis();
  char buf[16];
  sprintf(buf, "[%lu.%03lu] ", ms/1000, ms%1000);
  p.print(buf);
}

// EEPROM addresses
#define ADDR_RPM 0
#define ADDR_FUEL 2
#define ADDR_COOL 6
#define ADDR_SPEED 10

void saveEEPROM(int rpm, float fuelPct, float coolantC, int speed){
  EEPROM.put(ADDR_RPM, rpm);
  EEPROM.put(ADDR_FUEL, fuelPct);
  EEPROM.put(ADDR_COOL, coolantC);
  EEPROM.put(ADDR_SPEED, speed);
}

void readEEPROM(int &rpm, float &fuelPct, float &coolantC, int &speed){
  EEPROM.get(ADDR_RPM, rpm);
  EEPROM.get(ADDR_FUEL, fuelPct);
  EEPROM.get(ADDR_COOL, coolantC);
  EEPROM.get(ADDR_SPEED, speed);
}

// ---------- Parse ECU Flags ----------
void parseECUFlags(byte* data, int len) {
  for(int i=0;i<numFlags;i++){
    ECUFlag f = ecuFlags[i];
    for(int j=0;j<len;j++){
      if(data[j]==((f.addr>>8)&0xFF) && j+1<len && data[j+1]==(f.addr&0xFF)){
        if(j+2+f.bytePos<len){
          uint8_t val = data[j+2+f.bytePos];
          bool active = (val >> f.bitPos) & 1;
          Serial.print(f.name); Serial.print(": "); Serial.println(active?"ON":"OFF");
          display.print(f.name); display.print(":"); display.println(active?"ON":"OFF");
        }
      }
    }
  }
}

// ---------- Parse RPM/Fuel/Coolant/Speed ----------
void parseECUValues(byte* data, int len, int &rpm, float &fuelPct, float &coolantC, int &speed){
  rpm = -1; fuelPct = -1; coolantC = -1; speed = -1;
  for(int i=0;i<len-3;i++){
    if(data[i]==0x20 && data[i+1]==0x05) rpm = ((int)data[i+2]*256 + data[i+3])/4;
    if(data[i]==0x20 && data[i+1]==0x0A) fuelPct = data[i+2]/2.55;
    if(data[i]==0x20 && data[i+1]==0x06) coolantC = data[i+2]-40;
    if(data[i]==0x21 && data[i+1]==0x05) speed = data[i+2]; // typical 1-byte km/h
  }
}

// ---------- Honda Full Init Sequence ----------
byte hondaInitBytesFull[] = {
  0x68, 0x6A, 0xF5, 0xAF, 0xBF, 0xB3, 0xB2,
  0xC1, 0xDB, 0xB3, 0xE9
};
const int numInitBytes = sizeof(hondaInitBytesFull)/sizeof(hondaInitBytesFull[0]);

// ---------- Setup ----------
void setup() {
  pinMode(K_TX, OUTPUT);
  digitalWrite(K_TX, HIGH);

  Serial.begin(115200);
  KLine.begin(10400);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)){
    Serial.println(F("OLED init failed"));
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // ECU Init with full sequence
  Serial.println(F("Starting ECU Init Sequence..."));
  for(int i=0;i<numInitBytes;i++){
    KLine.write(hondaInitBytesFull[i]);
    delay(50); // small delay between bytes
    printTimestamp(Serial);
    Serial.print(F("Sent init byte: 0x")); 
    if(hondaInitBytesFull[i]<0x10) Serial.print("0");
    Serial.println(hondaInitBytesFull[i], HEX);
  }
  Serial.println(F("ECU Init Complete"));

  // EEPROM load
  int rpm = 0; float fuel = 0, cool = 0; int speed = 0;
  readEEPROM(rpm, fuel, cool, speed);
  Serial.print("Last Speed: "); Serial.print(speed); Serial.println(" km/h");
  Serial.print("Last RPM: "); Serial.println(rpm);
  Serial.print("Last Fuel: "); Serial.print(fuel); Serial.println("%");
  Serial.print("Last Coolant: "); Serial.print(cool); Serial.println("°C");
}

// ---------- Loop ----------
void loop() {
  static unsigned long lastEEPROM = 0;

  // Forward USB -> ECU
  while(Serial.available()){
    byte b = Serial.read();
    KLine.write(b);
    printTimestamp(Serial);
    Serial.print(F("TX->ECU: 0x")); if(b<0x10) Serial.print("0"); Serial.println(b, HEX);
  }

  // Read ECU -> Nano into frameBuf
  while(KLine.available()){
    byte b = KLine.read();
    if(frameLen < MAX_FRAME) frameBuf[frameLen++] = b;
  }

  // If we have a complete frame
  if(frameLen>0){
    display.clearDisplay();
    display.setCursor(0,0);
    display.println(F("K-Line Live Data:"));
    
    int rpm; float fuelPct, coolantC; int speed;
    parseECUValues(frameBuf, frameLen, rpm, fuelPct, coolantC, speed);
    parseECUFlags(frameBuf, frameLen);

    int y = 10;
    display.setCursor(0,y); y+=10;
    if(speed>=0){ display.print(F("Speed: ")); display.print(speed); display.println(F(" km/h")); }
    display.setCursor(0,y); y+=10;
    if(rpm>=0){ display.print(F("RPM: ")); display.println(rpm); }
    display.setCursor(0,y); y+=10;
    if(fuelPct>=0){ display.print(F("Fuel: ")); display.print(fuelPct); display.println(F("%")); }
    display.setCursor(0,y); y+=10;
    if(coolantC>=0){ display.print(F("Coolant: ")); display.print(coolantC); display.println(F("°C")); }

    display.display();

    // USB logging in same order
    if(speed>=0){ printTimestamp(Serial); Serial.print(F("Vehicle Speed: ")); Serial.print(speed); Serial.println(F(" km/h")); }
    if(rpm>=0){ printTimestamp(Serial); Serial.print(F("Engine RPM: ")); Serial.println(rpm); }
    if(fuelPct>=0){ printTimestamp(Serial); Serial.print(F("Fuel Level: ")); Serial.print(fuelPct); Serial.println(F("%")); }
    if(coolantC>=0){ printTimestamp(Serial); Serial.print(F("Coolant Temp: ")); Serial.print(coolantC); Serial.println(F("°C")); }

    // Save EEPROM every 5s
    if(millis() - lastEEPROM > 5000){
      if(rpm>=0 && fuelPct>=0 && coolantC>=-40 && speed>=0){
        saveEEPROM(rpm, fuelPct, coolantC, speed);
        lastEEPROM = millis();
      }
    }

    // Reset frame buffer for next read
    frameLen = 0;
  }
}
