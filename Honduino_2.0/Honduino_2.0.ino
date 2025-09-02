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

// ---------- ECU Init Sequences ----------
byte hondaInitBytes[][2] = {
  {0x68, 0x6A},
  {0x81, 0x13},
  {0x72, 0x05}
};
const int numInits = sizeof(hondaInitBytes)/sizeof(hondaInitBytes[0]);

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

void saveEEPROM(int rpm, float fuelPct, float coolantC){
  EEPROM.put(ADDR_RPM, rpm);
  EEPROM.put(ADDR_FUEL, fuelPct);
  EEPROM.put(ADDR_COOL, coolantC);
}

void readEEPROM(int &rpm, float &fuelPct, float &coolantC){
  EEPROM.get(ADDR_RPM, rpm);
  EEPROM.get(ADDR_FUEL, fuelPct);
  EEPROM.get(ADDR_COOL, coolantC);
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

// ---------- Parse RPM/Fuel/Coolant ----------
void parseECUValues(byte* data, int len, int &rpm, float &fuelPct, float &coolantC){
  rpm = -1; fuelPct = -1; coolantC = -1;
  for(int i=0;i<len-3;i++){
    if(data[i]==0x20 && data[i+1]==0x05) rpm = ((int)data[i+2]*256 + data[i+3])/4;
    if(data[i]==0x20 && data[i+1]==0x0A) fuelPct = data[i+2]/2.55;
    if(data[i]==0x20 && data[i+1]==0x06) coolantC = data[i+2]-40;
  }
}

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

  // ECU Init
  for(int i=0;i<numInits;i++){
    KLine.write(hondaInitBytes[i][0]);
    delay(25);
    KLine.write(hondaInitBytes[i][1]);
    delay(100);
    printTimestamp(Serial);
    Serial.print(F("Sent init: "));
    Serial.print(hondaInitBytes[i][0], HEX); Serial.print(" ");
    Serial.println(hondaInitBytes[i][1], HEX);
  }

  Serial.println(F("ECU Init Complete"));

  // EEPROM load
  int rpm = 0; float fuel = 0, cool = 0;
  readEEPROM(rpm, fuel, cool);
  Serial.print("Last RPM: "); Serial.println(rpm);
  Serial.print("Last Fuel: "); Serial.println(fuel);
  Serial.print("Last Coolant: "); Serial.println(cool);
}

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

  // If we have a complete frame (simplified: any data in buffer)
  if(frameLen>0){
    display.clearDisplay();
    display.setCursor(0,0);
    display.println(F("K-Line Live Data:"));
    
    int rpm; float fuelPct, coolantC;
    parseECUValues(frameBuf, frameLen, rpm, fuelPct, coolantC);
    parseECUFlags(frameBuf, frameLen);

    int y = 10;
    display.setCursor(0,y); y+=10;
    if(rpm>=0){ display.print(F("RPM: ")); display.println(rpm); }
    display.setCursor(0,y); y+=10;
    if(fuelPct>=0){ display.print(F("Fuel: ")); display.println(fuelPct); }
    display.setCursor(0,y); y+=10;
    if(coolantC>=0){ display.print(F("Coolant: ")); display.println(coolantC); }

    display.display();

    // USB logging
    if(rpm>=0){ printTimestamp(Serial); Serial.print(F("Engine RPM: ")); Serial.println(rpm);}
    if(fuelPct>=0){ printTimestamp(Serial); Serial.print(F("Fuel Level: ")); Serial.println(fuelPct);}
    if(coolantC>=0){ printTimestamp(Serial); Serial.print(F("Coolant Temp: ")); Serial.println(coolantC);}

    // Save EEPROM every 5s
    if(millis() - lastEEPROM > 5000){
      if(rpm>=0 && fuelPct>=0 && coolantC>=-40){
        saveEEPROM(rpm, fuelPct, coolantC);
        lastEEPROM = millis();
      }
    }

    // Reset frame buffer for next read
    frameLen = 0;
  }
}};

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

void saveEEPROM(int rpm, float fuelPct, float coolantC){
  EEPROM.put(ADDR_RPM, rpm);
  EEPROM.put(ADDR_FUEL, fuelPct);
  EEPROM.put(ADDR_COOL, coolantC);
}

void readEEPROM(int &rpm, float &fuelPct, float &coolantC){
  EEPROM.get(ADDR_RPM, rpm);
  EEPROM.get(ADDR_FUEL, fuelPct);
  EEPROM.get(ADDR_COOL, coolantC);
}

// Parse ECU switch/flag bits
void parseECUFlags() {
  for(int i=0;i<numFlags;i++){
    ECUFlag f = ecuFlags[i];
    for(int j=0;j<MAX_FRAME-2;j++){
      if(frameBuf[j]==((f.addr>>8)&0xFF) && frameBuf[j+1]==(f.addr&0xFF)){
        uint8_t val = frameBuf[j + 2 + f.bytePos];
        bool active = (val >> f.bitPos) & 1;
        // Serial
        Serial.print(f.name); Serial.print(": "); Serial.println(active?"ON":"OFF");
        // OLED
        display.print(f.name); display.print(":"); display.println(active?"ON":"OFF");
      }
    }
  }
}

void setup() {
  // K-line TX idle high
  pinMode(K_TX, OUTPUT);
  digitalWrite(K_TX, HIGH);

  // Serial interfaces
  Serial.begin(115200);
  KLine.begin(10400);

  // OLED init
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)){
    Serial.println(F("OLED init failed"));
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Initialize lastFrame
  for(int i=0;i<MAX_FRAME;i++) lastFrame[i]=0xFF;

  // ECU init sequences
  for(int i=0;i<numInits;i++){
    KLine.write(hondaInitBytes[i][0]);
    delay(25);
    KLine.write(hondaInitBytes[i][1]);
    delay(100);
    printTimestamp(Serial);
    Serial.print(F("Sent init: "));
    Serial.print(hondaInitBytes[i][0], HEX); Serial.print(" ");
    Serial.println(hondaInitBytes[i][1], HEX);
  }

  Serial.println(F("ECU Init Complete"));

  // Read last values from EEPROM
  int rpm = 0; float fuel = 0, cool = 0;
  readEEPROM(rpm, fuel, cool);
  Serial.print("Last RPM: "); Serial.println(rpm);
  Serial.print("Last Fuel: "); Serial.println(fuel);
  Serial.print("Last Coolant: "); Serial.println(cool);
}

void loop() {
  static int frameIndex = 0;

  // Forward USB -> ECU
  while(Serial.available()){
    byte b = Serial.read();
    KLine.write(b);
    printTimestamp(Serial);
    Serial.print(F("TX->ECU: 0x")); if(b<0x10) Serial.print("0"); Serial.println(b, HEX);
  }

  // Read ECU -> Nano
  while(KLine.available()){
    byte b = KLine.read();

    if(frameIndex < MAX_FRAME) frameBuf[frameIndex] = b;

    // Serial logging
    printTimestamp(Serial);
    Serial.print(F("RX<-ECU: 0x")); if(b<0x10) Serial.print("0"); Serial.println(b, HEX);

    // Detect byte changes
    if(frameIndex < MAX_FRAME && lastFrame[frameIndex] != b){
      printTimestamp(Serial);
      Serial.print(F("  **Changed at pos ")); Serial.print(frameIndex);
      Serial.print(F(": 0x")); Serial.println(b, HEX);
      lastFrame[frameIndex] = b;
    }

    frameIndex++;
    if(frameIndex >= MAX_FRAME) frameIndex = 0;

    // ---------- Parse RPM, Fuel, Coolant ----------
    int rpm = -1;
    float fuelPct = -1;
    float coolantC = -1;

    for(int i=0;i<frameIndex-3;i++){
      if(frameBuf[i]==0x20 && frameBuf[i+1]==0x05) rpm = ((int)frameBuf[i+2]*256 + frameBuf[i+3])/4;
      if(frameBuf[i]==0x20 && frameBuf[i+1]==0x0A) fuelPct = frameBuf[i+2]/2.55;
      if(frameBuf[i]==0x20 && frameBuf[i+1]==0x06) coolantC = frameBuf[i+2]-40;
    }

    // ---------- Display ----------
    display.clearDisplay();
    display.setCursor(0,0);
    display.println(F("K-Line Live Data:"));
    
    int y = 10;
    display.setCursor(0, y); y+=10;
    if(rpm>=0){ display.print(F("RPM: ")); display.println(rpm); }
    display.setCursor(0, y); y+=10;
    if(fuelPct>=0){ display.print(F("Fuel: ")); display.println(fuelPct); }
    display.setCursor(0, y); y+=10;
    if(coolantC>=0){ display.print(F("Coolant: ")); display.println(coolantC); }

    // Parse ECU switch/flag bits
    parseECUFlags();

    display.display();

    // ---------- USB logging ----------
    if(rpm>=0){
      printTimestamp(Serial); Serial.print(F("Engine RPM: ")); Serial.println(rpm);
    }
    if(fuelPct>=0){
      printTimestamp(Serial); Serial.print(F("Fuel Level: ")); Serial.println(fuelPct);
    }
    if(coolantC>=0){
      printTimestamp(Serial); Serial.print(F("Coolant Temp: ")); Serial.println(coolantC);
    }

    // ---------- Save last values to EEPROM every 5s ----------
    static unsigned long lastEEPROM = 0;
    if(millis() - lastEEPROM > 5000){
      if(rpm>=0 && fuelPct>=0 && coolantC>=-40){
        saveEEPROM(rpm, fuelPct, coolantC);
        lastEEPROM = millis();
      }
    }
  }
}
