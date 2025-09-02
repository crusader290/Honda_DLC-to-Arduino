#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>

// ---------- OLED Setup ----------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ---------- K-line Setup ----------
#define K_RX 3
#define K_TX 2
SoftwareSerial KLine(K_RX, K_TX); // RX, TX
#define MAX_FRAME 64
byte frameBuf[MAX_FRAME];
int frameLen = 0;

// ---------- Parsed Values ----------
int rpm = 0;
int coolantC = 0;
int fuelPct = 0;
int stft = 0;
int ltft = 0;
int map_kPa = 0;
float map_psi = 0;
int vss_kmh = 0;
float ecuVoltage = 0.0;
int iacvPct = 0;

// ---------- EEPROM Addresses ----------
#define ADDR_RPM 0
#define ADDR_FUEL 2
#define ADDR_COOL 6
#define ADDR_SPEED 10
#define ADDR_STFT 14
#define ADDR_LTFT 18

void saveEEPROM() {
  EEPROM.put(ADDR_RPM, rpm);
  EEPROM.put(ADDR_FUEL, fuelPct);
  EEPROM.put(ADDR_COOL, coolantC);
  EEPROM.put(ADDR_SPEED, vss_kmh);
  EEPROM.put(ADDR_STFT, stft);
  EEPROM.put(ADDR_LTFT, ltft);
}

void readEEPROM() {
  EEPROM.get(ADDR_RPM, rpm);
  EEPROM.get(ADDR_FUEL, fuelPct);
  EEPROM.get(ADDR_COOL, coolantC);
  EEPROM.get(ADDR_SPEED, vss_kmh);
  EEPROM.get(ADDR_STFT, stft);
  EEPROM.get(ADDR_LTFT, ltft);
}

// ---------- Setup ----------
void setup() {
  pinMode(K_TX, OUTPUT);
  digitalWrite(K_TX, HIGH);

  Serial.begin(115200);
  KLine.begin(10400);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)){
    Serial.println(F("OLED init failed"));
    for(;;);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  readEEPROM();
}

// ---------- Parse ECU Frame ----------
void parseFrame(byte* data, int len){
  for(int i=0;i<len-3;i++){
    // RPM
    if(data[i]==0x20 && data[i+1]==0x05 && data[i+2]==0x0C){
      rpm = ((int)data[i+3]*256 + data[i+4])/4;
    }
    // Fuel Level
    if(data[i]==0x20 && data[i+1]==0x05 && data[i+2]==0x0A){
      fuelPct = data[i+3]/2.55;
    }
    // Coolant Temp
    if(data[i]==0x20 && data[i+1]==0x05 && data[i+2]==0x06){
      coolantC = data[i+3]-40;
    }
    // Speed VSS
    if(data[i]==0x21 && data[i+1]==0x05){
      vss_kmh = data[i+2];
    }
    // STFT
    if(data[i]==0x20 && data[i+1]==0x05 && data[i+2]==0x20){
      stft = (data[i+3]-128)*100/128;
    }
    // LTFT
    if(data[i]==0x20 && data[i+1]==0x05 && data[i+2]==0x22){
      ltft = (data[i+3]-128)*100/128;
    }
    // MAP
    if(data[i]==0x20 && data[i+1]==0x05 && data[i+2]==0x12){
      map_kPa = data[i+3];
      map_psi = map_kPa * 0.145;
    }
    // ECU Voltage
    if(data[i]==0x20 && data[i+1]==0x05 && data[i+2]==0x17){
      ecuVoltage = data[i+3]/10.0;
    }
    // IACV
    if(data[i]==0x20 && data[i+1]==0x05 && data[i+2]==0x28){
      iacvPct = data[i+3];
    }
  }
}

// ---------- Loop ----------
void loop() {
  static unsigned long lastEEPROM = 0;

  // Read K-Line
  while(KLine.available()){
    byte b = KLine.read();
    if(frameLen < MAX_FRAME) frameBuf[frameLen++] = b;
  }

  if(frameLen>0){
    parseFrame(frameBuf, frameLen);

    // Display order: Speed, RPM, Fuel, Temp, ECU V, MAP, STFT, LTFT, IACV
    display.clearDisplay();
    display.setCursor(0,0);
    display.print("SPD: "); display.print(vss_kmh); display.println(" km/h");
    display.print("RPM: "); display.print(rpm); display.println(" rpm");
    display.print("Fuel: "); display.print(fuelPct); display.println("%");
    display.print("Temp: "); display.print(coolantC); display.println((char)247); display.println("C");
    display.print("ECU V: "); display.print(ecuVoltage,1); display.println("V");
    display.print("MAP: "); display.print(map_kPa); display.print(" kPa / "); display.print(map_psi,1); display.println(" psi");
    display.print("STFT: "); display.print(stft); display.println("%");
    display.print("LTFT: "); display.print(ltft); display.println("%");
    display.print("IACV: "); display.print(iacvPct); display.println("%");
    display.display();

    // Save to EEPROM every 5s
    if(millis() - lastEEPROM > 5000){
      saveEEPROM();
      lastEEPROM = millis();
    }

    frameLen = 0;
  }
}
