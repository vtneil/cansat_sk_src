/*
SK CanSat Tester and Operational Source Code
Processor: ATMega328P, 5V 16 MHz
Other details are Redacted.

Source code by SoRa.
Modified by vtneil.

Use in conjunction with SK Ground Station Tester and Operational Source Code.

Modify at your own risks.
You can reproduce and modify the source under conditions:
1. You don't use for commercial purposes.
2. You are fully liable for any damages caused outside the usages of 
   SK Camp CanSat workshop.
3. The functionality under other usages are not guaranteed.

Contacts for License Information

phachara@spaceac.net (Phachara Phumiprathet)
vivatsathorn@outlook.co.th (Vivatsathorn Thitasirivit, vtneil)
*/

#include <Arduino.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

/*
Constants
*/
#define _SSL 1013.25f

/*
Device Attributes
*/
#define _DEVICE_ID 1
#define _BASE_FREQUENCY ((int32_t) 420E6)
#define _DEVICE_FREQUENCY (_BASE_FREQUENCY + (_DEVICE_ID - 1) * ((int32_t) 9E6))

/*
Pins Definitions
*/
#define _LCD_ADDRESS 0x27
#define _LCD_COLS 20
#define _LCD_ROWS 4
#define _GPS_TX 4
#define _GPS_RX 2
#define _LED_R 7
#define _LED_G 6
#define _LED_B 5
#define _BATT_ADC A0
#define _BUZZER 3
#define _LORA_D0 8
#define _LORA_RST 9
#define _LORA_SS 10

/*
Typedef
*/
typedef struct {
  double lat;
  double lon;
  double batt;
  float temp;
  float pres;
  float humi;
  float alt_bar;
} Message;

/*
Global declarations
*/
enum CheckMode {
  _NO_LCD = 0,
  _LCD_AVAILABLE
} check_mode;

struct {
  bool bme;
  bool gps;
  bool lora;
} self_check;

SoftwareSerial ss(_GPS_RX, _GPS_TX);
TinyGPSPlus gps;
LiquidCrystal_I2C lcd(_LCD_ADDRESS, _LCD_COLS, _LCD_ROWS);
Adafruit_BME280 bme;

uint32_t runtime;
uint32_t pktcounter;
String packet;
Message readings;

/*
Function declarations
*/
bool lcd_exists(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

void beep(uint32_t duration) {
  digitalWrite(_BUZZER, HIGH);
  delay(duration);
  digitalWrite(_BUZZER, LOW);
}

void craft_message(String &buff, Message &msg) {
  /*
  Build the string
  */
  char tmp[2];
  sprintf(tmp, "%02d", _DEVICE_ID);

  buff = "CS:" + String(tmp) + "," + String(pktcounter) + "," + String(msg.lat, 6) + "," + String(msg.lon, 6) + ",";
  buff += String(msg.temp) + "," + String(msg.pres) + "," + String(msg.alt_bar) + "," + String(msg.humi);
  buff += "," + String(msg.batt) + ",";

  /*
  Sum check
  */
  buff += String(buff.length() + 3) + ",";
}

/*
Setup
*/
void setup() {
  /*
  Set pin modes
  */
  pinMode(_BUZZER, OUTPUT);
  pinMode(_LED_R, OUTPUT);
  pinMode(_LED_G, OUTPUT);
  pinMode(_LED_B, OUTPUT);

  /*
  Initialize lines
  */
  Wire.begin();
  Serial.begin(9600);
  ss.begin(9600);

  /*
  Init LCD if exists
  */
  if (lcd_exists(_LCD_ADDRESS)) {
    lcd.init();
    lcd.init();
    lcd.backlight();
    lcd.setCursor(1, 0);
    lcd.print(F("CanSat Self-check"));
    lcd.setCursor(0, 1);
    char tmp[2];
    sprintf(tmp, "%02d", _DEVICE_ID);
    String modelname = "ID:CS" + String(tmp) + " FREQ:" + String(_DEVICE_FREQUENCY / 1000000);
    lcd.print(modelname);
    lcd.setCursor(7, 2);
    lcd.print(F("By:Sora"));
    check_mode = _LCD_AVAILABLE;
    beep(100);
    delay(1500);
  }

  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print(F("CanSat Self-check"));
  lcd.setCursor(1, 1);
  lcd.print(F(">LoRa initiation<"));
  lcd.setCursor(7, 3);
  lcd.print(F("By:Sora"));
  lcd.setCursor(2, 2);
  lcd.print(F("Status:Checking"));
  delay(500);
  beep(100);

  /*
  LoRa Self-Check
  */
  LoRa.setPins(_LORA_SS, _LORA_RST, _LORA_D0);
  LoRa.setTxPower(20);
  
  if (!LoRa.begin(_DEVICE_FREQUENCY)) {
    lcd.setCursor(2, 2);
    lcd.print(F(" Status:FAILED "));
    digitalWrite(_LED_R, HIGH);
    digitalWrite(_LED_G, LOW);
  } else {
    self_check.lora = true;
    lcd.setCursor(2, 2);
    lcd.print(F(" Status:PASSED "));
    digitalWrite(_LED_R, LOW);
    digitalWrite(_LED_G, HIGH);
  }

  delay(1000);
  digitalWrite(_LED_G, LOW);
  digitalWrite(_LED_R, LOW);

  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print(F("CanSat Self-check"));
  lcd.setCursor(2, 1);
  lcd.print(F(">GPS initiation<"));
  lcd.setCursor(7, 3);
  lcd.print(F("By:Sora"));
  lcd.setCursor(2, 2);
  lcd.print(F("Status:Checking"));
  delay(500);

  /*
  GPS Self-Check
  */
  unsigned long thistime = millis();
  while (true) {
    while (ss.available())
      gps.encode(ss.read());

    if (millis() - thistime > 5000 && gps.charsProcessed() < 10) {
      beep(100);
      lcd.setCursor(2, 2);
      lcd.print(F(" Status:FAILED "));
      digitalWrite(_LED_R, HIGH);
      digitalWrite(_LED_G, LOW);
      break;
    } else if (gps.charsProcessed() > 10) {
      self_check.gps = true;
      beep(100);
      lcd.setCursor(2, 2);
      lcd.print(F(" Status:PASSED "));
      digitalWrite(_LED_R, LOW);
      digitalWrite(_LED_G, HIGH);
      break;
    }
  }
  delay(1000);
  digitalWrite(_LED_G, LOW);
  digitalWrite(_LED_R, LOW);

  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print(F("CanSat Self-check"));
  lcd.setCursor(2, 1);
  lcd.print(F(">BME initiation<"));
  lcd.setCursor(7, 3);
  lcd.print(F("By:Sora"));
  lcd.setCursor(2, 2);
  lcd.print(F("Status:Checking"));
  delay(500);
  beep(100);

  /*
  BME Self-Check
  */
  if (!bme.begin(0x76)) {
    lcd.setCursor(2, 2);
    lcd.print(F(" Status:FAILED "));
    digitalWrite(_LED_R, HIGH);
    digitalWrite(_LED_G, LOW);
  } else {
    self_check.bme = true;
    lcd.setCursor(2, 2);
    lcd.print(F(" Status:PASSED "));
    digitalWrite(_LED_R, LOW);
    digitalWrite(_LED_G, HIGH);
  }
  delay(1000);
  digitalWrite(_LED_G, LOW);
  digitalWrite(_LED_R, LOW);
  lcd.clear();

  if (self_check.bme && self_check.gps && self_check.lora) {
    if (!check_mode) {
      digitalWrite(_LED_R, HIGH);
      digitalWrite(_LED_G, HIGH);
      digitalWrite(_LED_B, HIGH);
      delay(1000);
      beep(100);
    } else {
      lcd.setCursor(1, 1);
      lcd.print(F("Start Working Seq."));
      lcd.setCursor(7, 2);
      lcd.print(F("By:Sora"));
      delay(1000);
      beep(100);
      lcd.clear();
    }
  } else {
    beep(100);
    if (!check_mode) {
      digitalWrite(_LED_R, self_check.bme);
      digitalWrite(_LED_G, self_check.gps);
      digitalWrite(_LED_B, self_check.lora);
    } else {
      lcd.setCursor(2, 0);
      lcd.print(F("System failure!!"));
      lcd.setCursor(4, 1);
      String failcom = "";
      if (!self_check.bme) failcom += "BME,";
      if (!self_check.lora) failcom += "LORA,";
      if (!self_check.gps) failcom += "GPS,";
      lcd.print(failcom);
      lcd.setCursor(2, 2);
      lcd.print(F("Contact support"));
      lcd.setCursor(2, 3);
      lcd.print(F("Tel:063-231-7613"));
      delay(1000);
      thistime = millis();
    }

    /*
    Pause here if the GPS isn't working.
    */
    while (true) {
      while (ss.available())
        gps.encode(ss.read());
      if (millis() - thistime > 5000 && gps.charsProcessed() < 10) {
        thistime = millis();
      } else if (gps.charsProcessed() > 10) {
        self_check.gps = true;
        beep(100);
        break;
      }
    }

    lcd.clear();
    lcd.setCursor(1, 1);
    lcd.print(F("Start Working Seq."));
    lcd.setCursor(7, 2);
    lcd.print(F("By:Sora"));
    delay(1000);
    beep(100);
    lcd.clear();
  }

  /*
  Reset LEDs
  */
  digitalWrite(_LED_R, LOW);
  digitalWrite(_LED_G, LOW);
  digitalWrite(_LED_B, LOW);
}

/*
Loop
*/
void loop() {
  /*
    Read GPS Values
  */
  while (ss.available())
      gps.encode(ss.read());

  if (millis() - runtime > 550) {
    if (gps.location.isValid()) {
      readings.lat = gps.location.lat();
      readings.lon = gps.location.lng();
    } else {
      readings.lat = 0.0f;
      readings.lon = 0.0f;
    }
    /*
    Read BME Values
    */
    readings.temp = bme.readTemperature();        // deg C
    readings.pres = bme.readPressure() / 100.0f;  // hPa
    readings.alt_bar = bme.readAltitude(_SSL);    // m
    readings.humi = bme.readHumidity();           // RH%
    /*
    Read battery voltage from ADC
    */
    readings.batt = 0.00483 * ((double)analogRead(_BATT_ADC));

    craft_message(packet, readings);

    /*
    LCD Screen Debug
    */
    if (check_mode) {
      lcd.setCursor(0, 0);
      lcd.print("lat:" + String(readings.lat, 6) + "  Batt:");
      lcd.setCursor(0, 1);
      lcd.print("lng:" + String(readings.lon, 6) + " " + String(readings.batt, 2) + "V");
      lcd.setCursor(0, 2);
      lcd.print("Humi" + String(readings.humi, 1) + "% Temp" + String(readings.temp, 1) + "*C");
      lcd.setCursor(0, 3);
      lcd.print("P:" + String(readings.pres, 1) + "hPa Alt" + String(readings.alt_bar, 1) + "m");
    }

    /*
    Begin LoRa Transmission
    */
    Serial.print("Echo back: ");
    Serial.println(packet);

    LoRa.beginPacket();
    LoRa.print(packet);
    LoRa.endPacket(true);
    
    /*
    Activate LED and Buzzer Flash
    */
    digitalWrite(_LED_B, HIGH);
    digitalWrite(_BUZZER, HIGH);

    /*
    Reset time, increment counter.
    */
    ++pktcounter;
    runtime = millis();
  }

  /*
  Deactivate LED and Buzzer Flash
  */
  if (millis() - runtime > 50) {
    digitalWrite(_LED_B, LOW);
    digitalWrite(_BUZZER, LOW);
  }
}
