/*
SK Ground Station Tester and Operational Source Code
Processor: ATMega328P, 5V 16 MHz
Other details are Redacted.

Source code by SoRa.
Modified by vtneil.

Use in conjunction with SK CanSat Tester and Operational Source Code.
This source code might not be consistent with the SK CanSat Tester and Operational Source Code.

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

#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define _BASE_FREQUENCY 420E6

#define GPSTX 4
#define GPSRX 2
#define REDLED 7
#define GREENLED 6
#define BLUELED 5
#define BATTREAD A0
#define BUZZER 3
#define LORAD0 8
#define LORARST 9
#define LORASS 10
#define LCDaddress 0x27
#define DEVICENAME "GS01"
#define _DEVICE_ID ((DEVICENAME[2] - '0') * 10 + (DEVICENAME[3] - '0'))
#define DEVICEFREQ (_BASE_FREQUENCY + (_DEVICE_ID - 1) * 9E6)

LiquidCrystal_I2C lcd(0x27, 20, 4);
bool runonlyonce = true;
unsigned long firstpkt = 0;
unsigned long amountrecieved = 0;
unsigned long amountloss = 0;
bool clearafterloss = true;
unsigned long runtime;

bool ifLCDexist(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

void Beep(int duration) {
  digitalWrite(BUZZER, HIGH);
  delay(duration);
  digitalWrite(BUZZER, LOW);
}
String checkmode = "NoLCD";
bool selfcheckLORA = false;
void setup() {
  pinMode(BUZZER, OUTPUT);
  pinMode(REDLED, OUTPUT);
  pinMode(GREENLED, OUTPUT);
  pinMode(BLUELED, OUTPUT);
  Wire.begin();
  Serial.begin(9600);
  if (ifLCDexist(LCDaddress)) {
    lcd.init();
    lcd.init();
    lcd.backlight();
    lcd.setCursor(1, 0);
    lcd.print(F("GndSta Self-check"));
    lcd.setCursor(0, 1);
    String modelname = "ID:" + String(DEVICENAME) + " FREQ:" + String(DEVICEFREQ / 1000000);
    lcd.print(modelname);
    lcd.setCursor(7, 2);
    lcd.print(F("By:Sora"));
    checkmode = "LCDavaliable";
    Beep(100);
    delay(1500);
  }

  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print(F("GndSta Self-check"));
  lcd.setCursor(1, 1);
  lcd.print(F(">LoRa initiation<"));
  lcd.setCursor(7, 3);
  lcd.print(F("By:Sora"));
  lcd.setCursor(2, 2);
  lcd.print(F("Status:Checking"));
  delay(500);
  Beep(100);
  LoRa.setPins(LORASS, LORARST, LORAD0);
  if (!LoRa.begin(420E6)) {
    selfcheckLORA = false;
    lcd.setCursor(2, 2);
    lcd.print(F(" Status:FAILED "));
    digitalWrite(REDLED, HIGH);
    digitalWrite(GREENLED, LOW);
  } else {
    selfcheckLORA = true;
    lcd.setCursor(2, 2);
    lcd.print(F(" Status:PASSED "));
    digitalWrite(REDLED, LOW);
    digitalWrite(GREENLED, HIGH);
  }
  delay(1000);
  digitalWrite(GREENLED, LOW);
  digitalWrite(REDLED, LOW);
  lcd.clear();
}

void loop() {
  String pkt = "";
  String codename = "";
  float lati, logi;
  float temp, humi, pres, balt;
  float battvol;
  uint32_t pktcounter = 0;
  int commapos[20];
  int positioncounter = 0;
  int arrayindex = 0;
  int packetSize = LoRa.parsePacket();
  int sumcheck = 0;
  bool pktvalidity = false;
  String gpsstatus = " N/A";
  String battstatus = "NORM";
  String bmestatus = " N/A";
  int duration = 50;

  if (packetSize) {

    // read packet
    while (LoRa.available()) {
      char rev = LoRa.read();
      if (rev == ',') commapos[arrayindex++] = positioncounter;
      positioncounter++;
      if (pkt != "\n") pkt += String(rev);
      else break;
    }
    //'CS:01,1391,0.000000,0.000000,30.25,1011.71,12.79,30.82,3.63,Endl,
    codename = pkt.substring(0, commapos[0]);
    pktcounter = pkt.substring(commapos[0] + 1, commapos[1]).toInt();
    lati = pkt.substring(commapos[1] + 1, commapos[2]).toFloat();
    logi = pkt.substring(commapos[2] + 1, commapos[3]).toFloat();
    temp = pkt.substring(commapos[3] + 1, commapos[4]).toFloat();
    pres = pkt.substring(commapos[4] + 1, commapos[5]).toFloat();
    balt = pkt.substring(commapos[5] + 1, commapos[6]).toFloat();
    humi = pkt.substring(commapos[6] + 1, commapos[7]).toFloat();
    battvol = pkt.substring(commapos[7] + 1, commapos[8]).toFloat();
    sumcheck = pkt.substring(commapos[8] + 1, commapos[9]).toInt();

    Serial.print(codename);
    Serial.print(",");
    Serial.print(pktcounter);
    Serial.print(",");
    Serial.print(lati, 6);
    Serial.print(",");
    Serial.print(logi, 6);
    Serial.print(",");
    Serial.print(temp);
    Serial.print(",");
    Serial.print(pres);
    Serial.print(",");
    Serial.print(balt);
    Serial.print(",");
    Serial.print(humi);
    Serial.print(",");
    Serial.print(battvol);
    Serial.print(",");
    Serial.print(pkt.length());
    Serial.print(",");
    Serial.println(sumcheck);

    if (pkt.length() != sumcheck) {
      runtime = millis();
      duration = 250;
      amountloss++;
      lcd.clear();
      lcd.setCursor(1, 1);
      lcd.print("Sumcheck Failed!!!");
      clearafterloss = true;
      digitalWrite(REDLED, HIGH);
      digitalWrite(BUZZER, HIGH);
    } else {
      if (clearafterloss == true) {
        clearafterloss = false;
        lcd.clear();
      }

      amountrecieved++;
      lcd.setCursor(1, 1);
      lcd.print("                  ");
      if (runonlyonce == true) {
        firstpkt = pktcounter;
        runonlyonce = false;
      }

      if ((lati >= 5.500 && lati <= 20.45) && (logi >= 97.36 && logi <= 105.62)) gpsstatus = "NORM";
      else if (lati == 0 && logi == 0) gpsstatus = " N/A";
      else gpsstatus = "ERR!";

      if (humi >= 2.0 && humi <= 99.0 && temp >= -20.0 && temp <= 70.0 && pres >= 900.0 && pres <= 1200.0) bmestatus = "NORM";
      else bmestatus = "ERR!";

      if (battvol >= 4.0 && battvol <= 4.23) battstatus = "FULL";
      else if (battvol >= 3.5 && battvol <= 4.0) battstatus = "NORM";
      else if (battvol >= 2.7 && battvol <= 3.5) battstatus = "LOW!";
      else battstatus = "ERR!";

      if (battstatus != "ERR!" && bmestatus != "ERR!" && gpsstatus != "ERR!") {
        digitalWrite(GREENLED, HIGH);
        digitalWrite(BUZZER, HIGH);
      } else {
        digitalWrite(BLUELED, HIGH);
        digitalWrite(BUZZER, HIGH);
      }
      duration = 50;
      runtime = millis();
      lcd.setCursor(0, 0);
      lcd.print("GPS:" + gpsstatus + "  BATT:" + battstatus);
      lcd.setCursor(0, 1);
      lcd.print("BME:" + bmestatus + "  PKT:" + pktcounter);
      lcd.setCursor(0, 2);
      lcd.print("Sig STR:" + String(LoRa.packetRssi()) + " dBm");
      lcd.setCursor(0, 3);
      lcd.print("Recv:" + String(amountrecieved) + "  Loss:" + String(amountloss));
    }
  }
  if (millis() - runtime > duration) {
    digitalWrite(REDLED, LOW);
    digitalWrite(GREENLED, LOW);
    digitalWrite(BLUELED, LOW);
    digitalWrite(BUZZER, LOW);
  }
}
