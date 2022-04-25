/*
*******************************************************************************
  Copyright (c) 2021 by Marko Langer
  Programm Name: VanLevel
  Version: 1.04
  Date：29.03.2022
  Author: Marko Langer
   Update 25.04.2022
  - Changed to AHRS-data for better deg. stability.
  
   Update 10.04.2022
  - LowPassFilter for IMU input
  - BugFix on Webinterface
  - Level 9 is adapted as maximum of level cross
  - Initial EEPROM setting write.

   Update 03.04.2022
  - Preset Radiobuttons in WebInterface
  - Charging Status with USB+in
*******************************************************************************
*/

#include <WiFi.h>
#include <EEPROM.h>
#include "M5StickCPlus.h"

TFT_eSprite tftSprite = TFT_eSprite(&M5.Lcd);

int addr = 0; //EEPROM Start number of an ADDRESS.
#define SIZE 32  //define the size of EEPROM(Byte).

const unsigned int BCKGRDCOL  = M5.Lcd.color565( 51,  51,  51);
const unsigned int TEXTCOL    = M5.Lcd.color565(187, 187, 187);
const unsigned int GREENLINE  = M5.Lcd.color565(  0, 255,   0);
const unsigned int GREENDOT   = M5.Lcd.color565(  0, 191,  15);
const unsigned int BLUELINE   = M5.Lcd.color565(  0,   0, 255);
const unsigned int YELLOWLINE = M5.Lcd.color565(255, 255,   0);
const unsigned int PINKLINE   = M5.Lcd.color565(255,   0, 255);
short menu = 1;
short BallX = 170;
short BallY = 67;
short hoeheY = 1;
short hoeheX = 1;
short Radstand = 380;
short Spurweite = 180;
short hoeheVL = 0;
short hoeheVR = 0;
short hoeheHL = 0;
short hoeheHR = 0;
short limit_aktuell = 0;
short limit_1 = 4;
short limit_2 = 8;
short limit_3 = 12;
short limit_4 = 16;
short limit_5 = 19;
short limitVL = 0;
short limitVR = 0;
short limitHL = 0;
short limitHR = 0;
short Timer = 10;
short Display = 3;
short Standby = 15;
short i = 0;
short I = 0;
short Sound = 1;
short Calibrate = 1; // 1 = no Calibration / 2 = Calibrate
float OffsetPitch = 0;
float OffsetRoll = 0;
float OffsetFactor = 1;
float pitch = 0.0F;
float roll  = 0.0F;
float pitch1 = 0.0F;
float roll1  = 0.0F;
float yaw   = 0.0F;
float pi = 3.1415926F;
float lowPassFilter = 0.025; // depends to delay in loop and max frequency
float lowPassGyroX = 0;
float lowPassGyroY = 0;
float lowPassGyroZ = 0;
int x1;
int x2;
bool x3;
float x4;
unsigned long previousMillis = 0;
unsigned long currentMillis = millis();
String checked1 = "";
String checked2 = "";
String checked3 = "";
String checked4 = "";
String checked5 = "";
String checked6 = "";
String checked7 = "";


IPAddress apIP(9, 9, 9, 9);
WiFiClient myclient;
WiFiServer server(80);
String ConfigValue[12]; //readings from WIFI connections


void setup() {
  M5.begin(); //Init M5StickC Plus.
  M5.Lcd.fillScreen(WHITE);
  delay(100);
  M5.Lcd.fillScreen(BCKGRDCOL);
  tftSprite.createSprite(240, 135);
  tftSprite.fillRect(0, 0, 240, 135, BCKGRDCOL);
  tftSprite.pushSprite(0, 0);
  tftSprite.setTextSize(2);
  tftSprite.setTextColor(TEXTCOL, BCKGRDCOL);
  delay(10);
  // Init Gyro
  M5.Imu.Init();                                                                 //Init IMU.
  // Init EEPROM
  if (!EEPROM.begin(20)) M5.Lcd.printf("\nFailed to initialise EEPROM!");       //Request storage of 10 sizes(success return 1).
  else {
    Sound = EEPROM.read(0);
    Display = EEPROM.read(1);
    Standby = EEPROM.read(2);
    Radstand = EEPROM.read(3) * 10;
    Spurweite = EEPROM.read(4) * 10;
    limit_1 = EEPROM.read(5);
    limit_2 = EEPROM.read(6);
    limit_3 = EEPROM.read(7);
    limit_4 = EEPROM.read(8);
    limit_5 = EEPROM.read(9);
    Calibrate = EEPROM.read(10);
    if (Sound == 255 || Display == 255 || Standby == 255 || Radstand == 255 || Spurweite == 255 || limit_1 == 255 || limit_2 == 255 || limit_3 == 255 || limit_4 == 255 || limit_5 == 255 || Calibrate == 255) {
      EEPROM.write(0, 1);  // Sound
      EEPROM.write(1, 3);  // Display
      EEPROM.write(2, 15); // Standby
      EEPROM.write(3, 38); // Radstand
      EEPROM.write(4, 18); // Spurweite
      EEPROM.write(5, 4);  // limit_1
      EEPROM.write(6, 8);  // limit_2
      EEPROM.write(7, 16); // limit_3
      EEPROM.write(8, 19); // limit_4
      EEPROM.write(9, 2);  // limit_5
      EEPROM.write(10, 0); // Calibrate
      EEPROM.write(11, 0);
      EEPROM.write(12, 0);
      EEPROM.write(13, 0);
      EEPROM.write(14, 0);
      EEPROM.write(15, 0);
      EEPROM.write(16, 0);
      EEPROM.write(17, 0);
      EEPROM.commit();
    }
    // Sound ////////////////////
    if (Sound == 1) {
      checked1 = "checked";
      checked2 = "";
    }
    else if (Sound == 0) {
      checked1 = "";
      checked2 = "checked";
    }
    // Display ////////////////////
    if (Display == 3) {
      checked3 = "checked";
      checked4 = "";
    }
    else if (Display == 1) {
      checked3 = "";
      checked4 = "checked";
    }
    // Standby ////////////////////
    if (Standby == 15)
    {
      checked5 = "checked" ;
      checked6 = "";
      checked7 = "";
    }
    else if (Standby == 10)
    {
      checked5 = "" ;
      checked6 = "checked";
      checked7 = "";
    }
    else if (Standby == 5)
    {
      checked5 = "" ;
      checked6 = "";
      checked7 = "checked";
    }
    // OffsetPitch ////////////////////
    x4 = ((EEPROM.read(12) * 100) + EEPROM.read(13));
    x4 = x4 / 100;
    if (EEPROM.read(11) == 0) x4 = x4 * -1;
    OffsetPitch = x4;
    //OffsetRoll
    x4 = ((EEPROM.read(15) * 100) + EEPROM.read(16));
    x4 = x4 / 100;
    if (EEPROM.read(14) == 0) x4 = x4 * -1;
    OffsetRoll = x4;
    if (Calibrate == 2) menu = 7;
    EEPROM.write(10, 1);
    EEPROM.commit();
  }
  // Init GPIO ////////////////////
  pinMode(M5_BUTTON_HOME, INPUT_PULLUP);
  pinMode(10, OUTPUT);
  // Init Display ////////////////////
  M5.Axp.ScreenBreath(10);
  digitalWrite(10, 1);                // turn the LED off
  M5.Beep.tone(4000);
  delay(100);
  M5.Beep.mute();
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(TEXTCOL, BCKGRDCOL);
  M5.Lcd.setRotation(Display);
  // BootScreen ////////////////////
  tftSprite.setCursor(25, 50);
  tftSprite.setTextSize(4);
  tftSprite.print("VanLevel");
  tftSprite.fillRect(22, 85, 193, 5, GREENDOT);
  tftSprite.setTextSize(2);
  tftSprite.pushSprite(0, 0);
  delay(1500);
}

void loop() {
  tftSprite.fillRect(0, 0, 240, 135, BCKGRDCOL);
  M5.update();
  BtnCheck();
  if (((millis() / 1000) / 60) >= Standby) menu = 4; //Go to standby when time equals to Standby

  switch (menu) {
    case 1:
      ball();
      menu1();
      break;
    case 2:
      calculation();
      if (Sound == 1) // jump to next menue when Sound is off
      {
        sound();
      }
      break;
    case 3:
      calculation();
      delay(1);
      if (Sound == 0) menu = 4;
      break;
    case 4:
      menu = 1;
      M5.Axp.PowerOff();
      break;
    case 5:
      Config();
      break;
    case 6:
      menu = 1;
      menuChange();
      ESP.restart();
      break;
    case 7:
      CAL();
      break;
  }
  tftSprite.pushSprite(0, 0);
}


// Berechnung Ball ////////////////////
void ball() {
  M5.Imu.getAhrsData(&pitch, &roll, &yaw);
  lowPassGyroX = (lowPassFilter * pitch * OffsetFactor) + ((1 - lowPassFilter) * lowPassGyroX);
  lowPassGyroY = (lowPassFilter * roll * OffsetFactor) + ((1 - lowPassFilter) * lowPassGyroY);
  pitch = lowPassGyroX;
  roll = lowPassGyroY;
  hoeheX = round(Radstand * tan((pitch + OffsetPitch) * pi / 180));
  hoeheY = round(Spurweite * tan((roll + OffsetRoll) * pi / 180));

  if (Display == 1) {
    hoeheX = hoeheX * -1;
    hoeheY = hoeheY * -1;
    BallX = map(hoeheX, limit_5 * -1, limit_5, 135, 0);
    BallY = map(hoeheY, limit_5 * -1, limit_5, 110, 230);
  }
  else {
    BallX = map(hoeheX, limit_5 * -1, limit_5, 135, 0);
    BallY = map(hoeheY, limit_5 * -1, limit_5, 110, 230);
  }
  if (BallX <= 0)   BallX =   7;
  if (BallX >= 135) BallX = 125;
  if (BallY <= 110) BallY = 110;
  if (BallY >= 230) BallY = 230;
}


// Fadenkreuz ////////////////////
void menu1() {
  //Background
  double batteryLevel = 100.0 * (((M5.Axp.GetVapsData() * 1.4 / 1000) - 3.0) / (4.078 - 3.0));  // calculate percentage until brown out @3,7V in %
  if (batteryLevel > 100.0) batteryLevel = 100;                                 // if result is bigger than 100% = status charging.
  if (batteryLevel >= 50.0 && M5.Axp.GetVBusVoltage() <= 4.7 ) {
    tftSprite.fillRect(30, 90, 30, 12, GREENDOT);
    tftSprite.fillRect(60, 93, 3, 6, GREENDOT);
  }
  if (batteryLevel < 50.0 && M5.Axp.GetVBusVoltage() <= 4.7 ) {
    tftSprite.fillRect(30, 90, 30, 12, YELLOWLINE);
    tftSprite.fillRect(60, 93, 3, 6, YELLOWLINE);
  }
  if (batteryLevel < 30.0 && M5.Axp.GetVBusVoltage() <= 4.7 )   // if result is under 20% = send out Warning.
  {
    tftSprite.fillRect(30, 90, 30, 12, RED);
    tftSprite.fillRect(60, 93, 3, 6, RED);
    tftSprite.setCursor(10, 30);
    tftSprite.setTextColor(RED, BCKGRDCOL);
    tftSprite.printf("Batt.\n LOW!");
    tftSprite.setTextColor(GREEN, BCKGRDCOL);
    if (batteryLevel < 20.0 && M5.Axp.GetVBusVoltage() <= 4.7) M5.Axp.PowerOff();
  }
  if (M5.Axp.GetVBusVoltage() >= 4.7 && M5.Axp.GetBatVoltage() >= 4.1) {
    tftSprite.fillRect(30, 90, 30, 12, GREENDOT);
    tftSprite.fillRect(60, 93, 3, 6, GREENDOT);
  }
  else if (M5.Axp.GetVBusVoltage() >= 4.7) {
    tftSprite.fillRect(60, 93, 3, 6, GREENDOT);
    tftSprite.fillRect(30, 90, 30, 12, TEXTCOL);
  }
  tftSprite.setCursor(10, 10);
  tftSprite.printf("VanLevel");
  tftSprite.drawRect(120, 17, 100, 100, PINKLINE);
  tftSprite.drawRect(130, 27, 80, 80, RED);
  tftSprite.drawRect(140, 37, 60, 60, YELLOWLINE);
  tftSprite.drawRect(150, 47, 40, 40, BLUELINE);
  tftSprite.drawRect(160, 57, 20, 20, GREENLINE);
  //Cross ////////////////////
  tftSprite.drawFastHLine(110, 67, 120, TEXTCOL);
  tftSprite.drawFastVLine(170, 7, 120, TEXTCOL);
  //Ball ////////////////////
  tftSprite.drawCircle(BallY, BallX, 6, WHITE);
  tftSprite.fillCircle(BallY, BallX, 5, GREENDOT);
}

// Calibration Mode ////////////////////
void CAL() {        //4 Runs / 300 times each. TO get a stable base.
  tftSprite.setCursor(0, 10);
  tftSprite.printf(" Calibrate\n Kalibriere ");
  tftSprite.pushSprite(0, 0);
  do {
    M5.Imu.getAhrsData(&pitch, &roll, &yaw);
    I++;
  } while (I < 300);
  tftSprite.setCursor(0, 10);
  tftSprite.printf(" Calibrate\n Kalibriere\n\n         3");
  tftSprite.pushSprite(0, 0);
  I = 0;
  do {
    M5.Imu.getAhrsData(&pitch, &roll, &yaw);
    I++;
  } while (I < 600);
  delay(500);
  tftSprite.setCursor(0, 10);
  tftSprite.printf(" Calibrate\n Kalibriere\n\n         2");
  tftSprite.pushSprite(0, 0);
  delay(1000);
  M5.Imu.getAhrsData(&pitch, &roll, &yaw);
  // last run ////////////////////
  tftSprite.setCursor(0, 10);
  tftSprite.printf(" Calibrate\n Kalibriere\n\n         1");
  tftSprite.pushSprite(0, 0);
  delay(1000);
  M5.Imu.getAhrsData(&pitch, &roll, &yaw);

  tftSprite.setCursor(0, 10);
  tftSprite.printf(" Calibrate\n Kalibriere\n\n         0");
  tftSprite.pushSprite(0, 0);
  M5.Beep.tone(8000);
  delay(1000);
  M5.Beep.mute();
  // Save to EEPROM ////////////////////
  // PITCH ////////////////////
  if (pitch <= 0) {
    x3 = 1;
    pitch = pitch * -1;
  }
  else x3 = 0;
  x1 = pitch;
  x2 = round((pitch - x1) * 100);
  x4 = ((x1 * 100) + x2);
  x4 = x4 / 100;
  EEPROM.write(11, x3);
  EEPROM.write(12, x1);
  EEPROM.write(13, x2);
  EEPROM.commit();
  delay(10);
  OffsetPitch = x4;
  // ROLL ////////////////////
  if (roll <= 0) {
    x3 = 1;
    roll = roll * -1;
  }
  else x3 = 0;
  x1 = roll;
  x2 = round((roll - x1) * 100);
  x4 = ((x1 * 100) + x2);
  x4 = x4 / 100;
  EEPROM.write(14, x3);
  EEPROM.write(15, x1);
  EEPROM.write(16, x2);
  EEPROM.commit();
  delay(10);
  OffsetRoll = x4;
  menu = 1;
  menuChange();
  ESP.restart();
}




void BtnCheck() {
  if (M5.BtnA.wasReleased()) {  //If the button A is pressed.
    menu = menu + 1;
    menuChange();
  }
  if (M5.BtnB.wasReleased()) {  //If the button B is pressed.
    menu = 5;
    menuChange();
  }

}


void menuChange() {
  tftSprite.fillScreen(BCKGRDCOL);
  M5.Beep.mute(); // Beep off
  digitalWrite(10, 1); // LED off
  i = 0;
}

void Config() {
  tftSprite.setCursor(10, 10);
  tftSprite.print("Setup");
  tftSprite.setCursor(0, 40);
  tftSprite.printf(" WIFI-Name:\n Setup_VanLevel\n\n Address:\n http://9.9.9.9/");


  // Init WiFi
  Webserver_Start();
  String GETParameter = Webserver_GetRequestGETParameter();
  if (GETParameter.length() > 0)        // we got a request, client connection stays open
  {
    if (GETParameter.length() > 1)      // request contains some GET parameter
    {
      int countValues = DecodeGETParameterAndSetConfigValues(GETParameter);     // decode the GET parameter and set ConfigValues

      menu = menu + 1;
    }
    String HTMLPageWithConfigForm = EncodeFormHTMLFromConfigValues(checked1, checked2, checked3, checked4, checked5, checked6, checked7); //"ESP32 Webserver Demo", 3);   // build a new webpage with form and new ConfigValues entered in textboxes
    Webserver_SendHTMLPage(HTMLPageWithConfigForm);    // send out the webpage to client = webbrowser and close client connection
  }
}


void calculation() {
  tftSprite.setCursor(10, 10);
  tftSprite.print("Level & Ramp Step");
  M5.Imu.getAhrsData(&pitch, &roll, &yaw);

  lowPassGyroX = (lowPassFilter * pitch * OffsetFactor) + ((1 - lowPassFilter) * lowPassGyroX);
  lowPassGyroY = (lowPassFilter * roll * OffsetFactor) + ((1 - lowPassFilter) * lowPassGyroY);
  pitch = lowPassGyroX;
  roll = lowPassGyroY ;
  hoeheX = round(Radstand * tan((pitch + OffsetPitch) * pi / 180));
  hoeheY = round(Spurweite * tan((roll + OffsetRoll) * pi / 180))*-1;

  if (Display == 1) {
    hoeheX = hoeheX * -1;
    hoeheY = hoeheY * -1;
  }
  if (hoeheX > 0) hoeheVL = hoeheX;
  else hoeheVL = 0;
  if (hoeheY > 0) hoeheVL = hoeheVL + hoeheY;
  else hoeheVL = hoeheVL + 0;
  if (hoeheVL < limit_1) limitVL = 0;
  else if (hoeheVL < limit_2) limitVL = 1;
  else if (hoeheVL < limit_3) limitVL = 2;
  else if (hoeheVL < limit_4) limitVL = 3;
  else if (hoeheVL < limit_5) limitVL = 4;
  else if (hoeheVL >= limit_5) limitVL = 9;

  if (hoeheX > 0) hoeheVR = hoeheX;
  else hoeheVR = 0;
  if (hoeheY < 0) hoeheVR = hoeheVR + (hoeheY * -1);
  else hoeheVR = hoeheVR + 0;
  if (hoeheVR < limit_1) limitVR = 0;
  else if (hoeheVR < limit_2) limitVR = 1;
  else if (hoeheVR < limit_3) limitVR = 2;
  else if (hoeheVR < limit_4) limitVR = 3;
  else if (hoeheVR < limit_5) limitVR = 4;
  else if (hoeheVR >= limit_5) limitVR = 9;

  if (hoeheX > 0) hoeheHL = 0;
  else hoeheHL = hoeheX;
  if (hoeheHL < 0) hoeheHL = hoeheHL * -1;
  if (hoeheY > 0) hoeheHL = hoeheHL + hoeheY;
  else hoeheHL = hoeheHL + 0;
  if (hoeheHL < limit_1) limitHL = 0;
  else if (hoeheHL < limit_2) limitHL = 1;
  else if (hoeheHL < limit_3) limitHL = 2;
  else if (hoeheHL < limit_4) limitHL = 3;
  else if (hoeheHL < limit_5) limitHL = 4;
  else if (hoeheHL >= limit_5) limitHL = 9;

  if (hoeheX < 0) hoeheHR = hoeheX * -1;
  else hoeheHR = 0;
  if (hoeheY > 0) hoeheHR = hoeheHR + 0;
  else hoeheHR = hoeheHR + (hoeheY * -1);
  if (hoeheHR < limit_1) limitHR = 0;
  else if (hoeheHR < limit_2) limitHR = 1;
  else if (hoeheHR < limit_3) limitHR = 2;
  else if (hoeheHR < limit_4) limitHR = 3;
  else if (hoeheHR < limit_5) limitHR = 4;
  else if (hoeheHR >= limit_5) limitHR = 9;

  if (hoeheVL >= 999 || hoeheHL >= 999 || hoeheVR >= 999 || hoeheHR >= 999) // warning if level is to high LED on
  {
    if (hoeheVL >= 999) hoeheVL = 999;
    if (hoeheHL >= 999) hoeheHL = 999;
    if (hoeheHR >= 999) hoeheHR = 999;
    if (hoeheVR >= 999) hoeheVR = 999;
  }

  if (limitVL >= 8 || limitHL >= 8 || limitVR >= 8 || limitHR >= 8) digitalWrite(10, 0); // warning if level is to high LED on
  else if (limitVL + limitHL + limitHR + limitVR <= 3)                                             // if sum off all are smaler than 3 than led on and sound on
  {
    digitalWrite(10, 0);
    i++;
  }
  else
  {
    digitalWrite(10, 1);
    i = 0;
  }
  tftSprite.setCursor(0, 40);
  tftSprite.printf(" %03dcm        %03dcm\n   %01d            %01d\n                   \n   %01d            %01d\n %03dcm        %03dcm", hoeheHR, hoeheHL, limitHR, limitHL, limitVR, limitVL, hoeheVR, hoeheVL);

  tftSprite.drawCircle(118, 60, 17, TEXTCOL);
  tftSprite.drawRect(100, 60, 37, 55, TEXTCOL);
  tftSprite.fillRect(101, 61, 35, 53, BCKGRDCOL);
  tftSprite.drawTriangle(113, 70, 123, 70, 118, 65, GREENDOT);
  tftSprite.drawLine(118, 70, 118, 85, GREENDOT);
  tftSprite.fillRect(100, 55, 5 , 10, WHITE);
  tftSprite.fillRect(132, 55, 5 , 10, WHITE);
  tftSprite.fillRect(100, 90, 5 , 10, WHITE);
  tftSprite.fillRect(132, 90, 5 , 10, WHITE);
  tftSprite.drawLine(131, 94, 105, 94, TEXTCOL);

}


//Beep
void sound() {
  if (i >= Timer) M5.Beep.tone(4000);
  else M5.Beep.mute();
  delay(10);
}


void Webserver_Start()
{
  WiFi.mode(WIFI_AP);
  WiFi.softAP("Setup_VanLevel");
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  for (int count = 0; count < 8; count++) ConfigValue[count] = "";
  server.begin();     // Start TCP/IP-Server on ESP32
}

String Webserver_GetRequestGETParameter()
{
  String GETParameter = "";
  myclient = server.available();   // listen for incoming clients
  if (myclient) {                            // if you get a client,
    String currentLine = "";                 // make a String to hold incoming data from the client
    while (myclient.connected()) {           // loop while the client's connected
      if (myclient.available()) {            // if there's bytes to read from the client,
        char c = myclient.read();            // read a byte, then
        if (c == '\n') {                     // if the byte is a newline character
          if (currentLine.length() == 0) {
            if (GETParameter == "") {
              GETParameter = "-";
            };    // if no "GET /?" was found so far in the request bytes, return "-"
            break;
          }
          else currentLine = "";
        } else if (c != '\r')currentLine += c;      // add it to the end of the currentLine
        if (c == '\r' && currentLine.startsWith("GET /?"))          GETParameter = currentLine.substring(currentLine.indexOf('?') + 1, currentLine.indexOf(' ', 6));    // extract everything behind the ? and before a space
      }
    }
  }
  return GETParameter;
}


void Webserver_SendHTMLPage(String HTMLPage)
{
  String httpResponse = "";
  httpResponse += "HTTP/1.1 200 OK\r\n";// begin with HTTP response header
  httpResponse += "Content-type:text/html\r\n\r\n";
  httpResponse += HTMLPage;  // then the HTML page
  httpResponse += "\r\n";  // The HTTP response ends with a blank line:
  myclient.println(httpResponse);  // send it out to TCP/IP client = webbrowser
  myclient.stop();  // close the connection
};



int DecodeGETParameterAndSetConfigValues(String GETParameter)
{
  int posFirstCharToSearch = 1;
  int count = 0;

  // while a "&" is in the expression, after a start position to search
  while (GETParameter.indexOf('&', posFirstCharToSearch) > -1)
  {
    int posOfSeparatorChar = GETParameter.indexOf('&', posFirstCharToSearch);  // position of & after start position
    int posOfValueChar = GETParameter.indexOf('=', posFirstCharToSearch);      // position of = after start position
    ConfigValue[count] = GETParameter.substring(posOfValueChar + 1, posOfSeparatorChar);  // extract everything between = and & and enter it in the ConfigValue
    posFirstCharToSearch = posOfSeparatorChar + 1;  // shift the start position to search after the &-char
    count++;
  }
  // no more & chars found
  int posOfValueChar = GETParameter.indexOf('=', posFirstCharToSearch);       // search for =
  ConfigValue[count] = GETParameter.substring(posOfValueChar + 1, GETParameter.length());  // extract everything between = and end of string
  count++;
  Sound = ConfigValue[0].toInt();
  Display = ConfigValue[1].toInt();
  Standby = ConfigValue[2].toInt();
  Radstand = ConfigValue[3].toInt();
  Spurweite = ConfigValue[4].toInt();
  limit_1 = ConfigValue[5].toInt();
  limit_2 = ConfigValue[6].toInt();
  limit_3 = ConfigValue[7].toInt();
  limit_4 = ConfigValue[8].toInt();
  limit_5 = ConfigValue[9].toInt();
  Calibrate = ConfigValue[10].toInt();

  EEPROM.write(0, Sound);
  EEPROM.write(1, Display);
  EEPROM.write(2, Standby);
  EEPROM.write(3, Radstand / 10);
  EEPROM.write(4, Spurweite / 10);
  EEPROM.write(5, limit_1);
  EEPROM.write(6, limit_2);
  EEPROM.write(7, limit_3);
  EEPROM.write(8, limit_4);
  EEPROM.write(9, limit_5);
  EEPROM.write(10, Calibrate);
  EEPROM.commit();
  return count;  // number of values found in GET parameter
}



String EncodeFormHTMLFromConfigValues(String checked1, String checked2, String checked3, String checked4, String checked5, String checked6, String checked7) //String TitleOfForm, int CountOfConfigValues)
{
  String HTMLPage = " <!DOCTYPE html> <html><meta name = 'viewport' content = 'width=device-width, initial-scale=1.0, user-scalable=no'><body>";                   // " + TitleOfForm + "</h2><form><table>";
  HTMLPage += "<form><table style=' border-collapse: collapse; font-family:Helvetica; color: DimGray; border-style: hidden; margin-left: auto; margin-right: auto; float: center;' border='0'>";
  HTMLPage += "<tbody><tr><td style='text-align: center; vertical-align: middle; width: 150px; font-size: xx-large;' colspan='3'><b>";
  HTMLPage += "VanLevel - Setup</b></td>";
  HTMLPage += "</tr><tr><td style='text-align: center; font-size: x-small; color: lightgrey; vertical-align: middle; width: 150px;' colspan='3'><br><i>";
  HTMLPage += "Configurate your VanLevel / Konfiguiere dein VanLevel</i></td>";
  HTMLPage += "</tr> <tr> <td style='text-align: right; vertical-align: top; width: 150px; border-bottom: 1px dotted #999;' colspan='3'>&nbsp;</td></tr> <tr> <td style='text-align: right; vertical-align: top; width: 150px;' colspan='3'>&nbsp;</td> </tr> <tr> <td style='text-align: center; vertical-align: middle; width: 150px;' rowspan='3'><b>";
  HTMLPage += "System</b></td>";
  HTMLPage += "<td style='text-align: right; vertical-align: middle; width: 150px;'>Alarm&nbsp;</td> <td style='text-align: left; vertical-align: top; width: 150px;'><input type='radio' name='Sound' value='1'  " + checked1 + " > on / an<br><input type = 'radio' name = 'Sound' value = '0' " + checked2 + "> off / aus&nbsp;</td> </tr> <tr> <td style = 'text-align: right; vertical-align: middle; width: 150px;'>Display&nbsp;</td> <td style = 'text-align: left; vertical-align: top; width: 150px;'> <input type = 'radio' name = 'Display' value = '3' " + checked3 + "> Normal<br><input type = 'radio' name = 'Display' value = '1' " + checked4 + "> 180&deg; </td > </tr > <tr> ";
  HTMLPage += "<td style = 'text-align: right; vertical-align: middle; width: 150px;'>Standby&nbsp;</td > <td style = 'text-align: left; vertical-align: top; width: 150px;'><input type = 'radio' name = 'Standby' value = '15' " + checked5 + "> 15 min<br><input type = 'radio' name = 'Standby' value = '10' " + checked6 + "> 10 min<br><input type = 'radio' name = 'Standby' value = '5' " + checked7 + "> &nbsp; 5 min&nbsp;</td> </tr> <tr> <td style = 'text-align: right; vertical-align: top; width: 150px; border-bottom: 1px dotted #999;' colspan = '3'>&nbsp;</td> </tr> <tr> <td style = 'text-align: right; vertical-align: top; width: 150px;' colspan = '3'>&nbsp;</td > </tr > <tr> <td style = 'text-align: center; vertical-align: middle; width: 150px;' rowspan = '2'><b>Vehicle<br>Fahrzeug </b > </td > <td style = 'text-align: right; vertical-align: top; width: 150px;'>Wheelbase&nbsp;<br>Radstand&nbsp;</td> <td style = 'text-align: left; vertical-align: middle; width: 150px;'> <input style = 'border:1px solid; border-radius: 5px; text-align: center;' id = 'Radstand' max = '999' min = '100' name = 'Radstand' step = '10' type = 'number' value = '";
  HTMLPage += Radstand;
  HTMLPage += "'/>  &nbsp;cm </td> ";
  HTMLPage += " </tr> <tr> <td style = 'text-align: right; vertical-align: top; width: 150px;'>Track width&nbsp;<br>Spurbreite&nbsp;</td> <td style = 'text-align: left; vertical-align: middle; width: 150px;'> <input style = 'border:1px solid; border-radius: 5px; text-align: center;' id = 'Spurbreite' max = '999' min = '100' name = 'Spurbreite' step = '10' type = 'number' value = '";
  HTMLPage += Spurweite;
  HTMLPage += "'/> &nbsp;cm </td> ";
  HTMLPage += " </tr> <tr> <td style = 'text-align: right; vertical-align: top; width: 150px; border-bottom: 1px dotted #999;' colspan = '3'>&nbsp; </td> </tr> <tr> <td style = 'text-align: right; vertical-align: top; width: 150px;'> </td> <td style = 'text-align: left; vertical-align: top; width: 150px;'>&nbsp; </td> </tr> <tr>";
  HTMLPage += "<td style = 'text-align: center; vertical-align: middle; width: 150px;' rowspan = '5'><b>Leveling block hight<br>Keilstufen </b> </td> <td style = 'text-align: right; vertical-align: top; width: 150px;'>Level 1 &nbsp; </td> <td style = 'text-align: left; vertical-align: top; width: 150px;'> <input style = 'border:1px solid; text-align: center; border-radius: 5px;' id = 'Level_1' max = '30' min = '1' name = 'Level_1' step = '1' type = 'number' value = '";
  HTMLPage += limit_1;
  HTMLPage += "'/> &nbsp; cm </td> ";
  HTMLPage += " </tr> <tr> <td style = 'text-align: right; vertical-align: top; width: 150px;'>Level 2 &nbsp; </td> <td style = 'text-align: left; vertical-align: top; width: 150px;'> <input style = 'border:1px solid; text-align: center; border-radius: 5px;' id = 'Limit_2' max = '30' min = '2' name = 'Limit_2' step = '1' type = 'number' value = '";
  HTMLPage += limit_2;
  HTMLPage += "'/> &nbsp; cm </td> ";
  HTMLPage += " </tr> <tr> <td style = 'text-align: right; vertical-align: top; width: 150px;'>Level 3 &nbsp; </td> <td style = 'text-align: left; vertical-align: top; width: 150px;'> <input style = 'border:1px solid; text-align: center; border-radius: 5px;' id = 'Limit_3' max = '30' min = '3' name = 'Limit_3' step = '1' type = 'number' value = '";
  HTMLPage += limit_3;
  HTMLPage += "'/> &nbsp; cm </td> ";
  HTMLPage += " </tr> <tr> <td style = 'text-align: right; vertical-align: top; width: 150px;'>Level 4 &nbsp; </td> <td style = 'text-align: left; vertical-align: top; width: 150px;'> <input style = 'border:1px solid; text-align: center; border-radius: 5px;' id = 'Limit_4' max = '30' min = '4' name = 'Limit_4' step = '1' type = 'number' value = '";
  HTMLPage += limit_4;
  HTMLPage += "'/> &nbsp; cm </td> ";
  HTMLPage += " </tr> <tr> <td style = 'text-align: right; vertical-align: top; width: 150px;'>Level 9 &nbsp; </td> <td style = 'text-align: left; vertical-align: top; width: 150px;'> <input style = 'border:1px solid; text-align: center; border-radius: 5px;' id = 'Limit_5' max = '30' min = '5' name = 'Limit_5' step = '1' type = 'number' value = '";
  HTMLPage += limit_5;
  HTMLPage += "'/> &nbsp; cm </td> ";
  HTMLPage += " </tr> <tr> <td style = 'text-align: right; vertical-align: top; width: 150px; border-bottom: 1px dotted #999;' colspan = '3'>&nbsp;</td> </tr>  <tr> <td style = 'text-align: right; vertical-align: top; width: 150px;' colspan = '3'>&nbsp;</td> </tr> <tr> <td style = 'text-align: center; vertical-align: center; width: 150px;' colspan = '3'><!--<input type = 'submit' name = 'button' style = 'border:1px solid green; color: green; border-radius: 5px;'value = 'Update'>--><button type='submit' name = 'button' value = '1' style = 'border:1px solid green; color: green; border-radius: 5px;'>Update</button> </td> </tr> <tr> <td style = 'text-align: right; vertical-align: top; width: 150px; border-bottom: 1px dotted #999;' colspan = '3'>&nbsp; </td> </tr> <tr> <td style = 'text-align: center; vertical-align: middle; width: 150px;'>&nbsp; </td> <td style = 'text-align: right; vertical-align: top; width: 150px;'>&nbsp; </td> <td style = 'text-align: left; vertical-align: top; width: 150px;'>&nbsp; </td> </tr> ";
  HTMLPage += "<tr> <td style = 'text-align: center; vertical-align: middle; width: 150px;' colspan = '3'><b>Update and Calibrate System </b> <br> please make shure the VanLevel is placed in the right position and your vehicle is in level. After start of this operation the actual level is set as new basis level. <br>&nbsp; </br> <b>System Kalibrieren und neue Einstellungen &uuml;bernehmen</b> <br> Bitte beachte das dein Fahrzeug / Anh&auml;nger gerade steht. Baue VanLevel am vorgesehen Platz ein und starte die Kalibration. Nach der Kalibration ist die aktuelle Fahrzeugeinstellung die neue Referenz. <br> <br> </td> </tr> <tr> <td style = 'text-align: center; vertical-align: middle; width: 150px;'>&nbsp; </td> <td style = 'text-align: center; vertical-align: top; width: 150px;'><!--<input type = 'submit' name = 'button' style = 'border:1px solid red; color: red; border-radius: 5px;' value = 'Calibrate / Kalibrieren'>--><button type='submit' name = 'button' value = '2' style = 'border:1px solid red; color: red; border-radius: 5px;'>Calibrate / Kalibrieren</button></form> </td> <td style = 'text-align: left; vertical-align: top; width: 150px;'>&nbsp; </td> </tr> ";
  HTMLPage += " </tbody> ";
  HTMLPage += " </table> </P> <P> </P>";
  HTMLPage += " </body> </html> ";
  HTMLPage += "";
  return HTMLPage;
}
