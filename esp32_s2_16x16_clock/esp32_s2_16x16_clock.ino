// 16x16 WS2812 5050 RGB WiFi clock converted for ESP32-S2 Mini
// HTML goes in data folder >> Upload LittleFS image to board

/*
  Required Libraries to install in Arduino IDE:
  - NeoPixelBus by Makuna (https://github.com/Makuna/NeoPixelBus)
  - Adafruit AHTX0 by Adafruit
  - AsyncTCP by me-no-dev / mathieucarbou (https://github.com/mathieucarbou/AsyncTCP)
  - ESPAsyncWebServer (https://github.com/mathieucarbou/ESPAsyncWebServer)
  - MQUnifiedsensor by Miguel California
  - Wire, WiFi, WiFiClientSecure, LittleFS, FS (Built-in with ESP32 Arduino Board Package)

  Default Pin Assignments (ESP32-S2 Mini):
  - WS2812 LED Data:   GPIO 7
  - AHT Temp/Humd I2C: GPIO 33 (SDA) & GPIO 35 (SCL) 
  - Motion Detect out: GPIO 16 [optional]
  - LDR positive:      GPIO 9  [optional]
  - Piezo Buzzer:      GPIO 5  [optional]
  - MQ-2 Smoke Sensor: GPIO 3 (Analog Input ADC1_CH2) [optional]

*/

/* TODO !!!                                                                    

test alarms to see if they work

wifi clock shows up in wifi SSID scan when off? fix!  (try forget SSID on phone?)
wifi AP indicator should change after reboot? not asap?

sms POST temps/humid, see serial output, POST sms disabled
 temp and humid both set to trigger by default. line ~179-180

send SMS string, for 2 phone numbers
visual flash mode for alarms, flash colour.
pull kpdata1.txt to browser to parse like dbfm?

Flashlight modes:
 fireplace - paint screen shifting colours of red orange?
   pre make a 15x15 flame oval
   draw from bottom, each line up can be shifted randomly based on last number.
   shift will cause wiggle.

 lightning - generated bolts of white/yellow flashes
   draw forks of white/yellow pixels downward.
   randomly fork at 5-10 px, go back in array and fork from there

 rain - falling pixels
  remember 10 raindrops and move them down each frame

 rgb spiral -colour shifting spiral
  draw across x, down y, back x, up y-1, across x-1, etc
  change colour each step, gradient.

 fire works -
  single firework, shoots upwards, explodes 4 or 8 projectiles
  random colours

 starfield -
  plot 10 pixels, move them inward each frame

  cops - flash  rr w bb w rr w bb w...

Change url to derben.ca/aurora/WFC for main build
 data is on derben.ca!
 Does data update? Probably not, may need to visit chk.php

Set up smoke alarms to use beeps and rate limits, when hardware arrives

afc = change font colour every first of month. Data in HTML though??

LOW RAM/reboots
 load https asynch? seems more free ram fixes crash??
  https://github.com/esp8266/Arduino/blob/master/libraries/ESP8266HTTPClient/examples/StreamHttpsClient/StreamHttpsClient.ino
 switch more String to string or char* ??
 paint crashes?
 
Extra stuff:
 HTML games
  connect 4, snake, pixel side scroller, light cycles/tron

 check/filter all data inputs on ESP side to prevent reboots.
 brightness slider doesn't work on pause.
 dim display at *time* for x *hours* ?
  Sliders: dim %, normal %
 Sparkles mode! Random pixels always, only with sound
 PINS (ESP32-S2 Mini):
  GPIO 3 (Analog) MQ2 smoke
  GPIO 5 Piezo
  GPIO 7 WS2812B LED Data
  GPIO 33 SDA TempHumd
  GPIO 35 SCL TempHumd
*/

#include <NeoPixelBusLg.h>  // https://github.com/Makuna/NeoPixelBus
#include <Adafruit_AHTX0.h>
#include <WiFi.h>
#include <AsyncTCP.h>           // https://github.com/me-no-dev/AsyncTCP
#include <ESPAsyncWebServer.h>  // https://github.com/me-no-dev/ESPAsyncWebServer
#include <WiFiClientSecure.h>
#include <Wire.h>
#include "FS.h"
#include <LittleFS.h>  
#include <time.h>
#include <sys/time.h>
#include <MQUnifiedsensor.h> // detect flammable gas in a range of 300 - 10000ppm

#define se ";"                                              // info separator
#define kyst "(key stored)"                                 // key stored text
#define WFT 50                                              // Wifi tries
#define ARSZ(array) ((sizeof(array)) / (sizeof(array[0])))  // array size

// to be dynamically set in GUI
#define mq2Pin 3   // smoke pin (GPIO 3)
#define TonePin 5  // piezo pin
#define LED_Pin 7  // WS2812 LED pin
#define LDR_Pin 9  // light sensor
#define MOTION_Pin 16  // motion
#define SDA_Pin 33 // I2C SDA
#define SCL_Pin 35 // I2C SCL

#define Width 16         // const uint8_t Width = 16;
#define Height 16        // const uint8_t Height = 16;
#define LEDnum 256       // const int LEDnum = (Width * Height);
#define Serpentine true  // const bool Serpentine = true;
#define flip 0           // const int flip = 0;  //0=rhw 1=emu
// webserver
#define tp "text/plain"
#define th "text/html"
#define jn "application/json"
#define ct "Content-Type"
#define bm "<p><a href='/'>Back to menu</a>"
// files
#define xsmskey "/smskey"
#define xsmspn1 "/smspn1"
#define xsmspn2 "/smspn2"
#define xsmsurl "/smsurl"
#define xsmspost "/smspost"
#define xMyTZ "/MyTZ"
#define xMyTZsi "/MyTZsi"
#define xbp1 "/bp1"
#define xbp2 "/bp2"
#define xbp3 "/bp3"
#define xbp4 "/bp4"
#define xbp5 "/bp5"
#define xNTP1 "/NTP1"
#define xNTP2 "/NTP2"
#define xHideAP "/HideAP"
#define xAPSSID "/APSSID"
#define xAPPSK "/APPSK"
#define xSTASSID "/STASSID"
#define xSTAPSK "/STAPSK"
#define xSmkR0 "/SmkR0"

// MQ2
#define Board ("ESP32")
#define Pin (3)  // Analog input GPIO 3 of ESP32-S2
#define Type ("MQ-2") // MQ2
#define Voltage_Resolution (3.3) // 3V3
#define ADC_Bit_Resolution (12) // 12-bit ADC for ESP32
#define RatioMQ2CleanAir (9.83) // 9.83 for smoke
MQUnifiedsensor MQ2(Board, Voltage_Resolution, ADC_Bit_Resolution, Pin, Type);

const char* HideAP PROGMEM = "2";
const char* MyTZ PROGMEM = "EST5EDT,M3.2.0,M11.1.0";
const char* bp1 PROGMEM = "B1-4,CS2-4,CS5-4,E5-8,F5-4";
const char* bp2 PROGMEM = "B2-4,CS6-4,CS4-4,E1-8,F7-4";
const char* bp3 PROGMEM = "C2-2,D2-2,C2-2,D2-2";
const char* bp4 PROGMEM = "C7-2";
const char* bp5 PROGMEM = "C5-3,C3-3";
const char* APSSID PROGMEM = "WiFi Clock";
const char* APPSK PROGMEM = "ChangeMe2345738641934z23";
const char* NTP1 PROGMEM = "time.nist.gov";
const char* NTP2 PROGMEM = "pool.ntp.org";
const char* smsurl PROGMEM ="https://secure.smsgateway.ca/services/message.svc/!k!/!p!";
const char* smspost PROGMEM = R"=({ "MessageBody":"Wifi Clock, !d!" })=";
const char* a1ms PROGMEM = " Alarm1 active";
const char* a2ms PROGMEM = " Alarm2 active";
const char* aums PROGMEM = " Aurora KP:";
const char* sdms PROGMEM = " Smoke Detected";
const char* tmms PROGMEM = " Temp:";
const char* hums PROGMEM = " Humid:";
const char* tabo PROGMEM = "<table id='chbt'>";
const char* td1 PROGMEM = "<tr><td><a href='/edit?f=/";
const char* td2 PROGMEM = "</a></td><td> ";
const char* td3 PROGMEM = "<a href='/delt?f=/";
const char* td4 PROGMEM = "'>Del</a></td></tr>";
const char* tabc PROGMEM = "</table>";
const char* nanw PROGMEM = "Number of available networks:";
const char* btnz PROGMEM = "(Buttons below set SSID in Main tab)<p>";
const char* bt1 PROGMEM = "<button onclick='upd(";
const char* bt2 PROGMEM = ")' type='button' id='wf";
const char* plzw PROGMEM = "<meta http-equiv='refresh' content='8; url=/' /><h1>Please Wait... You may have to reconnect to WiFi.</h1>";
const char* btfs PROGMEM = "Poof! <a href='/files.htm'>back to files</a>";
const char* ed1 PROGMEM = "<form action='/sfile' method='post'><textarea name=data rows='25' cols='25'>";
const char* ed2 PROGMEM = "</textarea><br><input name='f' value='";
const char* ed3 PROGMEM = "'><button>Save</button></form>";
const char* smstav PROGMEM = "https://secure.smsgateway.ca/services/account.svc/!k!/RemainingMessageCount/";
const char* chkau PROGMEM = "https://dbfm.derben.ca/aurora/wfckp.txt";

// WS2812 strip configuration for ESP32 on GPIO 7
NeoPixelBusLg<NeoGrbFeature, NeoEsp32Rmt0Ws2812xMethod> strip(LEDnum, LED_Pin);

// Temp Humid
Adafruit_AHTX0 aht;
sensors_event_t humd, temp;
Adafruit_Sensor *aht_humd, *aht_temp;

// generated
int tht = 1;  // TmpHud toggle, limiter
float atemp = 0;
float ahumd = 0;
float asmok = 0;
float aaura = 0;  // KPx
int awifi = 0;  // wifi bars
int wfc[]={0,0};  // bootup stuff: [0]=wifi connected, or turn on AP & [1]=temp/humd loaded
int flax[]={0,0,0,0,0,0,0,0,0,0};
int flay[]={0,0,0,0,0,0,0,0,0,0};

// set variables
int tempadj = -2;
int afoco = 1;           // Font Colour 0=manual
int amspd[] = { 3, 0 };  // marquee speed, counter

// toggle modes
int flm = 0;  // flashlight mode
int h24 = 1;  // 1= 24h clock
int dck = 0;  // disable clock updates
int dec = 0;  // decimal in marquee
int rst = 0;  // reset
int nxf = 0;  // next frame lock
int hkp = 3;  // low bar, hide hp if below

// Alert configs - flag is used for rate limits, 0=do alert, 1=active/skip
int alm1[] = { 1,2,0,0,0,0,0,0,0,0,0,0,15 };  // h,m,sun-sat,flag,bz,sms,freq
int alm2[] = { 1,2,29,11,2024,0,0,0,15 };  // h,m,d,m,y,flag,bz,sms,freq
int msmk[] = { 100,0,0,0,5 };               // smoke ppm,flag,bz,sms,freq
int maur[] = { 5,0,0,0,1800,0 };            // kp,flag,bz,sms,freq,http
int mtem[] = { 40,-10,0,0,0,30 };         // temp mx,mn,flag,bz,sms,freq
int mhum[] = { 70,10,0,0,0,60 };          // humid mx,mn,flag,bz,sms,freq
int alRL[] = { 60,0 };                   // rate limit SMS, flag
int frames[] = { 1,0,1,1,0,0,1,1,0 }; // cur frame, wifi, temp, humid, dayMn, smoke, aurora, moon, weather
// if frame[0]=-1, marquee mode

// piezo alarms play() // pin GPIO 5
int pn[] = { -1,0,0 };  // (play this note) index -1=off , wait time
String pbufr;          // Play buffer
const int d1[89] PROGMEM = { 31, 33, 35, 37, 39, 41, 44, 46, 49, 52, 55, 58, 62, 65, 69, 73, 78, 82, 87, 93, 98, 104, 110, 117, 123, 131, 139, 147, 156, 165, 175, 185, 196, 208, 220, 233, 247, 262, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494, 523, 554, 587, 622, 659, 698, 740, 784, 831, 880, 932, 988, 1047, 1109, 1175, 1245, 1319, 1397, 1480, 1568, 1661, 1760, 1865, 1976, 2093, 2217, 2349, 2489, 2637, 2794, 2960, 3136, 3322, 3520, 3729, 3951, 4186, 4435, 4699, 4978 };
const char* d2[89] PROGMEM = { "B0", "C1", "CS1", "D1", "DS1", "E1", "F1", "FS1", "G1", "GS1", "A1", "AS1", "B1", "C2", "CS2", "D2", "DS2", "E2", "F2", "FS2", "G2", "GS2", "A2", "AS2", "B2", "C3", "CS3", "D3", "DS3", "E3", "F3", "FS3", "G3", "GS3", "A3", "AS3", "B3", "C4", "CS4", "D4", "DS4", "E4", "F4", "FS4", "G4", "GS4", "A4", "AS4", "B4", "C5", "CS5", "D5", "DS5", "E5", "F5", "FS5", "G5", "GS5", "A5", "AS5", "B5", "C6", "CS6", "D6", "DS6", "E6", "F6", "FS6", "G6", "GS6", "A6", "AS6", "B6", "C7", "CS7", "D7", "DS7", "E7", "F7", "FS7", "G7", "GS7", "A7", "AS7", "B7", "C8", "CS8", "D8", "DS8" };

time_t now; struct tm tm;
boolean flag = true;
unsigned long sc = 0; // uptime, max is 4294967295 secs
int scc=0; // tik rotations into the second. (like fps)
int tik = 0; // tick count
const int tikr = 25; // max ticks between update
int c1 = 0, c2 = 0; // center prev
int dn[8] = { 1, 5, 3, 4, -1, -1, -1 }; // Digits now
int arck[9][3] = { { 120, 120, 0 }, { 150, 150, 0 }, { 200, 200, 0 }, { 250, 210, 0 }, { 100, 210, 0 }, { 50, 230, 0 }, { 0, 250, 0 }, { 200, 75, 0 }, { 200, 50, 0 } }; // aurora colours
int row[5][3] = { { 255, 0, 0 }, { 255, 128, 0 }, { 255, 255, 0 }, { 0, 255, 0 }, { 0, 0, 255 } };                                            // rainbow
const int arwb[6][4] = { { 3, 128, 0, 0 }, { 5, 255, 0, 0 }, { 7, 128, 128, 0 }, { 9, 255, 255, 0 }, { 11, 0, 128, 0 }, { 13, 0, 255, 0 } };  // wifi bar colours

String marquee = "", marqueue = "";  // queued message, via http
int mpos[] = { 1, 0 };               // mac position ( pos 0-3, string index )

AsyncWebServer server(80);

String hty[] = {"", ""};  // client

const char* diga PROGMEM = "0123456789AbBcCDdEFGHhIiJjKLlnOoPprSTtUuVXxY:^+-*%/[]=_?!. ";  // ^=^c and *=deg
const short digz[] = {
  0x7B6F,  // 0b111101101101111 0
  0x7493,  // 0b111010010010011 1
  0x73E7,  // 0b111001111100111 2
  0x79E7,  // 0b111100111100111 3
  0x49ED,  // 0b100100111101101 4
  0x79CF,  // 0b111100111001111 5
  0x7BCF,  // 0b111101111001111 6
  0x4927,  // 0b100100100100111 7
  0x7BEF,  // 0b111101111101111 8
  0x79EF,  // 0b111100111101111 9
  0x5BEA,  // 0b101101111101010 A 10
  0x7BC9,  // 0b111101111001001 b
  0x3AEB,  // 0b011101011101011 B
  0x73C0,  // 0b111001111000000 c
  0x624E,  // 0b110001001001110 C
  0x3B6B,  // 0b011101101101011 D 15
  0x7BE4,  // 0b111101111100100 d
  0x73CF,  // 0b111001111001111 E
  0x13CF,  // 000b1001111001111 F
  0x7B4F,  // 0b111101101001111 G
  0x5BED,  // 0b101101111101101 H 20
  0x5BC9,  // 0b101101111001001 h
  0x7497,  // 0b111010010010111 I
  0x2482,  // 0b010010010000010 i
  0x7B27,  // 0b111101100100111 J
  0x7B04,  // 0b111101100000100 j 25
  0x5AED,  // 0b101101011101101 K
  0x7249,  // 0b111001001001001 L
  0x2492,  // 0b010010010010010 l
  0x5AC0,  // 0b101101011000000 n
  0x2B6A,  // 0b010101101101010 O 30
  0x2A80,  // 0b010101010000000 o
  0x12EB,  // 0b001001011101011 P
  0x1758,  // 0b001011101011000 p
  0x1BC0,  // 0b001101111000000 r
  0x39CE,  // 0b011100111001110 S 35
  0x2497,  // 0b010010010010111 T
  0x25D0,  // 0b010010111010000 t
  0x7B6D,  // 0b111101101101101 U
  0x6B40,  // 0b110101101000000 u
  0x2B6D,  // 0b010101101101101 V 40
  0x5AAD,  // 0b101101010101101 X
  0x5540,  // 0b101010101000000 x
  0x24AD,  // 0b010010010101101 Y
  0x0410,  // 0b000010000010000 :
  0x01CF,  // 0b000000111001111 ^c 45
  0x05D0,  // 0b000010111010000 +
  0x01C0,  // 0b000000111000000 -
  0x01EF,  // 0b000000111101111 *deg
  0x42A1,  // 0b100001010100001 %
  0x02A0,  // 0b000001010100000 / 50
  0x324B,  // 0b011001001001011 [
  0x6926,  // 0B110100100100110 ]
  0x0E38,  // 0b000111000111000 =
  0x7000,  // 0b111000000000000 _
  0x21A7,  // 0b010000110100111 ? 55
  0x2092,  // 0b010000010010010 !
  0x2000,  // 0b010000000000000 .
  0x0000   // 0b000000000000000 null
};
const int ldigz = ARSZ(digz) - 1;  // length of digz array

// functions
uint16_t XYz(uint8_t x, uint8_t y) {
  uint16_t i;
  if (Serpentine == false) {
    i = (y * Width) + x;
  } else {  // true
    if (y & 0x01) {
      uint8_t rX = flip ? (Width - 1) - x : x;
      i = (y * Width) + rX;
    } else {
      uint8_t rX = flip ? x : (Width - 1) - x;
      i = (y * Width) + rX;
    }
  }
  return i;
}

String moony(int flp=0){
 String out;
 time(&now);
 localtime_r(&now, &tm);
 int dat2=1618209000; // Apr 12, 2021, 2:30
 if (now<dat2) { return ""; }
 float dms = 86400; // seconds in day
 float dn=29.53059; // Days between new moon 29.5305882
 float tmp= (now-dat2) / dms / dn;
 tmp-=floor(tmp); // AKA; tmp=tmp % 1;
 float pct = tmp>.5 ? .5-(tmp-.5) : tmp;
 pct=pct*100*2;
 int apct=(int) pct;
 if (tmp<.1 || tmp>.9){ out="nE"; }
 if (tmp>=.1 && tmp<=.2){ out="Cr"; }
 if (tmp>.2 && tmp<.3){ out="HF"; }
 if (tmp>=.3 && tmp<=.4){ out="Gb"; }
 if (tmp>.4 && tmp<.6){ out="FL"; }
 if (tmp>=.6 && tmp<=.7){ out="Gb"; }
 if (tmp>.7 && tmp<.8){ out="HF"; }
 if (tmp>=.8 && tmp<=.9){ out="Cr"; }
 if (flp==1) { out+=" ("+String(pct)+"% lit)"; }
 if (flp==2) { out+=":"+String(apct)+"%"; }
 return out;
}

int WifiSig() {
 int draw = 0;
 if (WiFi.status() == WL_CONNECTED) {
  int bars = WiFi.RSSI();
  if (bars >= -60) { draw = 6; }
  if (bars <= -61 && bars > -70) { draw = 5; }
  if (bars <= -71 && bars > -80) { draw = 4; }
  if (bars <= -81 && bars > -90) { draw = 3; }
  if (bars <= -91 && bars > -100) { draw = 2; }
  if (bars <= -101 && bars > -110) { draw = 1; }
  if (bars <= -111) { draw = 0; }
 }
 return draw;
}

int fdnt(String ntmp) {  // Find note
  int out = -1;
  for (int i = 0; i < 89; i++) {
    if (ntmp == d2[i]) {
      out = d1[i];
      break;
    }
  }
  return out;
}

void play() {
  // check flag if not playing
  int tmp = millis() - pn[2];
  if (pn[1] > 0 && pn[1] > tmp) { return; }  // delay set
  if (pbufr == "") {                         // done
    noTone(TonePin);
    pn[0] = -1;
    return;
  }
  
  // split off part needed from buffer
  String note = "";
  int dely;
  int idx = pbufr.indexOf(',', 0);
  if (idx > -1) {
    note = pbufr.substring(0, idx);
    pbufr.replace(note + ",", "");
  } else {
    note = pbufr;
    pbufr = "";
  }
  idx = note.indexOf('-');
  dely = note.substring(idx + 1, -1).toInt();
  note = note.substring(0, idx);
  pn[1] = 1000 / dely;  // set flag
  pn[2] = millis();
  tone(TonePin, fdnt(note), 1000 / dely);  // play
}

String listFiles(AsyncWebServerRequest* request) {
  String out = tabo;
  File root = LittleFS.open("/");
  if (root && root.isDirectory()) {
    File file = root.openNextFile();
    while (file) {
      String fileName = String(file.name());
      if (fileName.startsWith("/")) {
        fileName = fileName.substring(1);
      }
      if (!file.isDirectory() && fileName.indexOf(".") == -1) {
        out += td1;
        out += fileName;
        out += "'>";
        out += fileName;
        out += td2;
        out += td3;
        out += fileName;
        out += td4;
      }
      file = root.openNextFile();
    }
  }
  out += tabc;
  return out;
}

String readFile(const char* path, int rd = 1) {  // rd=0 check file exist, 1=regular, 2=var mode
  String out;
  File f = LittleFS.open(path, "r");
  if (!f || f.isDirectory()) {
    if (rd != 2) {
      out = "! "+String(path);
    } else {
      if (path == xMyTZ) { out = MyTZ; }
      if (String(path) == xsmsurl) { out = smsurl; }
      if (String(path) == xsmspost) { out = smspost; }
      if (String(path) == xbp1) { out = bp1; }
      if (String(path) == xbp2) { out = bp2; }
      if (String(path) == xbp3) { out = bp3; }
      if (String(path) == xbp4) { out = bp4; }
      if (String(path) == xbp5) { out = bp5; }
      if (String(path) == xAPSSID) { out = APSSID; }
      if (String(path) == xAPPSK) { out = APPSK; }
      if (String(path) == xNTP1) { out = NTP1; }
      if (String(path) == xNTP2) { out = NTP2; }
      if (String(path) == xHideAP) { out = HideAP; }
    }
  }
  if (rd > 0 && f) {
   while (f.available()) {
    char x = f.read();
    out.concat(x);
   }
  }
  
  if (f) f.close();
  return out;
}

String writeFile(const char* path, const char* message) {
  if (message==kyst){ return "x"; }
  String out = "Unchanged";
  String tmp = String(readFile(path, 9));
  if (tmp != message) {
    File f = LittleFS.open(String(path), "w");
    if (!f) {
      Serial.println(F("!Err open write ") + String(path));
      out = "Fail";
    } else {
      if (f.print(message)) {
        Serial.println(F("wrote ") + String(path));
        out = "Saved";
      } else {
        Serial.println(F("!fail ") + String(path));
        out = "Failed";
      }
      f.close();
    }
  }
  return out;
}

String wifilist(AsyncWebServerRequest* request) {
  String pg = "";
  const AsyncWebParameter* p = request->getParam(0);
  if (p->value() == "1") {
    // call scan in bg
    pg = "scanning";
    int ns = WiFi.scanNetworks(true);
  } else {
    // get data
    int ns = WiFi.scanComplete();
    if (ns >= 0) {
      pg = nanw + String(ns) + "<br>";
      pg += btnz;
      for (int sd = 0; sd < ns; sd++) {
        String nam = WiFi.SSID(sd);
        pg += bt1 + String(sd) + bt2;
        pg += String(sd) + "' name='" + nam + "'>" + nam + "</button>  ";
        pg += String(WiFi.RSSI(sd)) + " dBm<br>";
      }
      WiFi.scanDelete();
    } else {
      pg = ".";
    }
  }
  return pg;
}

String restartx(AsyncWebServerRequest* request) {
  rst = 666;  // reset
  return plzw;
}

String dlfl(AsyncWebServerRequest* request) {  // delete file
  const AsyncWebParameter* p = request->getParam(0);
  LittleFS.remove(p->value().c_str());
  return btfs;
}

String edfl(AsyncWebServerRequest* request) {  // edit file
  const AsyncWebParameter* p = request->getParam(0);
  String data = readFile(p->value().c_str());
  return ed1 + data + ed2 + p->value().c_str() + ed3 +bm;
}

String sfile(AsyncWebServerRequest* request) {
  const AsyncWebParameter* p = request->getParam(1);  // name
  String tmp1 = String(p->value());
  p = request->getParam(0);  // data
  String tmp2 = String(p->value());
  String data = writeFile(tmp1.c_str(), tmp2.c_str());
  data += bm; // back to menu
  return data;
}

String infox(AsyncWebServerRequest* request) {
  const AsyncWebParameter* p = request->getParam(0);
  int full = p->value().toInt();
  String data;
  if (full == 2) { data += String(dck); }
  if (full < 2) {
    data += String(WifiSig()) + se + String(atemp) + se + String(ahumd) + se + String(asmok)+ se + String(aaura)+se+moony(1);
    data += se + String(sc) + se + String(ESP.getFreeHeap());  // uptime secs / free ram
    data += se + String(tm.tm_mday) + "/" + String(tm.tm_mon + 1) + "/" + String(tm.tm_year + 1900);  // date day month year
  }
  if (full == 1) {
    String tmp= readFile("/aglow");
    if (tmp.charAt(0) == '!') { tmp="50"; }
    data += se + tmp + se + String(amspd[0]) + se + readFile(xSTASSID,2) + se + String(afoco) + se + String(tempadj) + se;
    data += readFile(xSTAPSK,2)!="" ? kyst : ""; data += se;
    data += readFile(xAPPSK,2)!="" ? kyst : ""; data += se;
    for (int ii = 0; ii < ARSZ(frames); ii++) {
      data += frames[ii];
    }
    data += se + String(h24) + String(dck) + String(flm) + String(dec) + String(hkp);
    data += se + readFile(xMyTZ,2) + se + readFile(xMyTZsi,2);
    data += se + readFile(xbp1,2) + se + readFile(xbp2,2) + se + readFile(xbp3,2) + se + readFile(xbp4,2) + se + readFile(xbp5,2);
    data += se + WiFi.localIP().toString() + se + WiFi.softAPIP().toString();
    data += se + readFile(xAPSSID,2) + se + readFile(xHideAP,2);  // AP
    data += se + readFile(xNTP1,2) + se + readFile(xNTP2,2);      // NTP

    char bf[6];
    for (int ii = 0; ii < 5; ii++) {  // 5 font colours
      data += se;
      data += "#";
      String r = String(row[ii][0], HEX);
      String g = String(row[ii][1], HEX);
      String b = String(row[ii][2], HEX);
      sprintf(bf, "%02s", r.c_str());
      data += bf;  // add leading 0
      sprintf(bf, "%02s", g.c_str());
      data += bf;
      sprintf(bf, "%02s", b.c_str());
      data += bf;
    }
    data += se;
    for (int ii = 0; ii < ARSZ(alm1); ii++) {  // alm1
      String tmp = String(alm1[ii]);
      data += ii == 0 ? tmp : "," + tmp;
    }
    data += se;
    for (int ii = 0; ii < ARSZ(alm2); ii++) {  // alm2
      String tmp = String(alm2[ii]);
      data += ii == 0 ? tmp : "," + tmp;
    }
    data += se;
    for (int ii = 0; ii < ARSZ(mhum); ii++) {  // humd
      String tmp = String(mhum[ii]);
      data += ii == 0 ? tmp : "," + tmp;
    }
    data += se;
    for (int ii = 0; ii < ARSZ(mtem); ii++) {  // temp
      String tmp = String(mtem[ii]);
      data += ii == 0 ? tmp : "," + tmp;
    }
    data += se;
    for (int ii = 0; ii < ARSZ(msmk); ii++) {  // smok
      tmp = String(msmk[ii]);
      data += ii == 0 ? tmp : "," + tmp;
    }
    data += se;
    tmp = String(alRL[0]);
    data += tmp;
    data += se;
    for (int ii = 0; ii < ARSZ(maur); ii++) {  // aur
      tmp = String(maur[ii]);
      data += ii == 0 ? tmp : "," + tmp;
    }
    data += se; data +=readFile(xsmskey, 2) !="" ? kyst : "";
    String dd[] = { xsmspn1, xsmspn2, xsmsurl, xsmspost };
    for (int ii = 0; ii < ARSZ(dd); ii++) {
      data += se;
      data += readFile(dd[ii].c_str(), 2);
    }
  }
  return data;
}

String clrp(AsyncWebServerRequest* request) {  // clear/load img
  int ii = 0;
  dck = 1;
  for (int y = 0; y < 16; y++) {
    for (int x = 0; x < 16; x++) {
      if (request->params() > 0) {
        const AsyncWebParameter* p = request->getParam(ii);
        String t1 = String(p->value()[0]);
        String t2 = String(p->value()[1]);
        String r = t1 + t2;
        t1 = String(p->value()[2]);
        t2 = String(p->value()[3]);
        String g = t1 + t2;
        t1 = String(p->value()[4]);
        t2 = String(p->value()[5]);
        String b = t1 + t2;
        strip.SetPixelColor(XYz(x, y), RgbColor(strtoul(r.c_str(), NULL, 16), strtoul(g.c_str(), NULL, 16), strtoul(b.c_str(), NULL, 16)));
      } else {
        strip.SetPixelColor(ii, RgbColor(0, 0, 0));
      }
      ii++;
    }
  }
  return "ok";
}

String dpxs(AsyncWebServerRequest* request) {  // draw line
  dck = 1;
  int qr, qb, qg, qx, qy;
  for (int ii = 0; ii < request->params(); ii++) {
    const AsyncWebParameter* p = request->getParam(ii);
    if (p->name() == "r") {
      qr = strtol(p->value().c_str(), NULL, 16);
    } else if (p->name() == "g") {
      qg = strtol(p->value().c_str(), NULL, 16);
    } else if (p->name() == "b") {
      qb = strtol(p->value().c_str(), NULL, 16);
    } else {
      if (p->name()[0] == 'v') {
        int pch = p->value().indexOf("x");
        if (pch != -1) {
          qx = p->value().substring(0, pch).toInt();
          qy = p->value().substring(pch + 1).toInt();
          strip.SetPixelColor(XYz(qx, qy), RgbColor(qr, qg, qb));
        }
      }
    }
  }
  strip.Show();
  return "ok";
}

String dpx(AsyncWebServerRequest* request) {  // draw pixel on screen
  dck = 1;
  const AsyncWebParameter* qx = request->getParam(0);
  const AsyncWebParameter* qy = request->getParam(1);
  const AsyncWebParameter* p = request->getParam(2);
  int qr = strtol(p->value().c_str(), NULL, 16);
  p = request->getParam(3);
  int qg = strtol(p->value().c_str(), NULL, 16);
  p = request->getParam(4);
  int qb = strtol(p->value().c_str(), NULL, 16);

  strip.SetPixelColor(XYz(qx->value().toInt(), qy->value().toInt()), RgbColor(qr, qg, qb));
  strip.Show();
  return "ok";
}

String dttm(AsyncWebServerRequest* request) {  // set date and time
  const AsyncWebParameter* p = request->getParam(0);
  time_t rtc = p->value().toInt();
  timeval tv = { rtc, 0 };
  settimeofday(&tv, nullptr);
  return "ok";
}

String togl(AsyncWebServerRequest* request) {  // toggle checks
  String message = "";
  for (uint8_t i = 0; i < request->params(); i++) {
    const AsyncWebParameter* p = request->getParam(i);
    message += " " + p->name() + ": " + p->value() + "=";
    if (p->value() == "wfs") {
      frames[1] = !frames[1];
      message += frames[1];
    }
    if (p->value() == "sht") {
      frames[2] = !frames[2];
      message += frames[2];
    }
    if (p->value() == "shh") {
      frames[3] = !frames[3];
      message += frames[3];
    }
    if (p->value() == "shd") {
      frames[4] = !frames[4];
      message += frames[4];
    }
    if (p->value() == "smk") {
      frames[5] = !frames[5];
      message += frames[5];
    }
    if (p->value() == "aur") {
      frames[6] = !frames[6];
      message += frames[6];
    }
    if (p->value() == "moo") {
      frames[7] = !frames[7];
      message += frames[7];
    }
    if (p->value() == "wea") {
      frames[8] = !frames[8];
      message += frames[8];
    }
    if (p->value() == "h24") {
      h24 = !h24;
      message += h24;
    }
    if (p->value() == "dec") {
      dec = !dec;
      message += dec;
    }
    if (p->value() == "dck") {
      dck = !dck;
      message += dck;
    }
    if (p->value() == "mac") {
      frames[0] = frames[0] > -1 ? -1 : 1;
      message += frames[0];
      for (uint8_t y = 8; y < 16; y++) {
        for (uint8_t x = 0; x < 16; x++) {
          strip.SetPixelColor(XYz(x, y), RgbColor(0, 0, 0));
        }
      }
    }
  }
  return "ok";
}

String flmo(AsyncWebServerRequest* request) {  // flashlight mode
  RgbColor tcol;
  const AsyncWebParameter* p = request->getParam(0);
  if (p->name() == "flmo") {
    flm=p->value().toInt();
    if (flm==1) {
      dck = 1;  // disable clock
      const AsyncWebParameter* p = request->getParam(1);
      String qr = p->value().substring(0, 2);
      String qg = p->value().substring(2, 4);
      String qb = p->value().substring(4, 6);
      int tr = strtol(qr.c_str(), NULL, 16);
      int tg = strtol(qg.c_str(), NULL, 16);
      int tb = strtol(qb.c_str(), NULL, 16);
      tcol = RgbColor(tr, tg, tb);
    } else if (flm==0) {
      dck = 0;
      tcol = RgbColor(0, 0, 0);
    } else {
      dck=1; tcol=RgbColor(0, 0, 0);
    }
    for (int ii = 0; ii < LEDnum; ii++) {  // fill screen
      strip.SetPixelColor(ii, tcol);
    }
    strip.Show();
  }
  return "ok";
}

void setvar(String nam, String dat) {
  int i = 0, j = 0, k = 0;
  while (i > -1) {
    j = dat.indexOf(',', i + 1);
    i = i == 0 ? i : i + 1;
    int tmp = dat.substring(i, j).toInt();
    if (nam == "alm1") { alm1[k++] = tmp; }
    if (nam == "alm2") { alm2[k++] = tmp; }
    if (nam == "mtem") { mtem[k++] = tmp; }
    if (nam == "mhum") { mhum[k++] = tmp; }
    if (nam == "msmk") { msmk[k++] = tmp; }
    if (nam == "maur") { maur[k++] = tmp; }
    if (nam == "alRL") { alRL[k++] = tmp; }
    i = dat.indexOf(',', j);
  }
  return;
}

String saveForm(AsyncWebServerRequest* request) {
  char buf[15];
  const AsyncWebParameter* p = request->getParam(0);
  String typ = p->value();
  for (int ii = 1; ii < request->params(); ii++) {
    const AsyncWebParameter* p = request->getParam(ii);
    strcpy(buf, "/");
    strcat(buf, p->name().c_str());
    if (p->value() == "-1") {  // delete file
     LittleFS.remove(buf);
    } else {
     if (p->name() != "") { writeFile(buf, p->value().c_str()); }
    }
    if (p->name() != "") {
     if (typ == "data") {  // set data vars
      if (p->name() == "amspd") { amspd[0] = p->value().toInt(); }
      }
      if (typ == "alrm") {  // set data vars
        setvar(p->name(), p->value());
      }
    }
  }
  return "Settings saved!";
}

String sped(AsyncWebServerRequest* request) {  // marq speed
  const AsyncWebParameter* p = request->getParam(0);
  amspd[0] = p->value().toInt();
  return "ok";
}

String glow(AsyncWebServerRequest* request) {  // brightness aglow
  const AsyncWebParameter* p = request->getParam(0);
  strip.SetLuminance(p->value().toInt());
  return "ok";
}

String tadj(AsyncWebServerRequest* request) {  // temp adjust
  const AsyncWebParameter* p = request->getParam(0);
  tempadj = p->value().toInt();
  return "ok";
}

String tb(AsyncWebServerRequest* request) {  // test beep
  const AsyncWebParameter* p = request->getParam(0);
  pn[0] = 0;  // reset play
  pbufr = p->value();
  return "ok";
}

String sms(AsyncWebServerRequest* request) {  // sms
  String ky, p1, p2, ty;
  String u, d, out;
  for (int ii = 0; ii < request->params(); ii++) {
    const AsyncWebParameter* p = request->getParam(ii);
    if (p->name() == "t") { ty = p->value(); }
    if (p->name() == "k") { ky = p->value(); }
    if (p->name() == "p") { p1 = p->value(); }
    if (p->name() == "s") { p2 = p->value(); }
  }
  if (ty == "0") {  // get reply
    if (hty[0]=="" && hty[1]!="") {
      out = hty[1];
      hty[0]="";
    } else {
      out = "!No Reply!";
    }
  }
  if (ty == "1" && ky != "") {  // check balance
    u = smstav;
    if (ky==kyst) {
     ky= readFile("/smskey");
    }
    u.replace("!k!", ky);
    hty[1] = ""; // get, set first
    hty[0] = u; // URL, set last
    out = "ok";
  }
  return out;
}

String foco(AsyncWebServerRequest* request) {  // font colour
  const AsyncWebParameter* p = request->getParam(0);
  afoco = p->value().toInt();
  for (int i = 0; i < 5; i++) {
    const AsyncWebParameter* p = request->getParam(i + 1);
    String qr = p->value().substring(0, 2);
    String qg = p->value().substring(2, 4);
    String qb = p->value().substring(4, 6);
    int tr = strtol(qr.c_str(), NULL, 16);
    int tg = strtol(qg.c_str(), NULL, 16);
    int tb = strtol(qb.c_str(), NULL, 16);
    row[i][0] = tr;
    row[i][1] = tg;
    row[i][2] = tb;
  }
  return "ok";
}

String newf(AsyncWebServerRequest* request) {
  const AsyncWebParameter* p = request->getParam(0);
  writeFile(("/" + p->value()).c_str(), "");
  return "File created, refresh!";
}

void notFound(AsyncWebServerRequest* request) {
 char out[50]="Not found!";
 strcat(out,bm);
 request->send(404, th, out);
}

String marq(AsyncWebServerRequest* request) {
  const AsyncWebParameter* p = request->getParam(0);
  marqueue = "         " + p->value() + "     ";
  mpos[1] = 0;
  frames[0] = -1;
  return "ok";
}

String kpch(AsyncWebServerRequest* request) {
  if (request->params() > 0) {
    const AsyncWebParameter* p = request->getParam(0);
    if (p != nullptr) {
      hkp = p->value().toInt();
    }
  }
  return "ok";
}

void setup() {
 Serial.begin(115200);
 delay(1000);  // wait for serial attach on ESP32

 // MQ2
 MQ2.setRegressionMethod(1); // _PPM =  a*ratio^b
 MQ2.setA(30000000); MQ2.setB(-8.308);

 MQ2.init();
 float calcR0 = 0;
 String tmp = readFile(xSmkR0);// load smoke R0 data
 if (tmp.charAt(0) != '!'){ // NA = Run once only clean air
  calcR0 = tmp.toFloat();
 } else {
  Serial.print("Calibrating please wait.");
  
  for(int i = 1; i<=10; i ++) {
    MQ2.update(); // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ2.calibrate(RatioMQ2CleanAir);
    Serial.print(".");
  }
  MQ2.setR0(calcR0/10);
  char ChValue[10];
  dtostrf(calcR0, 10, 8, ChValue); // convert float to string
  writeFile(xSmkR0,ChValue); // save calcR0 to data
  Serial.println("  done!.");
 }
  if(isinf(calcR0)) {Serial.println("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply"); while(1);}
  if(calcR0 == 0){Serial.println("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply"); while(1);}
  MQ2.serialDebug(true);

  strip.Begin();  // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.Show(); // clear

  if (!LittleFS.begin(true)) { // true = formatOnFail for ESP32
    Serial.println(F("LittleFS Mount Failed"));
  } else {  // load variables on boot
    String tmp;
    // display
    tmp = readFile("/tempadj");
    tempadj = tmp.charAt(0) != '!' ? tmp.toInt() : tempadj;
    tmp = readFile("/amspd");
    amspd[0] = tmp.charAt(0) != '!' ? tmp.toInt() : amspd[0];

    tmp = readFile("/aglow");
    int aglow = tmp.charAt(0) != '!' ? tmp.toInt() : 50;
    strip.SetLuminance(aglow);

    tmp = readFile("/afoco");
    afoco = tmp.charAt(0) != '!' ? tmp.toInt() : afoco;
    for (int ii = 1; ii < 6; ii++) {  // font1
      tmp = "/f" + String(ii);
      tmp = readFile(tmp.c_str());
      if (tmp.charAt(0) != '!') {
        // convert hex to decimal
        String qr = tmp.substring(1, 3);
        String qg = tmp.substring(3, 5);
        String qb = tmp.substring(5, 7);
        int tr = strtol(qr.c_str(), NULL, 16);
        int tg = strtol(qg.c_str(), NULL, 16);
        int tb = strtol(qb.c_str(), NULL, 16);
        row[ii - 1][0] = tr;
        row[ii - 1][1] = tg;
        row[ii - 1][2] = tb;
      }
    }
    // toggle
    tmp = readFile("/frames");
    if (tmp.charAt(0) != '!') {
      frames[0] = tmp.charAt(0) == '1' ? -1 : 0;  // marquee
      for (int ii = 1; ii < ARSZ(frames); ii++) {
        frames[ii] = tmp.charAt(ii) == '1' ? 1 : 0;
      }
    }
    tmp = readFile("/h24");
    h24 = tmp.charAt(0) != '!' ? tmp.toInt() : h24;
    tmp = readFile("/dec");
    dec = tmp.charAt(0) != '!' ? tmp.toInt() : dec;
    tmp = readFile("/hkp");
    hkp = tmp.charAt(0) != '!' ? tmp.toInt() : hkp;
    // alarms
    tmp = readFile("/alm1");
    if (tmp.charAt(0) != '!') { setvar("alm1", tmp); }
    tmp = readFile("/alm2");
    if (tmp.charAt(0) != '!') { setvar("alm2", tmp); }
    tmp = readFile("/msmk");
    if (tmp.charAt(0) != '!') { setvar("msmk", tmp); }
    tmp = readFile("/mhum");
    if (tmp.charAt(0) != '!') { setvar("mhum", tmp); }
    tmp = readFile("/mtem");
    if (tmp.charAt(0) != '!') { setvar("mtem", tmp); }
    tmp = readFile("/maur");
    if (tmp.charAt(0) != '!') { setvar("maur", tmp); }
    tmp = readFile("/alRL");
    if (tmp.charAt(0) != '!') { setvar("alRL", tmp); }
  }  // load v on boot end

  Wire.begin(SDA_Pin, SCL_Pin);
  aht.begin(&Wire);  // temp/hum
  yield();
  String SID = readFile(xSTASSID, 2);
  String PSK = readFile(xSTAPSK, 2);
  String RDF0, RDF1, RDF2;
  RDF2 = readFile(xHideAP, 2);
  if (SID == "" || PSK == "") { RDF2 = "2"; }  // Turns on AP if WiFi not config
  if (RDF2 == "0") {                           // 0=disable, 1=hide, 2=show
    WiFi.mode(WIFI_STA);
    Serial.println(F("STA Only"));
  } else {
    WiFi.mode(WIFI_AP_STA);
    bool hide = RDF2 == "1" ? true : false;
    RDF0 = readFile(xAPSSID, 2);
    RDF1 = readFile(xAPPSK, 2);
    String tmp = WiFi.softAP(RDF0.c_str(), RDF1.c_str(), 8, hide) ? "Ready" : "Failed";
    Serial.println(F("AP: ") + tmp);
    if (tmp == "Ready") {
      Serial.println(F("Clock Wifi AP:"));
      Serial.println(F("APSSID: ") + RDF0);
      Serial.println(F("APPSK: ") + RDF1);
      Serial.print(F("APIP: "));
      Serial.println(WiFi.softAPIP());
    }
  }

  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);

  if (SID != "" && PSK != "") {
    WiFi.begin(SID.c_str(), PSK.c_str());
    delay(100);
    Serial.println(F("Local Wifi:"));
    Serial.println(F("SSID: ") + SID);
    Serial.println(F("PSK: ") + PSK);
    wfc[0] = 0;
    WiFi.status();
  }

  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.htm");
  server.on("/restart", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(200, th, restartx(request));
  });
  server.on("/edit", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(200, th, edfl(request));
  });
  server.on("/delt", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(200, th, dlfl(request));
  });
  server.on("/sfile", HTTP_POST, [](AsyncWebServerRequest* request) {
    request->send(200, th, sfile(request));
  });
  server.on("/wifisig", HTTP_POST, [](AsyncWebServerRequest* request) {
    request->send(200, tp, String(WifiSig()));
  });
  server.on("/wifilist", HTTP_POST, [](AsyncWebServerRequest* request) {
    request->send(200, th, wifilist(request));
  });
  server.on("/togl", HTTP_POST, [](AsyncWebServerRequest* request) {
    request->send(200, tp, togl(request));
  });
  server.on("/dpxs", HTTP_POST, [](AsyncWebServerRequest* request) {
    request->send(200, tp, dpxs(request));
  });
  server.on("/dpx", HTTP_POST, [](AsyncWebServerRequest* request) {
    request->send(200, tp, dpx(request));
  });
  server.on("/clrp", HTTP_POST, [](AsyncWebServerRequest* request) {
    request->send(200, tp, clrp(request));
  });
  server.on("/info", HTTP_POST, [](AsyncWebServerRequest* request) {
    request->send(200, tp, infox(request));
  });
  server.on("/dttm", HTTP_POST, [](AsyncWebServerRequest* request) {
    request->send(200, tp, dttm(request));
  });
  server.on("/marq", HTTP_POST, [](AsyncWebServerRequest* request) {
    request->send(200, tp, marq(request));
  });
  server.on("/kpch", HTTP_POST, [](AsyncWebServerRequest* request) {
    request->send(200, tp, kpch(request));
  });
  server.on("/glow", HTTP_POST, [](AsyncWebServerRequest* request) {
    request->send(200, tp, glow(request));
  });
  server.on("/sped", HTTP_POST, [](AsyncWebServerRequest* request) {
    request->send(200, tp, sped(request));
  });
  server.on("/foco", HTTP_POST, [](AsyncWebServerRequest* request) {
    request->send(200, tp, foco(request));
  });
  server.on("/saveForm", HTTP_POST, [](AsyncWebServerRequest* request) {
    request->send(200, tp, saveForm(request));
  });
  server.on("/tadj", HTTP_POST, [](AsyncWebServerRequest* request) {
    request->send(200, tp, tadj(request));
  });
  server.on("/flmo", HTTP_POST, [](AsyncWebServerRequest* request) {
    request->send(200, tp, flmo(request));
  });
  server.on("/tb", HTTP_POST, [](AsyncWebServerRequest* request) {
    request->send(200, tp, tb(request));
  });
  server.on("/sms", HTTP_POST, [](AsyncWebServerRequest* request) {
    request->send(200, tp, sms(request));
  });
  server.on("/listf", HTTP_POST, [](AsyncWebServerRequest* request) {
    request->send(200, th, listFiles(request));
  });
  server.on("/newf", HTTP_POST, [](AsyncWebServerRequest* request) {
    request->send(200, th, newf(request));
  });
  server.onNotFound(notFound);
  server.begin();

  aht_temp = aht.getTemperatureSensor();
  aht_temp->printSensorDetails();
  aht_humd = aht.getHumiditySensor();
  aht_humd->printSensorDetails();
}

void Drawr(int xx, int yy, int data, int modx = 0) {  // puts 5x3 letter/num into buffer
 if (data<0 || data>ldigz) { Serial.println(data); return; }
  int ii = 1;
  for (uint8_t y = 0; y < 5; y++) {
    for (uint8_t x = 0; x < 3; x++) {
      uint8_t tx = x;
      tx = 3 - x;        // invert
      int cx = xx - tx;  // Width-1-xx;
      int cy = yy + y;
      if (cx < 16 && cy < 16 && cx >= 0 && cy >= 0) {  // trims off OoB
        if (digz[data] & ii) {
          strip.SetPixelColor(XYz(cx, cy), RgbColor( row[y][0], row[y][1], row[y][2] ));
        } else {
          if (!modx || (x == 1)) {
            strip.SetPixelColor(XYz(cx, cy), RgbColor(0,0,0));
          }
        }
      }
      ii = ii + ii;
    }
  }
  return;
}

void alarm() {
 // reset flags if expired.
 if (msmk[1]<sc) { msmk[1]=0; }
 if (maur[1]<sc) { maur[1]=0; }
 if (mtem[2]<sc) { mtem[2]=0; }
 if (mhum[2]<sc) { mhum[2]=0; }
 if (alm1[9]<sc) { alm1[9]=0; }
 if (alm2[5]<sc) { alm2[5]=0; }
 if (alRL[1]<sc) { alRL[1]=0; }
 
 String SMS="", tmp="/bp";

 if (asmok > msmk[0]) {
  if (msmk[3] == 1) { // set SMS
  }
 }
 
 if (aaura >= maur[0]) {
  if (pbufr=="" && maur[1]==0 && maur[2] > 0 && maur[2] < 6) {
   maur[1]=sc+maur[4]; // uptime + beep rate limit 
   tmp+=String(maur[2]);
   pbufr = readFile(tmp.c_str(),2).c_str();
   pn[0] = 0;
  }
  if (maur[3] == 1) { // set SMS
   SMS+=aums + String(aaura);
  }
 }
 if (ahumd > mhum[0] || ahumd < mhum[1]) {
   if (pbufr=="" && mhum[2]==0 && mhum[3] > 0 && mhum[3] < 6) {
    mhum[2]=sc+mhum[5]; // uptime + beep rate limit 
    tmp+=String(mhum[3]);
    pbufr = readFile(tmp.c_str(),2).c_str();
    pn[0] = 0;
   }
   if (mhum[4] == 1) { // set SMS
    SMS+=hums+ String(ahumd)+"%";
   }
  }
  if (atemp + tempadj > mtem[0] || atemp + tempadj < mtem[1]) {
   if (pbufr=="" && mtem[2]==0 && mtem[3] > 0 && mtem[3] < 6) {
    mtem[2]=sc+mtem[5]; // uptime + beep rate limit 
    tmp+=String(mtem[3]);
    pbufr = readFile(tmp.c_str(),2).c_str();
    pn[0] = 0;
   }
   if (mtem[4] == 1) { // send SMS
    SMS+=tmms+String(atemp + tempadj)+"C";
   }
  }
 if (tm.tm_hour == alm1[0] && tm.tm_min == alm1[1]) {  // check alarm1
  for (int i = 0; i < 7; i++) {
   if (alm1[i+2]==1 && tm.tm_wday==i) {
  
    if (pbufr=="" && alm1[9]==0 && alm1[10] > 0 && alm1[10] < 6) {
     alm1[9]=sc+alm1[12]; // uptime + beep rate limit 
     tmp+=String(alm1[10]);
     pbufr = readFile(tmp.c_str(),2).c_str();
     pn[0] = 0;
    }
    if (alm1[11] == 1) { // send SMS
     SMS+=a1ms;
    }
   }
  }
 }
 if (tm.tm_hour == alm2[0] && tm.tm_min == alm2[1] && tm.tm_mday == alm2[2] && tm.tm_mon == alm2[3] - 1) {  // check alarm2
  if (pbufr=="" && alm2[5]==0 && alm2[6] > 0 && alm2[6] < 6) {
   alm2[5]=sc+alm2[8]; // uptime + beep rate limit 
   tmp+=String(alm2[6]);
   pbufr = readFile(tmp.c_str(),2).c_str();
   pn[0] = 0;
  }
  if (alm2[7] == 1) { // send SMS
   SMS+=a2ms;
  }
 }
 if (SMS!="" && alRL[1]==0) {
  // Send SMS here!
  alRL[1]=(alRL[0]*60)+sc; // alRL[0]*60 for minutes
  Serial.println("SMS:"+SMS);

  String smspost=readFile(xsmspost,2);
  // change !d! for SMS
  smspost.replace("!d!",SMS);

  String smsurl=readFile(xsmsurl,2);
  // change !k! and !p! for key and phone number.
  smsurl.replace("!k!",readFile(xsmskey,2));
  smsurl.replace("!p!",readFile(xsmspn1,2));

  Serial.println(smsurl);
  Serial.println(smspost);
  
  hty[0]=smsurl;
  hty[1]=smspost;
 }
}

void Dtime(int numb) {  // draw time -top half, based on numb
  int tmp, i;
  time(&now);
  localtime_r(&now, &tm);
  boolean fp = flag;
  flag = tm.tm_sec % 2;
  if (fp != flag) { sc++; scc=0; }  // uptime sec counter

  int dp[8] = { -1, -2, -3, -4, -5, -6, -7 };  // Digits prv
  for (i = 0; i < 8; i++) {
    dp[i] = dn[i];
  }

  if (h24 == 0 && tm.tm_hour > 12) {  // 24h/12h
    dn[0] = (tm.tm_hour - 12) / 10;
    dn[1] = (tm.tm_hour - 12) % 10;
  } else if (h24 == 0 && tm.tm_hour == 0) {  // 24h/12h
    dn[0] = 1;
    dn[1] = 2;
  } else {
    dn[0] = tm.tm_hour / 10;  // hour 1
    dn[1] = tm.tm_hour % 10;  // hour 2
  }
  dn[0] = dn[0] == 0 ? ldigz : dn[0];  // hide 0
  dn[2] = tm.tm_min / 10;              // min 1
  dn[3] = tm.tm_min % 10;              // min 2

  // center if 3 dig
  int cen1 = dn[0] == ldigz ? -2 : -4;
  if (cen1 != c1) {
    for (int y=0; y<8; y++) {
      for (int x=0; x<16; x++) {
        strip.SetPixelColor(XYz(x, y), RgbColor(0, 0, 0));
      }
    }
  }
  c1 = cen1;  // store prev, used for clearing
  if (numb == 4) {
    tmp = flag ? 44 : ldigz;     // : or blank
    Drawr(6 - cen1, 2, tmp, 1);  // 1 column mode
    if (WiFi.status() != WL_CONNECTED) {
     RgbColor rgb = flag ? RgbColor(0,0,0) : RgbColor(200,0,0);
     strip.SetPixelColor(XYz(0,0), rgb);
    } else {
     strip.SetPixelColor(XYz(0,0), RgbColor(0,0,0));
    }
  } else {
    int xx = numb * 4;
    Drawr(xx - cen1, 2, dn[numb]);
  }
}

void drwifi() {  // 0=draw 1=data
  int tmp = WifiSig();
  for (int x=0; x<16; x++) {
      for (int y=9; y<14; y++) {
        strip.SetPixelColor(XYz(x, y), RgbColor( 0, 0, 0 ));  // clear lower
      }
    }
    for (int x=2; x<15; x++) {  // wifi H lines
      strip.SetPixelColor(XYz(x, 9), RgbColor( row[0][0], row[0][1], row[0][2] ));
      strip.SetPixelColor(XYz(x, 13), RgbColor( row[0][0], row[0][1], row[0][2] ));
    }
    for (int y=10; y<13; y++) {  // wifi V lines
      strip.SetPixelColor(XYz(2, y), RgbColor( row[0][0], row[0][1], row[0][2] ));
      strip.SetPixelColor(XYz(14, y), RgbColor( row[0][0], row[0][1], row[0][2] ));
    }
    for (int z=0; z<tmp; z++) {  // wifi bars
      for (int y=10; y<13; y++) {
        strip.SetPixelColor(XYz(arwb[z][0], y), RgbColor( arwb[z][1], arwb[z][2], arwb[z][3] ));
      }
  }
}

void Dfram(int numb) {                         // draw frame -bottom half, based on numb
  int dp[8] = { -1, -2, -3, -4, -5, -6, -7 };  // Digits prv
  for (int i = 0; i < 8; i++) {
    dp[i] = dn[i];
  }
  int cen2 = dp[4] == ldigz ? -2 : -4;
  if (cen2 != c2) {
    for (int y=9; y<14; y++) {
      for (int x=0; x<16; x++) {
        strip.SetPixelColor(XYz(x, y), RgbColor( 0, 0, 0 ));
      }
    }
  }
  c2 = cen2;  // used for clearing

  int xx = (numb - 4) * 4;
  if (numb < 8) {
    if (numb == 0 && dn[0] == 0) {
    } else {
      if (dn[4] == -1 && numb > 3) {  // dn4==-1 equals wifi disp mode
        drwifi();
      } else {
        if (dn[numb] != ldigz) {
          Drawr(xx - cen2, 9, dn[numb]);
        }
      }
    }
  }
}

void nxfr() {  // puts data in buffer for frames on lower half
  int aTH;     // tmp variable
  if (frames[0] == 1) {
    for (int x=0; x<15; x++) {
      for (int y=9; y<14; y++) {  // clear lower area
        strip.SetPixelColor(XYz(x, y), RgbColor(0, 0, 0));
      }
    }
  }

  if (frames[0] >= 0) {  // do no increase/flip marquee or non page modes
    int as = ARSZ(frames);
    if (frames[0] > as) { frames[0] = 1; }
    int j = frames[0] + 1;

    for (int i = 0; i < as - 1; i++) {  // find next frame set to 1
      if (frames[j] == 1) {
        frames[0] = j;
        break;
      }
      j = (j > as - 2) ? 1 : j + 1;
    }
  }
  if (frames[0] == 1) {  // wif
   dn[4] = -1;
   dn[5] = ldigz;
   dn[6] = ldigz;
   dn[7] = ldigz;
  }
  if (frames[0] == 2) {  // tem
    aTH = atemp + tempadj;
    dn[4] = (int)aTH >= 0 ? ldigz : 47;  // is -/+
    dn[5] = abs((int)aTH / 10);          // th 1
    dn[6] = abs((int)aTH % 10);          // th 2
    dn[7] = 48;                          // c
  }
  if (frames[0] == 3) {  // hum
    aTH = ahumd;
    dn[4] = ldigz;
    dn[5] = abs((int)aTH / 10);  // th 1
    dn[6] = abs((int)aTH % 10);  // th 2
    dn[7] = 49;                  // %
  }
  if (frames[0] == 4) {  // d/m
    time(&now);
    localtime_r(&now, &tm);
    aTH=tm.tm_year + 1900;
    dn[4] = abs((int)aTH / 1000);  // th 1
    dn[5] = abs((int)aTH / 100%10);  // th 2
    dn[6] = abs((int)aTH / 10%10);  // th 3
    dn[7] = abs((int)aTH % 10);  // th 4
  }
  if (frames[0] == 5) {  // smk
    aTH = asmok;
    dn[4] = ldigz;
    dn[5] = aTH;
    dn[6] = 50;  // /
    dn[7] = 9;
  }
  if (frames[0] == 6) {  // aur
    aTH = aaura;
    dn[4] = ldigz;
    dn[5] = 26;                  // K
    dn[6] = 32;                  // P
    dn[7] = abs((int)aTH % 10);  // th 2
  }
  if (frames[0] == 7) {  // aur
    String tmp=moony();
    dn[4] = 32; //'P';
    dn[5] = 44; //':';
    dn[6] = strchr(diga, tmp.charAt(0)) -diga;
    dn[7] = strchr(diga, tmp.charAt(1)) -diga;
  }
}

void Dmac(int num) {  // marquee mode, draw line num
  if (num == 0) {
    // clear
    for (int x=0; x<16; x++) {
      for (int y=9; y<14; y++) {  // clear marquee area only
        strip.SetPixelColor(XYz(x, y), RgbColor(0, 0, 0));
      }
    }
    mpos[0] = mpos[0] < 0 ? 3 : mpos[0] - 1;
    mpos[1] = mpos[0] == 3 ? mpos[1] + 1 : mpos[1];
    if (mpos[1] > marquee.length() - 5) {
      mpos[1] = 0;
      awifi = WifiSig();
      marqueue = "";
    }

    if (marqueue == "") {
      time(&now);
      localtime_r(&now, &tm);
      marquee = "   ";
      if (frames[1] == 1) { marquee += " [" + String(awifi) + "/6] "; }
      String op;
      if (dec==0) { op=String((int) atemp+tempadj); }
      else { op=String(atemp+tempadj); }
      if (frames[2] == 1) { marquee += " T:" + op + "* "; }
      if (dec==0) { op=String((int) ahumd); }
      else { op=String(ahumd); }
      if (frames[3] == 1) { marquee += " H:" + op + "% "; }
      if (frames[4] == 1) { marquee += " D:" + String(tm.tm_mday) + "/" + String(tm.tm_mon + 1) + "/" + String(tm.tm_year + 1900) + " "; }
      if (frames[5] == 1) { marquee += " S:" + String(asmok) + " "; }
      if (frames[6] == 1) { marquee += " KP:" + String(aaura) + " "; }
      if (frames[7] == 1) { marquee += " P:" + moony(2) +" "; }
      marquee += "     ";
    } else {
      marquee = marqueue;
    }
  } else if (num < 6) {  // 6 chars max, 0-5

    char ma = marquee.charAt(num - 1 + mpos[1]);
    char* tx = strchr(diga, ma);
    int tmp = (tx - diga);
    if (tmp != ldigz) {
      Drawr(mpos[0] + ((num - 1) * 4), 9, tmp);
    }
  }
}

void chkaur() {
 if (sc%1800==0 && scc==2){ // check every 1h(3600)/30m(1800) from boot. 2nd frame
  if (hty[0]==""){
   maur[5]=1; // http flag
   hty[0]=chkau; // setup http call
   hty[1]="";
  }
 }
 if (maur[5]==1 && hty[1]!=""){
  aaura=hty[1].toFloat();
  maur[5]=0; // clear http flag
 }
}

void chksmk() {
 MQ2.update(); // Update data, reads voltage from analog pin
 asmok = MQ2.readSensor();
}

void loop() {
 if (rst > 1) {
    rst--; // web reset
    if (rst < 2) { ESP.restart(); }
  }
  tik++;
  if (tik >= tikr) {
    scc++; // increase sc tik counter.
    tik = -1;
    if (!dck && flm==0){
     int tmp = readFile(xHideAP, 2).toInt();
     if (tmp==2){ strip.SetPixelColor(XYz(15, 0), RgbColor( 0,128,0 )); }
     else if (tmp==1){ strip.SetPixelColor(XYz(15, 0), RgbColor( 0,0,128 )); }
     else{ strip.SetPixelColor(XYz(15, 0), RgbColor( 0,0,0 )); }
    }
   strip.Show();
   amspd[1]++;  // marquee speed
   if (amspd[1] > amspd[0]) {
    amspd[1] = 0;
   }
  }
  if (!dck && flm==0) { // dck=disable clock draw
    if (tik >= 0 && tik < 5) { Dtime(tik); }       // 0-3 clock numbers, 4 colon
    if (wfc[1]<2) { // wait for temp/humd
      if (tik > 4 && tik < 9) { Dfram(tik - 1); }
    } else {
     if (frames[0] >= 0) {                          // frame mode
      if (tik > 4 && tik < 9) { Dfram(tik - 1); }  // 4-7 numbers for frames
      if (tik == 8 && nxf == 0 && tm.tm_sec % 10 == 0) {
        nxf = 1;
        nxfr();
      }
     }
     if (frames[0] == -1 && tik > 4 && tik < 11) {  // marquee mode: 0-5
      if (amspd[1] == 0) { Dmac(tik - 5); }
     }
    }
    if (tm.tm_sec % 10 != 0) { nxf = 0; }  // unlock
  }

  if (tik==11) {
    if (wfc[0] != -1) { // Test wifi
        if (WiFi.status() != WL_CONNECTED) {
          Serial.print(F("."));
          wfc[0]++;
      } else {
        Serial.println(F(" Connected, IP address: "));
        yield();
        Serial.println(WiFi.localIP());
        time(&now);
        localtime_r(&now, &tm);
        wfc[0]= -1;
        String RDF0, RDF1, RDF2;
        RDF0 = readFile(xMyTZ, 2);
        RDF1 = readFile(xNTP1, 2);
        RDF2 = readFile(xNTP2, 2);
        configTime(RDF0.c_str(), RDF1.c_str(), RDF2.c_str());
        now = time(nullptr);
        drwifi(); strip.Show();
        while (now < 8 * 3600 * 2) {
         delay(400);
         Serial.print(F("+"));
         now = time(nullptr);
        }
      }
    }
    if (wfc[0] > 1666) {  // if wifi can't connect, turn on AP
      writeFile(xHideAP, "2");
      delay(500);
      ESP.restart(); // no wifi reboot
    }
  }
 
 if (tik==12 && flm>1){
  if (flm==2) { // lightning
   flax[1]=random(0,16); flay[1]=random(0,16);
   strip.SetPixelColor( XYz(flax[1],flay[1]), RgbColor(random(100,250),random(100,250),random(100,250)));
  }
  if (flm==3) { // rain
   
  }
  if (flm==4) { // fireplace
  }
  if (flm==5) { // Random
   flax[1]=random(0,16); flay[1]=random(0,16);
   strip.SetPixelColor( XYz(flax[1],flay[1]), RgbColor(random(100,250),random(100,250),random(100,250)));
  }
  if (flm==5) { // spiral
   flax[0]+= flax[1];
   flay[0]+= flay[1];
   if (flax[0]>15) {
    flay[0]++; flax[0]=0;
   }
   
   strip.SetPixelColor( XYz(flax[0],flay[0]), RgbColor(random(100,250),random(100,250),random(100,250)));
  }

 }

  if (tik == 16) { chkaur(); } // check aurora
  if (flm==0){
   if (tik==13 && scc==3 && sc%10==0){ // aurora low bar every 10s, frame3
   int t=(int)aaura; int xy=0;
   for (int x = 1; x <= 9; x++) {
    strip.SetPixelColor(XYz(x+2, 15), RgbColor( 0,0,0 )); // clear old KP bar
   }
   if (t>hkp){ // draw if enabled
    for (int x = 1; x <= t; x++) { // only draw bar colours as needed
     xy= ((9-t)/2)+2; // center dots
     strip.SetPixelColor(XYz(x+xy, 15), RgbColor( arck[t-1][0],arck[t-1][1],arck[t-1][2] ));
    }
   }
  }
  if (tik==14 && mpos[1]==0 && hty[0]!="") { // HTTP, wait for marquee to be at 0
   if (WiFi.status() == WL_CONNECTED) {
    Serial.print("h0:");
    Serial.println(hty[0]);
    String url=hty[0];
    String o[5]={"","","","","1"}; 
    for (int i = 0; i < url.length(); i++) {
     if (o[4]=="1"){ // https
       if (url[i] == ':') { o[4]="2"; }
       else o[0] += url[i];
     }
     if (o[4]=="2" && url[i] != ':'){ // ://
      if (url[i] != '/') { o[4]="3"; }
     }
     if (o[4]=="3"){ // domain
       if (url[i] == ':') o[4]="4";
       else {
        if (url[i] == '/') o[4]="5";
        else o[1] += url[i];
       }
     }
     if (o[4]=="4"){ // port?
       if (url[i] == '/') o[4]="5";
       else o[2] += url[i];
     }
     if (o[4]=="5"){ // path
      o[3] += url[i];
     }
    }
    String line; WiFiClientSecure client; client.setInsecure();
                         
    if (!client.connect(o[1].c_str(), 443)) {
     Serial.println(F("Connection failed"));
    } else {
     if (hty[1]=="") {
      client.print(String("GET ") + o[3] + " HTTP/1.1\r\nHost: " + o[1] + "\r\nUser-Agent: ESP32\r\nConnection: close\r\n\r\n");
     } else {
      client.stop();
      hty[0]="";
     }
  
     line = client.readString();
     hty[1]=line.substring(line.indexOf("\r\n\r\n"));
     hty[0] = "";
     Serial.print("h1:");
     Serial.println(hty[1]);
    }
   }
  }
   if (tik == 15 && wfc[1]>1) { alarm(); } // make sure sensors are set
   
  } else { yield(); } // skip above if flm mode
    
  if (tik == 17 && pn[0] > -1) { play(); }  // play note
  if (tik == 18 && sc>60 && sc%4==0 && scc==1) { chksmk(); } // check smoke after 60s and every 2 secs
  if (tik == 19) {  // temp/humd -> alternate checks every 10s
    tik = 25;
    if (tht == 2 && (tm.tm_sec == 0 || tm.tm_sec == 20 || tm.tm_sec == 40)) {
      tht = 1; if (wfc[1]<2) { wfc[1]++; }
      yield();
      aht_humd->getEvent(&humd);
      ahumd = humd.relative_humidity;
    } else if (tht == 1 && (tm.tm_sec == 10 || tm.tm_sec == 30 || tm.tm_sec == 50)) {
      tht = 2; if (wfc[1]<2) { wfc[1]++; }
      yield();
      aht_temp->getEvent(&temp);
      atemp = temp.temperature;
    }
  }
  if (tik >= 20 && tik <= tikr) { yield(); }
}
