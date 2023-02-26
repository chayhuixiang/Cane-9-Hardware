/***************************************************
 DFPlayer - A Mini MP3 Player For Arduino
 <https://www.dfrobot.com/index.php?route=product/product&product_id=1121>
 
 ***************************************************
 This example shows the all the function of library for DFPlayer.
 
 Created 2016-12-07
 By [Angelo qiao](Angelo.qiao@dfrobot.com)
 
 GNU Lesser General Public License.
 See <http://www.gnu.org/licenses/> for details.
 All above must be included in any redistribution
 ****************************************************/

/***********Notice and Trouble shooting***************
 1.Connection and Diagram can be found here
<https://www.dfrobot.com/wiki/index.php/DFPlayer_Mini_SKU:DFR0299#Connection_Diagram>
 2.This code is tested on ESP32 boards (changes by pcbreflux).
 ****************************************************/

#include <Arduino.h>
//#include <SoftwareSerial.h>
//#include "AltSoftSerial.h"
#include "DFRobotDFPlayerMini.h"

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "Credentials.h"

#include <random>

//SoftwareSerial mySoftwareSerial(16, 17); // RX, TX
//AltSoftSerial mySoftwareSerial(16, 17); // RX, TX
HardwareSerial mySoftwareSerial(1);
DFRobotDFPlayerMini myDFPlayer;
bool playing = false;
const String patientId = "iZJE99WIH4VQGzWptmDxpV3skpv1";

const double LATITUDE = 1.354381849970657;
const double LONGITUDE = 103.68785764576984;

std::default_random_engine generator;
std::normal_distribution<double> latitude_distribution(LATITUDE, 0.0001);
std::normal_distribution<double> longitude_distribution(LONGITUDE, 0.0001);

//const char* ssid = "";
//const char* password = "";
String serverName = "http://192.168.1.116:3000";

double generateLatitude();
double generateLongitude();

bool testAndSet(bool* playing);

void setup()
{
  mySoftwareSerial.begin(9600, SERIAL_8N1, 13, 12);  // speed, type, RX, TX
  Serial.begin(115200);
  
  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));
  
  if (!myDFPlayer.begin(mySoftwareSerial)) {  //Use softwareSerial to communicate with mp3.
    
    Serial.println(myDFPlayer.readType(),HEX);
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while(true);
  }
  Serial.println(F("DFPlayer Mini online."));
  
  myDFPlayer.setTimeOut(500); //Set serial communictaion time out 500ms
  
  //----Set volume----
  myDFPlayer.volume(10);  //Set volume value (0~30).
  myDFPlayer.EQ(DFPLAYER_EQ_NORMAL);
  myDFPlayer.outputDevice(DFPLAYER_DEVICE_SD);

  int delayms=100;

  Serial.begin(115200);
  WiFi.begin(ssid, password);
  Serial.println("Connecting");
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");  
  }
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());
  
  //----Mp3 play----
  /*
  Serial.println(F("myDFPlayer.next()"));
  myDFPlayer.next();  //Play next mp3
  delay(delayms);
  Serial.println(F("myDFPlayer.previous()"));
  myDFPlayer.previous();  //Play previous mp3
  delay(delayms);
  Serial.println(F("myDFPlayer.play(1)"));
  myDFPlayer.play(1);  //Play the first mp3
  delay(delayms);
  Serial.println(F("myDFPlayer.loop(1)"));
  myDFPlayer.loop(1);  //Loop the first mp3
  delay(delayms);
  Serial.println(F("myDFPlayer.pause()"));
  myDFPlayer.pause();  //pause the mp3
  delay(delayms);
  Serial.println(F("myDFPlayer.start()"));
  myDFPlayer.start();  //start the mp3 from the pause
  delay(delayms);
  Serial.println(F("myDFPlayer.playFolder(15, 4)"));
  myDFPlayer.playFolder(15, 4);  //play specific mp3 in SD:/15/004.mp3; Folder Name(1~99); File Name(1~255)
  delay(delayms);
  Serial.println(F("myDFPlayer.enableLoopAll()"));
  myDFPlayer.enableLoopAll(); //loop all mp3 files.
  delay(delayms);
  Serial.println(F("myDFPlayer.disableLoopAll()"));
  myDFPlayer.disableLoopAll(); //stop loop all mp3 files.
  delay(delayms);
  Serial.println(F("myDFPlayer.playMp3Folder(4)"));
  myDFPlayer.playMp3Folder(4); //play specific mp3 in SD:/MP3/0004.mp3; File Name(0~65535)
  delay(delayms);
  Serial.println(F("myDFPlayer.advertise(3)"));
  myDFPlayer.advertise(3); //advertise specific mp3 in SD:/ADVERT/0003.mp3; File Name(0~65535)
  delay(delayms);
  Serial.println(F("myDFPlayer.stopAdvertise()"));
  myDFPlayer.stopAdvertise(); //stop advertise
  delay(delayms);
  Serial.println(F("myDFPlayer.playLargeFolder(2,999)"));
  myDFPlayer.playLargeFolder(2, 999); //play specific mp3 in SD:/02/004.mp3; Folder Name(1~10); File Name(1~1000)
  delay(delayms);
  Serial.println(F("myDFPlayer.loopFolder(5)"));
  myDFPlayer.loopFolder(5); //loop all mp3 files in folder SD:/05.
  delay(delayms);
  Serial.println(F("myDFPlayer.randomAll()"));
  myDFPlayer.randomAll(); //Random play all the mp3.
  delay(delayms);
  Serial.println(F("myDFPlayer.enableLoop()"));
  myDFPlayer.enableLoop(); //enable loop.
  delay(delayms);
  Serial.println(F("myDFPlayer.disableLoop()"));
  myDFPlayer.disableLoop(); //disable loop.
  delay(delayms);
  */
}

void loop() {
  DynamicJsonDocument doc(1024);
  doc["patientId"] = patientId;
  doc["lat"] = String(generateLatitude(), 10);
  doc["long"] = String(generateLongitude(), 10);
  doc["registrationToken"] = "123";
  String jsonString;

  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;

    String serverPath = serverName + "/location/update";
    http.begin(serverPath.c_str());
    http.addHeader("Content-Type", "application/json");
    serializeJson(doc, jsonString);

    int httpResponseCode = http.POST(jsonString);

    if (httpResponseCode > 0) {
      String payload = http.getString();
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);
      Serial.print("Payload: ");
      Serial.println(payload);
      // Sound Alert
      if (payload == "1" && !testAndSet(&playing)) {
        myDFPlayer.loop(1);
        Serial.println("Playing Audio...");
      } else if (payload == "0") {
        myDFPlayer.stop();
        playing = false;
      }
    }
  }
  
  delay(5000);  
}

bool testAndSet(bool* playing) {
  bool prev = *playing;
  *playing = true;
  return prev;
}

double generateLatitude() {
  double generatedLatitude = latitude_distribution(generator);
  return generatedLatitude;
}

double generateLongitude() {
  double generatedLongitude = longitude_distribution(generator);
  return generatedLongitude;  
}
