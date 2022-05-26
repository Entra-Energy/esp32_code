/*
 * Project SM_SIM800L_ESP32
 * Description:
 * Author: Georgi Manev
 * Date:09.03.2022
 */



// Please select the corresponding model
#define SIM800L_IP5306_VERSION_20190610
#define TINY_GSM_MODEM_SIM800
#include <ArduinoJson.h>
#include <Wire.h>
#include <TinyGsmClient.h>

#include "EmonLib.h"

#include <vector>
#include <sstream>
#include <locale>
#include <iomanip>
#include <map>
#include <string>
#include <cstdio>
#include <algorithm>

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial
// Set serial for AT commands
#define SerialAT Serial1

#define TINY_GSM_DEBUG SerialMon

// Your GPRS credentials, if any
const char apn[] = "internet.vivacom.bg"; // APN (example: internet.vodafone.pt) use https://wiki.apnchanger.org
const char gprsUser[] = "";
const char gprsPass[] = "";

// SIM card PIN (leave empty, if not defined)
const char simPIN[]   = ""; 

// set GSM PIN, if any
#define GSM_PIN ""

#ifdef DUMP_AT_COMMANDS
  #include <StreamDebugger.h>
  StreamDebugger debugger(SerialAT, SerialMon);
  TinyGsm modem(debugger);
#else
  TinyGsm modem(SerialAT);
#endif

#include <PubSubClient.h>

TinyGsmClient client(modem);
PubSubClient mqtt(client);


uint32_t lastReconnectAttempt = 0;

// MQTT details
const char* broker = "159.89.103.242";                    // Public IP address or domain name
const char* mqttUsername = "REPLACE_WITH_YOUR_MQTT_USER";  // MQTT username
const char* mqttPassword = "REPLACE_WITH_YOUR_MQTT_PASS";  // MQTT password

String reset;
int init_delay = 1;
int start_measure = 0;

int cali = 1;
int ver = 3;
int timer = 0;
long powerCorrection;
EnergyMonitor emon1;  
EnergyMonitor emon2;
EnergyMonitor emon3; 
float blynkPublish;
float one_min_measure;

int nextPeriod;
float average;
long interval = 2000;
long previousMillis = 0; 

float accumulatePow;
float hourBegining;
float hourConsumption;
float dayBegining;
float dayConsumption;
float monthBegining;
float monthConsumption;

float currPriceH;
float currPriceD;
float currPriceM;
float currPrice;

long currentMillisSend = 0;
long previousMillisSend = 0;
long previousMillisMeasureDelay = 0;
long intervalSend = 5000;
long intervalSendBlynk = 10000;
long previousMillisSendBlynk = 0;
long previousMillisCorrection = 0;
int reportFreq;

int currMin;
int currHour;
int currDay;
int currMonth;
int blynkGridButton;
int16_t currYear;
int16_t prevYear;
uint8_t prevMin;
uint8_t prevDay;
uint8_t prevHour;
uint8_t prevMonth;

#define NUMSAMPLES 5
float tenSec[NUMSAMPLES];


std::map<String, float>powerMap; 
std::map<String, float>pingMap;
std::vector< float > measurements;
std::vector< float > measure_delay;


int trigger = 0;
char data[80]; 
char pingData[80];

double Irms1; 
double Irms2;
double Irms3; 

int j;

String stampRecieved;
String corrRecieved;

StaticJsonDocument<200>parser; 
StaticJsonDocument<200>correct_parser;

const char* deviceID = "sm-0004";

//Topic to receive data from db (for db robustness)
const char* timestampStr = "/timestamp";
String timestampReceived = String(deviceID)+String(timestampStr);
const char* timestampRe = timestampReceived.c_str();

//Topic for reset calibration
const char* resetStr = "/reset";
String resetReceived = String(deviceID)+String(resetStr);
const char* resetRe = resetReceived.c_str();

//Topic for time correction
const char* correctionStr = "/correction";
String correctionReceived = String(deviceID)+String(correctionStr);
const char* correctionRe = correctionReceived.c_str();

//Topic to receive calibration
String calibrationKoeff;
const char* cal = "cali/";
String calKoeffTopic = String(cal) + String(deviceID);
const char* calibration = calKoeffTopic.c_str();
float cali_koeff = 1;

String adj;
const char* adjust = "/adjustment";
String adjMeasurement = String(deviceID)+String(adjust);
const char* adjTopic = adjMeasurement.c_str();

//Topic for update status and dash table every 5s
const char* dashCh = "ping/";
String pingDev = String(dashCh)+String(deviceID);
const char* ping = pingDev.c_str();

//Topic for send data
const char* dataChar = "data/";
const char* fifteenToChar = "/fifteen";
String dataSend = String(dataChar)+String(deviceID)+String(fifteenToChar);
const char* sendFifteen = dataSend.c_str();

//Topic for send data for db check
const char* er = "error/check/";
String error = String(er)+String(deviceID);
const char* errorSendDash = error.c_str();

//Blynk related topics for sending the consumption
const char* blh = "blynkHourConsumption/";
String blynkh = String(blh)+String(deviceID);
const char* blynkHourCons = blynkh.c_str();

const char* bld = "blynkDayConsumption/";
String blynkd = String(bld)+String(deviceID);
const char* blynkDayCons = blynkd.c_str();

const char* blM = "blynkMonthConsumption/";
String blynkm = String(blM)+String(deviceID);
const char* blynkMonthCons = blynkm.c_str();




//callback function for receiving mqtt data on different topics
void mqttCallback(char* topic, byte* message, unsigned int len) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String p = "";
  for (int i=0;i<len;i++)
  {
    p += (char)message[i];        
  }      
  Serial.println();    
  //system topic
  if (String(topic) == "RED")
  {       
      mqtt.publish("size_plusOnemin", "ECHO ECHO");               
  }

  if (String(topic) == calibration)
  {       
    calibrationKoeff = String(p);          
  }
  
  if (String(topic) == String(deviceID)+"/adjustment")
  {       
      adj = String(p);   
  }

  //receive data from db and check if exist into device "array" (db robustness related)
  if (String(topic) == String(deviceID)+"/timestamp")
  {
    stampRecieved = String(p);
    DeserializationError error = deserializeJson(parser, stampRecieved);
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      return;
    }
    String strValue = parser["time"];
    float powValue = parser["pow"];
    mqtt.publish("parserPow", (char*) strValue.c_str());
    std::map<String,float>::iterator it;
    it = powerMap.find(strValue);
    if (it != powerMap.end())
    {
      powerMap.erase (it);  
      trigger = 1;   
    }              
  }
  //if received power correction and period for that correction
  if (String(topic) == String(deviceID)+"/correction")
  {
    corrRecieved = String(p);
    DeserializationError error = deserializeJson(correct_parser, corrRecieved);
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      return;
    }
    powerCorrection = correct_parser["power"];      
    timer = correct_parser["timer"];      
  }
  //if calibration reset is received
  if (String(topic) == String(deviceID)+"/reset")
  {
    reset = String(p);
    Serial.print("RESET!!");
    if (reset)
    {
      cali_koeff = 1;
    }
  }
}


// initialize and check mqtt connection, subscribe to topics
boolean mqttConnect() 
{
  SerialMon.print("Connecting to ");
  SerialMon.print(broker);
  boolean status = mqtt.connect(deviceID);
  if (status == false) {
    SerialMon.println("fail");
    ESP.restart();
    return false;
  }
  SerialMon.println("success");
  mqtt.subscribe("meter/ibexIn");
  mqtt.subscribe("correction");
  mqtt.subscribe(timestampRe);
  mqtt.subscribe(correctionRe);
  mqtt.subscribe(resetRe);
  mqtt.subscribe(calibration);
  mqtt.subscribe("RED");
  return mqtt.connected();
}


void setup() { 
  SerialMon.begin(115200);
  delay(10);
  
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(23, OUTPUT);

  digitalWrite(4, LOW);
  digitalWrite(5, HIGH);
  digitalWrite(23, HIGH);

  // Set modem reset, enable, power pins
  SerialAT.begin(115200, SERIAL_8N1, 26, 27); 
  SerialMon.println("Wait...");
  delay(6000);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");
  //modem.restart();
  modem.init();

  SerialMon.print("Waiting for network...");
  if (!modem.waitForNetwork()) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" success");
  if (modem.isNetworkConnected()) {
    SerialMon.println("Network connected");
  }  

  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);

  // Unlock your SIM card with a PIN if needed
  if ( GSM_PIN && modem.getSimStatus() != 3 ) {
    modem.simUnlock(GSM_PIN);
  }
  SerialMon.print("Connecting to APN: ");
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println(" fail");
    ESP.restart();
  }
  else {
    SerialMon.println(" OK");
  }
  
  if (modem.isGprsConnected()) {
    SerialMon.println("GPRS connected");
    timeConvert();
    prevHour = currHour;
    prevDay = currDay;
    prevMonth = currMonth;
    nextPeriod = currMin + 2;
  }  
  // MQTT Broker setup
  mqtt.setServer(broker, 1883);
  mqtt.setCallback(mqttCallback);

  //initialize analog pins and constant for measurements
  emon1.current(12, 74.07); //27 ohm 74.07 (50mA)
  emon2.current(13, 74.07);
  emon3.current(15, 74.07);
  analogReadResolution(ADC_BITS);
  calibrationKoeff = "1";
  adj= "1";
 
  //connect to mqtt broker
  if (!mqtt.connected()) {
  SerialMon.println("=== MQTT NOT CONNECTED ===");
  // Reconnect every 10 seconds
  uint32_t t = millis();
  if (t - lastReconnectAttempt > 10000L) {
    lastReconnectAttempt = t;
    if (mqttConnect()) {
      lastReconnectAttempt = 0;
    }
  }
  delay(100);
  return;
  }    
  powerMap.clear();   
}

// send data every 10s
void blynkUpdate()
{      
  struct tm t1;
  String timestamp = timeGet(t1);
  pingMap.insert(std::pair<String, float>(timestamp,blynkPublish));
  String timeSt = "\"timestamp\": " + String(pingMap.begin()->first);
  String powerSt = "\"power\": " + String(pingMap.begin()->second);
  String jObj = timeSt +','+ powerSt;
  String payload = "{ \"payload\": {" + jObj + "}}";
  payload.toCharArray(pingData, (payload.length() + 1));
  mqtt.publish(ping, pingData);
  pingMap.clear();       
}

//measurements
void measure()
{  
  unsigned long currentMillis = millis();  
    
  if(currentMillis - previousMillis > interval)
  { 
    Irms1 = emon1.calcIrms(2307);//2307 after calibration for 50ma 
    if (Irms1 <= 0.2)
    {
      Irms1 = 0;
    }       
    Irms2 = emon2.calcIrms(2307);
    if (Irms2 <= 0.2)
    {
      Irms2 = 0;
    } 
    Irms3 = emon3.calcIrms(2307);
    if (Irms3 <= 0.2)
    {
      Irms3 = 0;
    } 
    double Irms = Irms1 + Irms2 + Irms3;
    double power = Irms*230; 

    if (init_delay = 1)
    { 
      measure_delay.push_back(power);//start to keep track on initial measurements first ones have peaks         
    }
      
    int delay_size = measure_delay.size();
    Serial.println(delay_size);
    if (delay_size > 30 || start_measure == 1) //Start the real measurements after first 30        
    {
      tenSec[j] = power;         
      j++;        
      if (j > 4)
      {
        for (int k=0; k < 5; k++) 
        {          
          average += tenSec[k];           
        }          
        average /= 5;

        float devCalibrate = calibrationKoeff.toFloat();
        
        if (devCalibrate != 1)
        {                
          cali_koeff = devCalibrate/average;  
          Serial.println(cali_koeff);
          calibrationKoeff = "1";
        }
        
        average = average*cali_koeff + powerCorrection; 
                    
        blynkPublish = average;
        Serial.println(blynkPublish);
        measurements.push_back(blynkPublish);//add every 10s                
        average = 0;
        j = 0;
      }   
      init_delay = 0;
      if (delay_size > 0)
      {
        measure_delay.clear();
      }
      start_measure = 1;       
    }
    previousMillis = currentMillis;
  }   
}

//get the time, convert and prepear it
void timeConvert (){
  String time = modem.getGSMDateTime(DATE_FULL);
  int splitT = time.indexOf(",");
  int splitY = time.indexOf("/");
  String timeStamp = time.substring(splitT+1, time.length());
  String timeStampY = time.substring(splitY, 0);
  currYear = timeStampY.toInt();
  String currHourStr = timeStamp.substring(0,2);
  currHour = currHourStr.toInt();

  int splitM = timeStamp.indexOf(":");
  String timeStampM = timeStamp.substring(splitM+1, timeStamp.length());
  String currMinString = timeStampM.substring(0,2);
  currMin = currMinString.toInt();
  String timeStampMonth = time.substring(splitY+1, time.length());
  String currMonthString = timeStampMonth.substring(0,2);
  currMonth = currMonthString.toInt();
  int splitDay = timeStampMonth.indexOf("/");
  String timeStampDay = timeStampMonth.substring(splitDay+1, timeStampMonth.length());
  String currDayString = timeStampDay.substring(0,2);
  currDay = currDayString.toInt();
}
//time as timestamp
String timeGet(struct tm t)
{
  t = {0};
  t.tm_year = (currYear+2000) - 1900;
  t.tm_mon = currMonth - 1;
  t.tm_mday = currDay;
  t.tm_hour = currHour;
  t.tm_min = currMin;
  t.tm_sec = 00;
  time_t timeSinceEpoch = mktime(&t);    
  int stamp = int(timeSinceEpoch);
  String timestamp = String(stamp);
  return timestamp;    
} 

struct timestampPower 
{
  //send the measurements or all accumulated data that is not be sent for some reasons 
  void sendData()
  {
    for (std::map<String, float>::iterator it = powerMap.begin(); it != powerMap.end(); ++it) 
    {
      String timeSt = "\"timestamp\": " + String((*it).first);
      String powerSt = "\"power\": " + String((*it).second);
      String jObj = timeSt +','+ powerSt;
      String payload = "{ \"payload\": {" + jObj + "}}";
      payload.toCharArray(data, (payload.length() + 1));
      mqtt.publish(sendFifteen, data);  
    }
  }  
  //send the last timestamp-power pair from the accumulated measurements
  void checkDb(){  
    if(!powerMap.empty())
    {
      String mapKey = (--powerMap.end())->first;
      float mapValue = (--powerMap.end())->second;
      String timeSt = "\"timestamp\": " + mapKey;
      String powerSt = "\"power\": " + String(mapValue);
      String jObj = timeSt +','+ powerSt;
      String payload = "{ \"payload\": {" + jObj + "}}";
      payload.toCharArray(data, (payload.length() + 1));
      mqtt.publish(errorSendDash, data);
    }
  }
}tp;

void loop() {

  timeConvert();
  if (!mqtt.connected()) {
    SerialMon.println("=== MQTT NOT CONNECTED ===");
    // Reconnect every 10 seconds
    uint32_t t = millis();
    if (t - lastReconnectAttempt > 10000L) {
      lastReconnectAttempt = t;
    if (mqttConnect()) {
      lastReconnectAttempt = 0;
    }
  }
    delay(100);
    return;
  } 
       
  unsigned long currentMillisSend = millis();

  if(currentMillisSend - previousMillisSend > intervalSend)//db robustness related
  {  
    if (trigger == 1 && powerMap.size() > 0)
    {
      tp.checkDb();
      trigger = 0;
    }
    previousMillisSend = currentMillisSend;
  }        

  if(currentMillisSend - previousMillisSendBlynk > intervalSendBlynk)//update dash table every 10s
  {        
      blynkUpdate();
      previousMillisSendBlynk = currentMillisSend;
  } 

  if(currentMillisSend - previousMillisCorrection > 1000)//power correction for given period
  {     
    if (timer > 0)
    {
      timer = timer - 1;               
    }
    else {
      powerCorrection = 0;
    }   
    previousMillisCorrection = currentMillisSend;    

  }
  

  if (currMin != prevMin)   
  {   
    accumulatePow += blynkPublish;        
    hourConsumption = (accumulatePow - hourBegining)/60;    
    char hourConsumptionBlynk[8];
    dtostrf(hourConsumption, 1, 2, hourConsumptionBlynk);
    mqtt.publish(blynkHourCons, hourConsumptionBlynk); 

    dayConsumption = (accumulatePow - dayBegining )/60;    
    char dayConsumptionBlynk[8];
    dtostrf(dayConsumption, 1, 2, dayConsumptionBlynk);
    mqtt.publish(blynkDayCons, dayConsumptionBlynk);

    monthConsumption = (accumulatePow - monthBegining)/60; 
    char monthConsumptionBlynk[8];
    dtostrf(monthConsumption, 1, 2, monthConsumptionBlynk);
    mqtt.publish(blynkMonthCons, monthConsumptionBlynk);

    //average for min
    one_min_measure = accumulate( measurements.begin(), measurements.end(), 0.0)/measurements.size();
    measurements.clear();
    struct tm t0;
    String timestamp = timeGet(t0); 
    if (!isnan(one_min_measure) && !isinf(one_min_measure))
    {
      powerMap.insert(std::pair<String, float>(timestamp,one_min_measure));
      tp.sendData();
    }
    prevMin = currMin;  
    //set time for send the data for checking db
    if (nextPeriod >= 60)
    {
        nextPeriod = 0;
    }
        
  }
  //send data for checking if exist into the db
  if (currMin == nextPeriod)
  {    
    int err_size = powerMap.size();
    String err_string = String(err_size);                
    mqtt.publish("boron", (char*) err_string.c_str());//just send the size of the data that has not be sent for some reasons

    if (err_size > 1000)
    {
      powerMap.clear();
    }
                      
    tp.checkDb();
    trigger = 1;
    nextPeriod += 2;
  }  
  if (currHour != prevHour)
  {   
      hourBegining = accumulatePow;
      prevHour = currHour;
  }
  if (prevDay != currDay)
  {
      dayBegining = accumulatePow;
      prevDay = currDay;
  }
  if (prevMonth != currMonth)
  {
      monthBegining = accumulatePow;
      prevMonth = currMonth;
  }   
  measure();   
  mqtt.loop();    
       
}
