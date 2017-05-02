/*
    hamster_wheel.ino
    Copyright (C) 2017  Sarah Reitinger

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/* 
 * Notes:
 * 
 *    WIFI:
 *        After you start the ESP, it will try to connect to WiFi. If it fails (which happens when you start it the very first 
 *        time a/o no or an incorrect WiFi SSID/password are stored yet) it starts in Access Point (AP) mode. 
 *        While in AP mode, connect to it (see WIFI_CONFIG_AP_NAME and WIFI_CONFIG_AP_PW) then open a browser to the gateway IP (default 192.168.4.1), 
 *        configure wifi, save and it should reboot and connect.
 *        
 *    MQTT:
 *        Connection settings are currently hardcoded. You may, however, want to make those configurable via the on-demand configuration
 *        portal of WiFiManger -> see https://github.com/tzapu/WiFiManager for details
 *        
 *    Sensor source:
 *        mostly taken from https://www.arduino.cc/en/Tutorial/Debounce
 */

#include <ESP8266WiFi.h>          // v1.0.0   https://github.com/esp8266/Arduino
#include <WiFiManager.h>          // v0.12.0  https://github.com/tzapu/WiFiManager
#include <PubSubClient.h>         // v2.6.0   https://github.com/knolleary/PubSubClient

#define SEC_1                   1000
#define SEC_5                   5000
#define SEC_10                  10000
#define SEC_15                  15000
#define MIN_1                   1*60*1000
#define HOURS_5                 5*60*60*1000

#define WIFI_CONFIG_AP_NAME     "wheel"
#define WIFI_CONFIG_AP_PW       "hamster"

#define BAUDRATE                115200
#define KM_TO_CM                100000 // 1 km as cm
#define CM_MS__AS__KM_H         36 // 1 km/h in cm/ms

#define WHEEL_DEBUG                       false
#define WHEEL_DEBUG_VERBOSE               false
#define WHEEL_SENSOR_PIN                  4
#define WHEEL_LED_PIN                     LED_BUILTIN // let's use the on-board LED pin of the NodeMCU
#define WHEEL_NAN                         -1
#define WHEEL_DEBOUNCE_DELAY_MILLIS       10 // the debounce time; increase if the output flickers
#define WHEEL_MIN_VALID_ROTATION_MILLIS   800 // =^ 4.23km/h
#define WHEEL_MAX_VALID_ROTATION_MILLIS   4500 // =^ 0.75km/h
#define WHEEL_NIGHT_RESET                 HOURS_5
#define WHEEL_CIRCUMFERENCE               94 //cm
#define WHEEL_DISTANCE_CM                 WHEEL_CIRCUMFERENCE
#define WHEEL_DISTANCE_KM                 WHEEL_DISTANCE_CM / KM_TO_CM

#define MQTT_ID                         "wheel #1"
#define MQTT_KEEPALIVE                  60 // seconds -> this is a constant of PubSubClient which we will override
#define MQTT_RECONNECT_TIMEOUT          MIN_1
#define MQTT_SERVER                     "mqtt.example.org" // please change to a valid MQTT broker
#define MQTT_PORT                       1883
#define MQTT_TOPIC                      "hamster/wheel/"
#define MQTT_HELLO                      MQTT_TOPIC "hello-world"
#define MQTT_SPEED                      MQTT_TOPIC "rotation/speed/km-h"
#define MQTT_RPM                        MQTT_TOPIC "rotation/speed/rpm"
#define MQTT_DURATION                   MQTT_TOPIC "rotation/duration/sec"
#define MQTT_LAP_DISTANCE               MQTT_TOPIC "lap/distance/km"
#define MQTT_LAP_ROTATION_CNT           MQTT_TOPIC "lap/rotation/cnt"
#define MQTT_NIGHT_DISTANCE             MQTT_TOPIC "night/distance/km"
#define MQTT_NIGHT_ROTATION_CNT         MQTT_TOPIC "night/rotation/cnt"
#define MQTT_NIGHT_LAP_CNT              MQTT_TOPIC "night/lap/cnt"




WiFiClient wc;
PubSubClient mqttClient;

// mqtt stats
unsigned long _mqttLastConnect = -1; // the last time an MQTT connection was tried to establish

// sensor stats
unsigned long _lastDebounceTime = -1;  // the last time the input pin was toggled
int _sensorState = HIGH;
int _lastSensorIn = _sensorState;
unsigned long _lastSensorChange = -1; // the last time the sensor state changed

// rotation stats
unsigned long _lastRotation = -1; // the last time a valid rotation was registered
int _lapRotationCnt = WHEEL_NAN;
float _lapDistance = 0;
int _nightRotationCnt = 0;
int _nightLapCnt = 0;
float _nightDistance = 0;


void log(const String message){
  if (Serial) {
    Serial.println(message);
  }
}

void debugLog(const String message){
  if (WHEEL_DEBUG && Serial) {
    Serial.print("[DEBUG]  ");
    Serial.println(message);
  }
}

void debugVerboseLog(const String message){
  if (WHEEL_DEBUG_VERBOSE && Serial) {
    Serial.print("[VDEBUG] ");
    Serial.println(message);
  }
}

void logSSID() {
  String ssid = "ssid: "+WiFi.SSID();
  log(ssid);
}

bool wifiConnected() {
  return WiFi.status() == WL_CONNECTED;
}

bool mqttConnected() {
  return mqttClient.connected();
}

void mqttConnect() {
  log("Attempting MQTT connection...");
  if (mqttClient.connect(MQTT_ID)) {
    log("...connected");
    // Once connected, publish an announcement...
    mqttClient.publish(MQTT_HELLO, MQTT_ID);
  } else {
    log("...failed, client state="+String(mqttClient.state()));
  }
  _mqttLastConnect = millis();
}

void mqttReconnect() {
  long uptime = millis();
  if (_mqttLastConnect + MQTT_RECONNECT_TIMEOUT > uptime) {
    // skipping mqtt reconnect as not due yet
    return;
  }
  mqttConnect();
}

void mqttPublish(const char* topic, const String msg) {
  mqttPublish(topic, msg, false);
}

void mqttPublish(const char* topic, const String msg, bool retain) {
  if (!mqttConnected()) {
    // at least log as debug info 
    String debugMsg = String(topic) + ": " + msg;
    debugLog(debugMsg);
    return;
  }
  
  int len = msg.length() + 1;
  char msgBuf[len];
  msg.toCharArray(msgBuf, len);
  mqttClient.publish(topic, msgBuf, retain);
}

void updateLed(const int status) {
  digitalWrite(WHEEL_LED_PIN, status);
}

void resetAllCounters() {
  resetLapCounters();
  resetNightCounters();
}

void resetNightCounters() {
  _nightRotationCnt = 0;
  _nightLapCnt = 0;
  _nightDistance = 0;
}

void resetLapCounters() {
  _lapRotationCnt = WHEEL_NAN;
  _lapDistance = 0;
}

bool checkForNightTimeout(const unsigned long lastSensorChange) {
  if (lastSensorChange >= WHEEL_NIGHT_RESET) {
    resetAllCounters();
  }
}

bool evaluateRotation(const unsigned long uptime) {
  long durationMillis = uptime - _lastSensorChange;
  bool exceededMin = durationMillis <= WHEEL_MIN_VALID_ROTATION_MILLIS;
  bool exceededMax = durationMillis >= WHEEL_MAX_VALID_ROTATION_MILLIS;
  
  bool validRotation = !exceededMin && !exceededMax;
  if (validRotation) {
    // only count if rotation took a certain amount of time otherwise the wheel's just swinging
    logRotation(uptime);     
  } else {
    resetLapCounters();
    if (exceededMin) {
      debugLog("invalid rotation - too fast (not counted)");            
    } else {
      debugLog("invalid rotation - too slow (not counted)");            
    }
  }
  return validRotation;
}

void logRotation(const unsigned long uptime) {
  int prevRotationCnt = _lapRotationCnt;
  long durationMillis = uptime - _lastRotation;

  _lastRotation = uptime; 
  _lapRotationCnt++;
  
  // do not count yet -> just started
  if (prevRotationCnt == WHEEL_NAN) {
    debugLog("ignore very first rotation of lap");
    return;
  }

  float km = (float) WHEEL_DISTANCE_KM;
  float durationSec = (float) durationMillis / (float) SEC_1;
  float cmPerMillis = (float) WHEEL_DISTANCE_CM / (float) durationMillis;
  float kmPerHours = cmPerMillis * (float) CM_MS__AS__KM_H;
  float rpm = (float) MIN_1 / (float) durationMillis;
  bool publishLapCnt = false;

  // update counters
  _nightDistance += km;
  _lapDistance += km;
  _nightRotationCnt++;
  if (_nightRotationCnt == 1) {
    log("new night");
  }
  if (_lapRotationCnt == 1) {
    publishLapCnt = true;
    _nightLapCnt++;
    log("new lap");
  } 
  
  // rotation stats
  mqttPublish(MQTT_DURATION, String(durationSec));
  mqttPublish(MQTT_SPEED, String(kmPerHours));
  mqttPublish(MQTT_RPM, String(rpm));
  
  // lap stats
  mqttPublish(MQTT_LAP_ROTATION_CNT, String(_lapRotationCnt));
  mqttPublish(MQTT_LAP_DISTANCE, String(_lapDistance));
  
  // night stats
  mqttPublish(MQTT_NIGHT_ROTATION_CNT, String(_nightRotationCnt));
  if (publishLapCnt) {
    mqttPublish(MQTT_NIGHT_LAP_CNT, String(_nightLapCnt));
  }
  mqttPublish(MQTT_NIGHT_DISTANCE, String(_nightDistance));
  
  String logMsg = "rotation #"+String(_nightRotationCnt)+" ("+kmPerHours+" km/h, total: "+String(_nightDistance)+" km)";
  log(logMsg);
}



void readSensor() {
  long uptime = millis();
  int sensorIn = digitalRead(WHEEL_SENSOR_PIN);

   // If the switch changed, due to noise or pressing reset the debouncing timer
   if (sensorIn != _lastSensorIn) {
     _lastDebounceTime = uptime; 
   }
   
   bool steadyState = (uptime - _lastDebounceTime) > WHEEL_DEBOUNCE_DELAY_MILLIS;
   if (steadyState) {
     // whatever the sensorIn is at, it's been there for longer
     // than the debounce delay, so take it as the actual current state:
     
     updateLed(HIGH);
     
     if (sensorIn != _sensorState) {
       _sensorState = sensorIn;       

       // only count if the new button state is HIGH
       if (_sensorState == HIGH) {
          debugVerboseLog("sensor change (high)");
          
          bool validRotation = evaluateRotation(uptime);
          if (validRotation) {
            updateLed(LOW); // valid rotation
          }
          _lastSensorChange = uptime;
       } else {
          debugVerboseLog("sensor change (low)");
       }
     }
   }

   _lastSensorIn = sensorIn;
   checkForNightTimeout(uptime - _lastSensorChange);
}

void initWifi() {
  logSSID();
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  //wifiManager.resetSettings();
  wifiManager.setConfigPortalTimeout(240); // seconds
  //wifiManager.startConfigPortal(WIFI_CONFIG_AP_NAME, WIFI_CONFIG_AP_PW); //start the config portal, without trying to connect first
  wifiManager.autoConnect(WIFI_CONFIG_AP_NAME, WIFI_CONFIG_AP_PW);
  logSSID();
  log("wifi initialized");
}

void initMqtt() {
  mqttClient
    .setClient(wc)
    .setServer(MQTT_SERVER, MQTT_PORT);
  mqttConnect();
  log("mqtt initialized");
}

void initSerial() {
  Serial.begin(BAUDRATE);
  log("serial initialized");
}

void initSensor() {
  // init in-/outputs
  pinMode(WHEEL_LED_PIN, OUTPUT);
  pinMode(WHEEL_SENSOR_PIN, INPUT);

  // init led
  updateLed(_sensorState);

  log("wheel sensor initialized.");  
}

void setup() {
  initSerial();
  initSensor();
  initWifi();
  initMqtt();
}

void loop() {
  if (wifiConnected()) {
    if (!mqttConnected()) {
      mqttReconnect();
    } else {
      mqttClient.loop();
    }
  }

  readSensor();
}

