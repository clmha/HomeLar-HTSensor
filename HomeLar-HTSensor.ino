/*HOMELAR-HTSENSOR A remote/wireless temperature and humidity sensor

  Compiled using:
    * Arduino IDE 1.8.9
    * ESP8266 Arduino Core 2.5.0
    * DHT Sensor Library 1.3.4
    * Adafruit Unified Sensor 1.0.3
    * PubSubClient 2.7.0
 */

// Standard
#include <string.h>

// ESP8266 Arduino Core
#include <Arduino.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>

// PubSubClient
#include <PubSubClient.h>

// DHT Sensor Library
// Adafruit Unified Sensor
#include <DHT.h>

// SW Configuration
#define DHT_PIN                     D1
#define HW_UART_SPEED               115200
#define MSG_PERIOD                  1200      // [s]
#define MSG_OFFSET                  0         // [s]
#define SENSE_TIME                  2         // [s]
#define SETUP_TIME                  0.1       // [s]
#define LOOP_PERIOD                 0.5       // [s]

/*
 * Static Variables
 */
DHT dht(DHT_PIN, DHT22);
WiFiClient network;
PubSubClient client(network);
struct eeConf {
  char ssid[32];
  char passwd[64];
  char mqttSrv[32];
} conf;
unsigned short int iLoop; // Number of LOOP iterations

/*
 *  Function Declarations
 */
void waitTimeHit(unsigned long tStart_ms, unsigned long tPeriod_ms);

/*
 * Function Definitions
 */
void setup() {
  //SETUP Code here to run once

  // Init
  iLoop = 0;
  EEPROM.begin(sizeof(struct eeConf));
  EEPROM.get(0, conf);

  // Serial
  Serial.begin(HW_UART_SPEED);
  while(!Serial);
  Serial.print("*N* SETUP START ");
  Serial.println(millis());
  
  // DHT Sensor
  dht.begin();

  // WiFi
  WiFi.begin(conf.ssid, conf.passwd);

  // MQTT
  client.setServer(conf.mqttSrv, 1883);

  // Sync.
  Serial.print("*N* SETUP END ");
  Serial.println(millis());
  waitTimeHit(0, SETUP_TIME*1E3);
}

void loop() {
  //LOOP Code to run repeatedly
  bool isSleepingTime = false;
  char msg[4];
  float h_Pct;
  float T_DegC;
  unsigned long tStart_ms = millis();

  iLoop++;
  Serial.print("*N* LOOP START ");
  Serial.println(millis());
  if (WiFi.status() == WL_CONNECTED) {
    // We are still connected to WiFi, we can continue with the wireless services.
    
    if(!client.connected()) {
      // We are not connected to the MQTT server, try to connect.
      
      Serial.print("*N* Connecting to MQTT server ");
      Serial.println(conf.mqttSrv);
      
      if(!client.connect(WiFi.hostname())) {
        Serial.print("*W* MQTT connection failed (");
        Serial.print(client.state());
        Serial.println(")");
      }
    } else {
      // We are connected to the MQTT server, we can continue with MQTT operations.
      client.loop();
      
      if (iLoop >= SENSE_TIME/LOOP_PERIOD) {
        // According to the count and period of the loops, we have reached the end of the allocated sensing time. 
        // Send the messages and go to sleep.
    
        // The temperature sensor accuracy is +/-.5, 
        // there is no need to keep the floating point accuracy.
        T_DegC = dht.readTemperature();
        snprintf(msg, 4, "%2d", (int) round(T_DegC));
        Serial.print("T = ");
        Serial.print(T_DegC);
        Serial.print(" °C; ");
        client.publish("sensors/testRoom/T_DegC", msg);
        
        // The relative humidity sensor accuracy is between 2 and 5 points,
        // there is no need to keep the floating point accuracy.
        h_Pct = dht.readHumidity();
        snprintf(msg, 4, "%3d", (int) round (h_Pct));
        Serial.print("h = ");
        Serial.print(h_Pct);
        Serial.print("%");
        Serial.println();
        client.publish("sensors/testRoom/h_Pct", msg);

        // MQTT messages sent, we are ready to sleep.
        isSleepingTime = true;
      }
    }
  } else {
    // We are not connected to WiFi, print a notice and try again at the next iteration.
    Serial.print("*N* Connecting to WiFi ");
    Serial.println(conf.ssid);
  }

  // Sync.
  Serial.print("*N* LOOP END ");
  Serial.println(millis());
  waitTimeHit(tStart_ms, LOOP_PERIOD*1E3);

  // Sleep
  if (isSleepingTime) {
    ESP.deepSleep((MSG_PERIOD+MSG_OFFSET-SENSE_TIME-SETUP_TIME)*1E6);
  }
}

void waitTimeHit(unsigned long tStart_ms, unsigned long tPeriod_ms) {
  //WAITTIMEHIT Pause the execution until the next time hit
  unsigned long T_ms = millis()-tStart_ms;
  
  if (T_ms < tPeriod_ms) {
    delay(tPeriod_ms-T_ms); // Wait for the next time hit.
  } else {
    // The task took more time to run than its period, print an error message.
    Serial.println("*E* Task overrun");
  }
}
