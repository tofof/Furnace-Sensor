#include <Arduino.h>
#include <Wire.h>
#include "Omron_D6FPH.h"
#include "DHTesp.h"
#include "NTPClient.h"
#include "ESP8266WiFi.h"
#include "PubSubClient.h"
#include "NTPClient.h"
#include "WiFiUdp.h"

#define CERT mqtt_broker_cert
#define MSG_BUFFER_SIZE (50)

const char* ssid = "Equestria";
const char* password = "20%cooler";
const char* mqtt_server = "tofof-2evcmqe1ysph.cedalo.cloud";
const uint16_t mqtt_server_port = 1883; 
const char* mqttUser = "airsensor";
const char* mqttPassword = "derp";
const char* mqttTopicIn = "esp-8266-in";
const char* mqttTopicOut = "esp-8266-out";

WiFiClient wifiClient;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
PubSubClient mqttClient(wifiClient);
Omron_D6FPH omron;
DHTesp dht;
char c[8];

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.print(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  timeClient.begin();
  Serial.print("WiFi connected on IP address ");
  Serial.println(WiFi.localIP());
}

//--------------------------------------
// function connect called to (re)connect
// to the broker
//--------------------------------------
void connect() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    String mqttClientId = "";
    if (mqttClient.connect(mqttClientId.c_str(), mqttUser, mqttPassword)) {
      Serial.println("connected");
      mqttClient.subscribe(mqttTopicIn);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" will try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  setup_wifi();
  mqttClient.setServer(mqtt_server, mqtt_server_port);

  // D6FPH Differential Pressure Sensor
  omron.begin(MODEL_0025AD1);

  // DHT22 Humidity/Temp Sensor
  Serial.println();
  Serial.println("Status\tHumidity (%)\tTemperature (C)\t(F)\tHeatIndex (C)\t(F)");
  String thisBoard= ARDUINO_BOARD;
  Serial.println(thisBoard);
  dht.setup(2, DHTesp::DHT22); //2 is LED, so we get a blink on each update
} 

float getPressureOmron() {
  Serial.println();
  Serial.println("Omron D6FPH");
  float pressure = omron.getPressure();
  if(isnan(pressure)){
  Serial.println("\tError: Could not read pressure data");
  }else{
      Serial.print("\tDifferential Pressure: ");
      Serial.println(pressure);
      dtostrf(pressure, 1, 1, c); //arg2 is mininum width, arg3 is precision in places past decimal
      mqttClient.publish("omron-differential-pressure-Pa", c);
  }
  return(pressure);
}

float getTemperatureOmron() {
  float temperature = omron.getTemperature();
  if(isnan(temperature)){
  Serial.println("\tError: Could not read temperature data");
  }else{
      Serial.print("\tTemperature C: ");
      Serial.println(temperature);              
      dtostrf(temperature, 1, 1, c);
      mqttClient.publish("omron-temperature-C", c);
      dtostrf(dht.toFahrenheit(temperature), 1, 1, c);
      mqttClient.publish("omron-temperature-F", c);
  }
  return(temperature);
}

void getSampleDHT() {
  Serial.println();
  Serial.println("DHT22");
  Serial.println("\tStatus\tHumidity (%)\tTemp(C)\t(F)\tHeatIndex (C)\t(F)");
  float humidity = dht.getHumidity();
  float temperature = dht.getTemperature();
  Serial.print("\t");
  Serial.print(dht.getStatusString());
  Serial.print("\t");
  Serial.print(humidity, 1);
  dtostrf(humidity, 1, 1, c);
  mqttClient.publish("DHT-humidity-%", c);
  Serial.print("\t\t");
  Serial.print(temperature, 1);
  dtostrf(temperature, 1, 1, c);
  mqttClient.publish("DHT-temperature-C", c);
  Serial.print("\t");
  Serial.print(dht.toFahrenheit(temperature), 1);
  dtostrf(dht.toFahrenheit(temperature), 1, 1, c);
  mqttClient.publish("DHT-temperature-F", c);
  Serial.print("\t\t");
  float heatindex = dht.computeHeatIndex(temperature, humidity, false);
  Serial.print(heatindex, 1);
  dtostrf(heatindex, 1, 1, c);
  mqttClient.publish("DHT-heat-index", c);
  Serial.print("\t");
  Serial.println(dht.computeHeatIndex(dht.toFahrenheit(temperature), humidity, true), 1);
  
}

void loop() {
  if (!mqttClient.connected()) {
    connect();
  }
  mqttClient.loop();
  timeClient.update();

  getPressureOmron();
  getTemperatureOmron();
  getSampleDHT();
  delay(dht.getMinimumSamplingPeriod());
}

