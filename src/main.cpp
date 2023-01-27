#include <Arduino.h>
#include <Wire.h>
#include "Omron_D6FPH.h"
#include "DHTesp.h"
#include "NTPClient.h"
#include "ESP8266WiFi.h"
#include "PubSubClient.h"
#include "NTPClient.h"
#include "WiFiUdp.h"
#include "secrets.h"

#define CERT mqtt_broker_cert
#define MSG_BUFFER_SIZE (50)

WiFiClient wifiClient;
WiFiUDP ntpUDP;
PubSubClient mqttClient(wifiClient);
Omron_D6FPH omron;
DHTesp dht;
char c[8];

void setup_wifi() {
  delay(10);
  Serial.println();
  WiFi.hostname("ESP-host");
  WiFi.setPhyMode(WIFI_PHY_MODE_11B);
  Serial.print("Connecting to ");
  Serial.print(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
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
    if (mqttClient.connect(mqttClientId.c_str(), MQTT_User, MQTT_Password)) {
      Serial.println("connected");
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
  mqttClient.setServer(MQTT_Server, MQTT_Port);

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
      mqttClient.publish("sensor/omron/pressure", c);
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
      dtostrf(dht.toFahrenheit(temperature), 1, 1, c);
      mqttClient.publish("sensor/omron/temperature", c);
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
  mqttClient.publish("sensor/dht/humidity", c);
  Serial.print("\t\t");
  Serial.print(temperature, 1);
  dtostrf(temperature, 1, 1, c);
  Serial.print("\t");
  Serial.print(dht.toFahrenheit(temperature), 1);
  dtostrf(dht.toFahrenheit(temperature), 1, 1, c);
  mqttClient.publish("sensor/dht/temperature", c);
  Serial.print("\t\t");
  float heatindex = dht.computeHeatIndex(temperature, humidity, false);
  Serial.print(heatindex, 1);
  dtostrf(dht.toFahrenheit(heatindex), 1, 1, c);
  mqttClient.publish("sensor/dht/heat_index", c);
  Serial.print("\t");
  Serial.println(dht.computeHeatIndex(dht.toFahrenheit(temperature), humidity, true), 1);
}

void loop() {
  if (!mqttClient.connected()) {
    connect();
  }
  mqttClient.loop();

  getPressureOmron();
  getTemperatureOmron();
  getSampleDHT();
  delay(dht.getMinimumSamplingPeriod());
}

