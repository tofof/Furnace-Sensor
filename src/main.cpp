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

//--------------------------------------
// function callback called everytime 
// if a mqtt message arrives from the broker
//--------------------------------------
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived on topic: '");
  Serial.print(topic);
  Serial.print("' with payload: ");
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  String myCurrentTime = timeClient.getFormattedTime();
  mqttClient.publish(mqttTopicOut,("ESP8266: Cedalo Mosquitto is awesome. ESP8266-Time: " + myCurrentTime).c_str());
}


void setup() {
  Serial.begin(115200);
  Wire.begin();
  setup_wifi();
  mqttClient.setServer(mqtt_server, mqtt_server_port);
  mqttClient.setCallback(callback);

  // D6FPH Differential Pressure Sensor
  omron.begin(MODEL_0025AD1);

  // DHT22 Humidity/Temp Sensor
  Serial.println();
  Serial.println("Status\tHumidity (%)\tTemperature (C)\t(F)\tHeatIndex (C)\t(F)");
  String thisBoard= ARDUINO_BOARD;
  Serial.println(thisBoard);
  dht.setup(2, DHTesp::DHT22); // Connect DHT sensor to GPIO 17
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
  }
  return(temperature);
}

void getSampleDHT() {
  Serial.println();
  Serial.println("DHT22");
  Serial.println("\tStatus\tHumidity (%)\tTemp(C)\t(F)\tHeatIndex (C)\t(F)");
  float humidity = dht.getHumidity();
  float temperature22 = dht.getTemperature();
  Serial.print("\t");
  Serial.print(dht.getStatusString());
  Serial.print("\t");
  Serial.print(humidity, 1);
  Serial.print("\t\t");
  Serial.print(temperature22, 1);
  Serial.print("\t");
  Serial.print(dht.toFahrenheit(temperature22), 1);
  Serial.print("\t\t");
  Serial.print(dht.computeHeatIndex(temperature22, humidity, false), 1);
  Serial.print("\t");
  Serial.println(dht.computeHeatIndex(dht.toFahrenheit(temperature22), humidity, true), 1);
  
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

