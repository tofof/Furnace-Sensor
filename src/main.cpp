#include <Arduino.h>
#include <Wire.h>
#include "Omron_D6FPH.h"
#include "DHTesp.h"
#include "ESP8266WiFi.h"
#include "PubSubClient.h"
#include "ArduinoJson.h"
#include "TM1637TinyDisplay.h"
#include "secrets.h"

#define CERT mqtt_broker_cert
#define MSG_BUFFER_SIZE (50)
#define CLK 16
#define DIO 14

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
Omron_D6FPH omron;
DHTesp dht;
TM1637TinyDisplay display(CLK, DIO);

void setup_wifi() {
  delay(10);
  display.showString("WIFI");
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
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  Serial.print("WiFi connected on IP address ");
  Serial.println(WiFi.localIP());
}

void sendMQTTDiscoveryMsg(String topic, String name, String unit, String dev_cla, String val_tpl) {
  String discoveryTopic = "homeassistant/sensor/" + topic + "/config";

  DynamicJsonDocument doc(1500);
  char buffer[512];

  doc["name"] = name;
  doc["stat_t"] = "furn/state";
  doc["unit_of_meas"] = unit;
  doc["dev_cla"] = dev_cla;
  doc["frc_upd"] = true;
  doc["val_tpl"] = val_tpl;
  doc["uniq_id"] = "furn/" + topic;
  
  JsonObject device = doc.createNestedObject("dev");
  device["ids"] = ["ff1"];
  device["name"] = "furn";

  size_t n = serializeJson(doc, buffer);
  Serial.println(buffer);

  mqttClient.publish(discoveryTopic.c_str(), buffer, n);
}

//--------------------------------------
// function connect called to (re)connect
// to the broker
//--------------------------------------
void connect() {
  display.showString("MQTT");
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    String mqttClientId = "";
    if (mqttClient.connect(mqttClientId.c_str(), MQTT_User, MQTT_Password)) {
      Serial.println("connected");
      sendMQTTDiscoveryMsg("omronT", "Temperature", "°F", "temperature", "{{ value_json.temperatureO }}");
      sendMQTTDiscoveryMsg("omronP", "Pressure", "Pa", "pressure", "{{ value_json.pressure }}");
      sendMQTTDiscoveryMsg("dhtT", "Temperature", "°F", "temperature", "{{ value_json.temperatureD }}");
      sendMQTTDiscoveryMsg("dhtH", "Humidity", "%", "humidity", "{{ value_json.humidity }}");
      sendMQTTDiscoveryMsg("dhtI", "Heat Index", "°F", "temperature", "{{ value_json.heatindex }}");
      sendMQTTDiscoveryMsg("airWF", "Wifi Strength", "dBm", "signal_strength", "{{ value_json.wifi }}");
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
  display.setBrightness(BRIGHT_7);
  setup_wifi();
  mqttClient.setServer(MQTT_Server, MQTT_Port);

  // D6FPH Differential Pressure Sensor
  omron.begin(MODEL_0025AD1);

  // DHT22 Humidity/Temp Sensor
  Serial.println();
  Serial.println("Status\tHumidity (%)\tTemperature (C)\t(F)\tHeatIndex (C)\t(F)");
  String thisBoard= ARDUINO_BOARD;
  Serial.println(thisBoard);
  dht.setup(2, DHTesp::DHT22); //2 is LED, so we also get a blink on each update
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
  float temperature = dht.getTemperature();
  Serial.print("\t");
  Serial.print(dht.getStatusString());
  Serial.print("\t");
  Serial.print(humidity, 1);
  Serial.print("\t\t");
  Serial.print(temperature, 1);
  Serial.print("\t");
  Serial.print(dht.toFahrenheit(temperature), 1);
  Serial.print("\t\t");
  float heatindex = dht.computeHeatIndex(temperature, humidity, false);
  Serial.print(heatindex, 1);
  Serial.print("\t");
  Serial.println(dht.computeHeatIndex(dht.toFahrenheit(temperature), humidity, true), 1);
}

void getReadings() {
  DynamicJsonDocument doc(1024);
  char buffer[256];
  float pressure = getPressureOmron();
  doc["pressure"] = pressure;
  doc["temperatureO"] = dht.toFahrenheit(getTemperatureOmron());
  float humidity = dht.getHumidity();
  float temperature = dht.getTemperature();
  float heatindex = dht.computeHeatIndex(temperature, humidity, false);
  doc["humidity"] = humidity;
  doc["temperatureD"] = dht.toFahrenheit(temperature);
  doc["heatindex"] = dht.toFahrenheit(heatindex);
  doc["wifi"] = WiFi.RSSI();
  size_t n = serializeJson(doc, buffer);
  mqttClient.publish("furn/state", buffer, n);

  //display.showString("\xB0", 1, 3); //degree symbol in rightmost (position 3)
  //display.showNumber(dht.toFahrenheit(temperature), false, 3, 0); 
  display.showNumber(pressure*10);
}

void loop() {
  if (!mqttClient.connected()) {
    connect();
  }
  mqttClient.loop();

  //getPressureOmron();
  //getTemperatureOmron();
  //getSampleDHT();
  getReadings();
  //delay(dht.getMinimumSamplingPeriod());
  delay(10000);
}

