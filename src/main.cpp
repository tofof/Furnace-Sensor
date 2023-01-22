#include <Arduino.h>
#include <Wire.h>
#include <Omron_D6FPH.h>
#include "DHTesp.h" // Click here to get the library: http://librarymanager/All#DHTesp

#ifdef ESP32
#pragma message(THIS EXAMPLE IS FOR ESP8266 ONLY!)
#error Select ESP8266 board.
#endif

Omron_D6FPH omron;
DHTesp dht;

void setup() {
  // D6FPH Differential Pressure Sensor
  Serial.begin(9600);
  Wire.begin();
  omron.begin(MODEL_0025AD1);

  // DHT22 Humidity/Temp Sensor
  Serial.println();
  Serial.println("Status\tHumidity (%)\tTemperature (C)\t(F)\tHeatIndex (C)\t(F)");
  String thisBoard= ARDUINO_BOARD;
  Serial.println(thisBoard);

  // Autodetect is not working reliable, don't use the following line
  // dht.setup(17);
  // use this instead: 
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
  getPressureOmron();
  getTemperatureOmron();
  getSampleDHT();
  delay(dht.getMinimumSamplingPeriod());
}

