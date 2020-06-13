#include <OneWire.h> 
#include <DallasTemperature.h>

OneWire ourWire(8);
DallasTemperature DS18B20(&ourWire);

unsigned long dt;
float tem;

void setup() {
  Serial.begin(115200); 
  DS18B20.begin(); 
}

void loop() {
  dt = millis();
  DS18B20.requestTemperatures(); 
  tem = DS18B20.getTempCByIndex(0); 
  dt = millis() - dt;
  Serial.print("Temperatura = ");
  Serial.print(tem);
  Serial.print(" °C");
  Serial.print(", dt = ");
  Serial.println(dt);
  delay(1000);
}