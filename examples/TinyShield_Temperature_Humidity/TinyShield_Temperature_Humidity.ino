/*
  TinyCircuits Si7020 Temperature and Humidity Sensor TinyShield Example Sketch
  
  This demo shows the bare minimum to read temperature and humidity data from 
  the Si7020 sensor using the Si7020 library written by Marcus Sorensen.

  This example is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  Written 20 March 2016
  By Ben Rose
  Modified 07 January 2019
  By Hunter Hykes

  https://TinyCircuits.com
*/

#include <Wire.h>
#include <SI7021.h>

SI7021 sensor;

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  Serial.print("Initializing sensor... ");
  if(!sensor.begin()){
  	Serial.println("Sensor not found!");
  	while(true);
  }
  Serial.println("Success!");
}

void loop()
{
  int celcius=sensor.getCelsiusHundredths()/100;
  int relativeHumidity=sensor.getHumidityPercent();
  Serial.print(celcius);
  Serial.print(" deg Celsius\t");
  Serial.print(relativeHumidity);
  Serial.println("% relative humidity");
  
  delay(500);
}
