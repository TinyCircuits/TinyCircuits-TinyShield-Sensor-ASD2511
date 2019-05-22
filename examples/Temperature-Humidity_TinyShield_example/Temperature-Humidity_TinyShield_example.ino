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

#if defined(ARDUINO_ARCH_SAMD)
#define SerialMonitorInterface SerialUSB
#else
#define SerialMonitorInterface Serial
#endif

SI7021 sensor;

void setup()
{
  SerialMonitorInterface.begin(115200);
  Wire.begin();
  SerialMonitorInterface.print("Initializing sensor... ");
  if(!sensor.begin()){
  	SerialMonitorInterface.println("Sensor not found!");
  	while(true);
  }
  SerialMonitorInterface.println("Success!");
}

void loop()
{
  int celcius=sensor.getCelsiusHundredths()/100;
  int relativeHumidity=sensor.getHumidityPercent();
  SerialMonitorInterface.print(celcius);
  SerialMonitorInterface.print(" deg Celsius\t");
  SerialMonitorInterface.print(relativeHumidity);
  SerialMonitorInterface.println("% relative humidity");
  
  delay(500);
}
