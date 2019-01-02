/*
TinyDuino Accelerometer Demo
  
Updated 15 August 2016 to use the correct serial monitor interface for
TinyDuino or TinyScreen+

This example code is in the public domain.

http://www.tinycircuits.com

*/


#include <Wire.h>
#include "BMA250.h"


#if defined(ARDUINO_ARCH_SAMD)
#define SerialMonitorInterface SerialUSB
#else
#define SerialMonitorInterface Serial
#endif

BMA250 accel;


void setup()
{
  SerialMonitorInterface.begin(9600);
  Wire.begin();
  accel.begin(BMA250_range_2g, BMA250_update_time_64ms);//This sets up the BMA250 accelerometer
}

void loop()
{
  accel.read();//This function gets new data from the accelerometer
  SerialMonitorInterface.print("X = ");
  SerialMonitorInterface.print(accel.X);
  SerialMonitorInterface.print("  ");
  SerialMonitorInterface.print("Y = ");
  SerialMonitorInterface.print(accel.Y);
  SerialMonitorInterface.print("  ");
  SerialMonitorInterface.print("Z = ");
  SerialMonitorInterface.print(accel.Z);
  SerialMonitorInterface.print("  Temperature(C) = ");
  SerialMonitorInterface.println((accel.rawTemp*0.5)+24.0,1);
  delay(250);//We'll make sure we're over the 64ms update time set on the BMA250
}