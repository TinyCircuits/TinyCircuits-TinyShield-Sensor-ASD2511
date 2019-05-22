//-------------------------------------------------------------------------------
//  TinyCircuits Compass TinyShield Example Sketch
//  Using Honeywell HMC5883 in I2C mode to read out the x, y, and z axis compass
//  data
//
//  Created 2/16/2014
//  by Ken Burns, TinyCircuits http://TinyCircuits.com
//  Modified 3/18/2019
//  By Laverena Wienclaw, TinyCircuits
//
//  This example code is in the public domain.
//
//-------------------------------------------------------------------------------

#include <Wire.h>
#ifndef ARDUINO_ARCH_SAMD
#include <EEPROM.h>
#endif

#define HMC5883_I2CADDR     0x1E

int CompassX;
int CompassY;
int CompassZ;

#if defined(ARDUINO_ARCH_SAMD)
  #define SerialMonitor SerialUSB
#else
  #define SerialMonitor Serial
#endif

void setup()
{
  Wire.begin();
  SerialMonitor.begin(115200);
  HMC5883Init();
}


void loop()
{
  HMC5883ReadCompass();

  // Print out the compass data to the Serial Monitor (found under Tools)
  SerialMonitor.print("x: ");
  SerialMonitor.print(CompassX);
  SerialMonitor.print(", y: ");
  SerialMonitor.print(CompassY);
  SerialMonitor.print(", z:");
  SerialMonitor.println(CompassZ); 

  // Delay a second
  delay(1000); 
}


void HMC5883Init()
{
  //Put the HMC5883 into operating mode
  Wire.beginTransmission(HMC5883_I2CADDR);
  Wire.write(0x02);     // Mode register
  Wire.write(0x00);     // Continuous measurement mode
  Wire.endTransmission();
}


void HMC5883ReadCompass()
{
  uint8_t ReadBuff[6];
  
  // Read the 6 data bytes from the HMC5883
  Wire.beginTransmission(HMC5883_I2CADDR);
  Wire.write(0x03); 
  Wire.endTransmission();
  Wire.requestFrom(HMC5883_I2CADDR,6);

  // Retrieve from all six data registers
  for(int i = 0; i < 6;i++)
  {
    ReadBuff[i] = Wire.read();
  }
  
  CompassX = ReadBuff[0] << 8;
  CompassX |= ReadBuff[1];

  CompassZ = ReadBuff[2] << 8;
  CompassZ |= ReadBuff[3];
  
  CompassY = ReadBuff[4] << 8;
  CompassY |= ReadBuff[5];
}



