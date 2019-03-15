/*
  TinyDuino Barometric Pressure Demo
  
  This example code is in the public domain.

  Written 
  By 
  Modified 07 January 2019
  By Hunter Hykes

  https://TinyCircuits.com
*/

#include "BMP280.h"
#include "Wire.h"
#define P0 1013.25
BMP280 bmp;

#if defined(ARDUINO_ARCH_SAMD)
#define SerialMonitorInterface SerialUSB
#else
#define SerialMonitorInterface Serial
#endif

void setup()
{
  SerialMonitorInterface.begin(9600);
  if(!bmp.begin()){
    SerialMonitorInterface.println("BMP init failed!");
    while(1);
  }
  else SerialMonitorInterface.println("BMP init success!");
  
  bmp.setOversampling(4);
  
}
void loop()
{
  double T,P;
  char result = bmp.startMeasurment();
 
  if(result!=0){
    delay(result);
    result = bmp.getTemperatureAndPressure(T,P);
    
      if(result!=0)
      {
        double A = bmp.altitude(P,P0);
        
        SerialMonitorInterface.print("T = \t");SerialMonitorInterface.print(T,2); SerialMonitorInterface.print(" degC\t");
        SerialMonitorInterface.print("P = \t");SerialMonitorInterface.print(P,2); SerialMonitorInterface.print(" mBar\t");
        SerialMonitorInterface.print("A = \t");SerialMonitorInterface.print(A,2); SerialMonitorInterface.println(" m");
       
      }
      else {
        SerialMonitorInterface.println("Error.");
      }
  }
  else {
    SerialMonitorInterface.println("Error.");
  }
  
  delay(100);
}
