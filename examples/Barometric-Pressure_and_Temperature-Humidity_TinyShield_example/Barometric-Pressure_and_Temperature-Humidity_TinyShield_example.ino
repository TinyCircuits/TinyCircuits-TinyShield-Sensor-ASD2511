/*  The follow sketch is as combination of the example program for 
 *  the humidity and temperature sensor, and the barometric pressure
 *  sensor populated on the TinyCircuits Combo Sensor Board.
 *  
 *  Written by: Laveréna Wienclaw for TinyCircuits
 *  Date: July 18, 2019
*/

// Include files necessary for the Barometric Pressure Sensor
#include "BMP280.h"
#include "Wire.h"
#define P0 1013.25
BMP280 bmp;

// Include files necessary for the Temperature and Humidity Sensor
#include <SI7021.h>
SI7021 sensor;

#if defined(ARDUINO_ARCH_SAMD)
#define SerialMonitorInterface SerialUSB
#else
#define SerialMonitorInterface Serial
#endif

void setup()
{
  Wire.begin(); // Begin I2C communication
  SerialMonitorInterface.begin(115200);
  while(!SerialMonitorInterface);
  
  // Initialize B.P. Sensor
  if(!bmp.begin()){
    SerialMonitorInterface.println("BMP init failed!");
    while(1);
  }
  else SerialMonitorInterface.println("BMP init success!");
  
  bmp.setOversampling(4);

  // Initialize Humidty and Temperature sensor
  SerialMonitorInterface.print("Initializing Humidity sensor... ");
  if(!sensor.begin()){
    SerialMonitorInterface.println("Sensor not found!");
    while(true);
  }
  SerialMonitorInterface.println("Success!");
  
}
void loop()
{
  /****************** BAROMETRIC PRESSURE SENSOR CODE ******************/ 
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
  
//  delay(100);

  /********************* HUMIDITY SENSOR CODE *********************/
  int celcius=sensor.getCelsiusHundredths()/100;
  int relativeHumidity=sensor.getHumidityPercent();
  SerialMonitorInterface.print(celcius);
  SerialMonitorInterface.print(" deg Celsius\t");
  SerialMonitorInterface.print(relativeHumidity);
  SerialMonitorInterface.println("% relative humidity");
  
  delay(500);
}
