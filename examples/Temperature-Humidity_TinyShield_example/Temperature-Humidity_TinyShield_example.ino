/*
  TinyCircuits Si7020 Temperature and Humidity Sensor TinyShield Example Sketch

  This demo shows how to read temperature and humidity data from the Si7020 
  sensor using the Si7020 library: https://github.com/LowPowerLab/SI7021

  The program will print the temperature & humidity, then enable the heater for 
  30 seconds and print the temperature & humidity to show the difference the 
  internal heater can make. The heater will then be disabled with a cool down 
  period of 30 seconds.
  
  Rewritten May 2021 to include built-in heater functionality by L. Wienclaw

  https://TinyCircuits.com
*/

#include <Wire.h>
#include <SI7021.h>

SI7021 sensor;

// Make Serial Monitor compatible for all TinyCircuits processors
#if defined(ARDUINO_ARCH_AVR)
  #define SerialMonitorInterface Serial
#elif defined(ARDUINO_ARCH_SAMD)
  #define SerialMonitorInterface SerialUSB
#endif

void setup() {
  SerialMonitorInterface.begin(115200);
  Wire.begin();
  SerialMonitorInterface.print("Initializing sensor... ");
  if(!sensor.begin()){
    SerialMonitorInterface.println("Sensor not found!");
    while(true);
  }
  SerialMonitorInterface.println("Success!");

  // this driver should work for SI7020 and SI7021, this returns 20 or 21
  int deviceid = sensor.getDeviceId();
//    SerialMonitorInterface.println(deviceid);
}

void loop() {
    // read and print temp and humidity
    float temperature = sensor.getCelsiusHundredths() / 100.0;
    SerialMonitorInterface.print(temperature);
    SerialMonitorInterface.print(" deg Celsius\t");
    int humidity = sensor.getHumidityPercent();
    SerialMonitorInterface.print(humidity);
    SerialMonitorInterface.println("% relative humidity");

    // enable internal heater for testing
    sensor.setHeater(true);
    delay(30000);
    sensor.setHeater(false);
    
    // see if heater changed temperature
    // read and print temp and humidity
    temperature = sensor.getCelsiusHundredths() / 100.0;
    SerialMonitorInterface.print(temperature);
    SerialMonitorInterface.print(" deg Celsius\t");
    humidity = sensor.getHumidityPercent();
    SerialMonitorInterface.print(humidity);
    SerialMonitorInterface.println("% relative humidity -- (after heater on)");
    
    //cool down
    delay(30000);

    // get humidity and temperature in one shot, saves power because sensor takes temperature when doing humidity anyway
//    si7021_env data = sensor.getHumidityAndTemperature();
//    delay(5000);
}
