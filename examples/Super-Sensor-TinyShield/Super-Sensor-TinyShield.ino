/************************************************************************
 * Combo Sensor TinyShield Example Program
 * 
 * This program utilizes all sensors on the Combo Sensor TinyShield and
 * prints the data to the Serial Monitor.
 *
 * Hardware by: TinyCircuits
 * Written by: Hunter Hykes for TinyCircuits
 *
 * Initiated: 01/14/2020
 * Updated: 01/14/2020
 ************************************************************************/

#include <Wire.h>
#include "RTIMUSettings.h"  // 9-Axis
#include "RTIMU.h"          // 9-Axis
#include "RTFusionRTQF.h"   // 9-Axis
#include <SI7021.h>         // Temp/Hum
#include "BMP280.h"         // Pressure Sensor

#ifndef ARDUINO_ARCH_SAMD
#include <EEPROM.h>
#endif

// Make compatible with all TinyCircuits processors
#if defined(ARDUINO_ARCH_AVR)
#define SerialMonitorInterface Serial
#elif defined(ARDUINO_ARCH_SAMD)
#define SerialMonitorInterface SerialUSB
#endif

// sets the rate at which results are displayed
#define DISPLAY_INTERVAL 10000    // interval between SerialMonitorInterface updates
#define  SERIAL_PORT_SPEED 115200 // defines the speed to use for the debug serial port

/* * * * * * * * * * * * 9-Axis Sensor * * * * * * * * * * * */
RTIMU *imu;               // the IMU object
RTFusionRTQF fusion;      // the fusion object
RTIMUSettings settings;   // the settings object
unsigned long lastDisplay;
unsigned long lastRate;
int sampleCount;
unsigned long delta;
unsigned long now = millis();
RTVector3 accelData;    // for storing 9-Axis accelerometer data
RTVector3 gyroData;     // for storing 9-Axis gyroscope data
RTVector3 compassData;  // for storing 9-Axis compass data
RTVector3 fusionData;   // for storing 9-Axis fusion data

/* * * * * * * * * * * Temp/Hum Sensor * * * * * * * * * * */
SI7021 sensor;        // Temp/Hum Sensor Object
int celcius;          // temperature data
int relativeHumidity; // humidity data

/* * * * * * * *  ** * Pressure Sensor * * * * * * * * * * */
#define P0 1013.25
BMP280 bmp;
bool PrintPressure = true; // if true, pressure values will be printed
double T, A, P; // temperature, altitude, and pressure value readings

/* * * * * * * * * Ambient Light Sensor * * * * * * * * * */
#define TSL2572_I2CADDR 0x39 // address of ambient light sensor

#define GAIN_1X 0
#define GAIN_8X 1
#define GAIN_16X 2
#define GAIN_120X 3

#define GAIN_DIVIDE_6 true //only use this with 1x and 8x gain settings
int gain_val = 0;

float AmbientLightLux;  // Illuminance value reading


void setup() {
  imu = RTIMU::createIMU(&settings);  // 9-Axis object

  SerialMonitorInterface.begin(SERIAL_PORT_SPEED); // Setup the Serial Monitor to operate at the specified Baud rate
  while (!SerialMonitorInterface); // do nothing until the Serial Monitor is opened

  Wire.begin(); // start I2C communications

  init9axis();          // initialize the 9-axis sensor
  initTempSensor();     // initialize the temperature and humidity sensor
  initPressSensor();    // initialize the pressure sensor
  TSL2572init(GAIN_1X); // initialize the illuminance sensor

  SerialMonitorInterface.println("\n- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");
  SerialMonitorInterface.println("\t\t\tEND OF SETUP\t\t\t");
  SerialMonitorInterface.println("- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");
}

void loop() {
  /* * * * * * * * * * 9-Axis * * * * * * * * * */
  read9axis();
  //printAccel();
  //printGyro();
  //printMag();
  printFused();

  /* * * * * * * * * * Temp/Hum * * * * * * * * * */
  readTempSensor();
  printTempSensor();

  /* * * * * * * * * * Pressure * * * * * * * * * */
  readPressSensor();

  /* * * * * * * * * * Light * * * * * * * * * */
  AmbientLightLux = Tsl2572ReadAmbientLight();
  printLight();

  SerialMonitorInterface.println("\n- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -\n");

  //delay(DISPLAY_INTERVAL); // wait some predetermined time before printing the next set of samples
}

// initialize the 9-Axis sensor
void init9axis() {
  int errcode;

  SerialMonitorInterface.print("Initializing 9-Axis..."); SerialMonitorInterface.println(imu->IMUName());
  if ((errcode = imu->IMUInit()) < 0) {
    SerialMonitorInterface.print("\t9-Axis Init Failed!"); SerialMonitorInterface.println(errcode);
  }

  if (imu->getCalibrationValid())
    SerialMonitorInterface.println("\tUsing compass calibration");
  else
    SerialMonitorInterface.println("\tNo valid compass calibration data");

  lastDisplay = lastRate = millis();
  sampleCount = 0;

  // Slerp power controls the fusion and can be between 0 and 1
  // 0 means that only gyros are used, 1 means that only accels/compass are used
  // In-between gives the fusion mix.
  fusion.setSlerpPower(0.02);

  // use of sensors in the fusion algorithm can be controlled here
  // change any of these to false to disable that sensor
  fusion.setGyroEnable(true);
  fusion.setAccelEnable(true);
  fusion.setCompassEnable(true);
}

// read values from 9-Axis and update variables
void read9axis() {
  now = millis();

  if (imu->IMURead()) {                                // get the latest data if ready yet
    fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
    sampleCount++;
    if ((delta = now - lastRate) >= 1000) {
      SerialMonitorInterface.print("Sample rate: "); SerialMonitorInterface.print(sampleCount);
      if (imu->IMUGyroBiasValid())
        SerialMonitorInterface.println(", gyro bias valid");
      else
        SerialMonitorInterface.println(", calculating gyro bias - don't move IMU!!");

      sampleCount = 0;
      lastRate = now;
    }

    accelData = imu->getAccel();
    gyroData = imu->getGyro();
    compassData = imu->getCompass();
    fusionData = fusion.getFusionPose();
  }
}

// initialize the temperature/humidity sensor
void initTempSensor() {
  SerialMonitorInterface.println("Initializing Temp/Hum Sensor... ");
  if (!sensor.begin()) {
    SerialMonitorInterface.println("\tSI7021 Init Failed!");
    while (true);
  }
  SerialMonitorInterface.println("\tSuccess!");
}

// read values from temperature/humidity sensor and update variables
void readTempSensor() {
  celcius = sensor.getCelsiusHundredths() / 100;
  relativeHumidity = sensor.getHumidityPercent();
}

// print temperature/humidity sensor variables to SerialMonitorInterface
void printTempSensor() {
  SerialMonitorInterface.println("\nTEMPERATURE DATA");
  SerialMonitorInterface.print(celcius);
  SerialMonitorInterface.print("°C\t");
  SerialMonitorInterface.print(relativeHumidity);
  SerialMonitorInterface.println("% Humidity");
}

// print 9-Axis accelerometer values to SerialMonitorInterface
void printAccel() {
  SerialMonitorInterface.println("\nACCELEROMETER DATA");
  displayAxis("Accel: ", accelData.x(), accelData.y(), accelData.z());        // accel data
}

// print 9-Axis gyroscope values to SerialMonitorInterface
void printGyro() {
  SerialMonitorInterface.println("\nGYROSCOPE DATA");
  displayAxis("Gyro: ", gyroData.x(), gyroData.y(), gyroData.z());            // gyro data
}

// print 9-Axis magnetometer values to SerialMonitorInterface
void printMag() {
  SerialMonitorInterface.println("\nMAGNETOMETER DATA");
  displayAxis("Mag: ", compassData.x(), compassData.y(), compassData.z());    // compass data
}

// print 9-Axis fused data values to SerialMonitorInterface
void printFused() {
  SerialMonitorInterface.println("\nFUSED DATA");
  displayDegrees("Pose: ", fusionData.x(), fusionData.y(), fusionData.z());   // fused output
}

// prints the specified data label followed by the provided x, y, z values
void displayAxis(const char *label, float x, float y, float z) {
  SerialMonitorInterface.print(label);
  SerialMonitorInterface.print("\tx: "); SerialMonitorInterface.println(x);
  SerialMonitorInterface.print("\ty: "); SerialMonitorInterface.println(y);
  SerialMonitorInterface.print("\tz: "); SerialMonitorInterface.println(z);
}

// prints the specified data label followed by the provided x, y, z values (but in degrees)
void displayDegrees(const char *label, float x, float y, float z) {
  SerialMonitorInterface.print(label);
  SerialMonitorInterface.print("\tx: "); SerialMonitorInterface.println(x * RTMATH_RAD_TO_DEGREE);
  SerialMonitorInterface.print("\ty: "); SerialMonitorInterface.println(y * RTMATH_RAD_TO_DEGREE);
  SerialMonitorInterface.print("\tz: "); SerialMonitorInterface.println(z * RTMATH_RAD_TO_DEGREE);
}

// initialize the pressure sensor
void initPressSensor() {
  SerialMonitorInterface.println("Initializing Pressure Sensor... ");

  if (!bmp.begin()) {
    SerialMonitorInterface.println("\tBMP Init Failed!");
    while (1);
  }
  else SerialMonitorInterface.println("\tSuccess!");

  bmp.setOversampling(4);
}

// read values from pressure sensor and update variables
void readPressSensor() {
  char result = bmp.startMeasurment();

  if (result != 0) {
    delay(result);
    result = bmp.getTemperatureAndPressure(T, P);

    if (result != 0) {
      A = bmp.altitude(P, P0);

      printPressSensorPressure();
      printPressSensorTemperature();
      printPressSensorAltitude();
    } else {
      int debug = bmp.getError();
      SerialMonitorInterface.println(String(debug));
    }
  } else {
    SerialMonitorInterface.println("\nPressure Sensor Error.");
  }
}

// print barometric pressure value to SerialMonitorInterface
void printPressSensorPressure() {
  SerialMonitorInterface.println("\nPRESSURE DATA");
  SerialMonitorInterface.print("P = "); SerialMonitorInterface.print(P, 2); SerialMonitorInterface.println(" mBar\t");
}

// print temperature value (from pressure sensor) to SerialMonitorInterface
void printPressSensorTemperature() {
  SerialMonitorInterface.println("\nTEMPERATURE DATA (PRESSURE SENSOR)");
  SerialMonitorInterface.print("T = "); SerialMonitorInterface.print(T, 2); SerialMonitorInterface.println("°C\t");
}

// print altitude value to SerialMonitorInterface
void printPressSensorAltitude() {
  SerialMonitorInterface.println("\nALTITUDE DATA");
  SerialMonitorInterface.print("A = "); SerialMonitorInterface.print(A, 2); SerialMonitorInterface.println(" m");
}

// initialize the ambient light sensor
void TSL2572init(uint8_t gain) {
  Tsl2572RegisterWrite( 0x0F, gain );//set gain
  Tsl2572RegisterWrite( 0x01, 0xED );//51.87 ms
  Tsl2572RegisterWrite( 0x00, 0x03 );//turn on
  if (GAIN_DIVIDE_6)
    Tsl2572RegisterWrite( 0x0D, 0x04 );//scale gain by 0.16
  if (gain == GAIN_1X)gain_val = 1;
  else if (gain == GAIN_8X)gain_val = 8;
  else if (gain == GAIN_16X)gain_val = 16;
  else if (gain == GAIN_120X)gain_val = 120;
}

// writes the specified byte to the specified address of the TSL2572
void Tsl2572RegisterWrite( byte regAddr, byte regData ) {
  Wire.beginTransmission(TSL2572_I2CADDR);
  Wire.write(0x80 | regAddr);
  Wire.write(regData);
  Wire.endTransmission();
}

// read values from ambient light sensor and update variable
float Tsl2572ReadAmbientLight() {
  uint8_t data[4];
  int c0, c1;
  float lux1, lux2, cpl;

  Wire.beginTransmission(TSL2572_I2CADDR);
  Wire.write(0xA0 | 0x14);
  Wire.endTransmission();
  Wire.requestFrom(TSL2572_I2CADDR, 4);
  for (uint8_t i = 0; i < 4; i++)
    data[i] = Wire.read();

  c0 = data[1] << 8 | data[0];
  c1 = data[3] << 8 | data[2];

  //see TSL2572 datasheet
  cpl = 51.87 * (float)gain_val / 60.0;
  if (GAIN_DIVIDE_6) cpl /= 6.0;
  lux1 = ((float)c0 - (1.87 * (float)c1)) / cpl;
  lux2 = ((0.63 * (float)c0) - (float)c1) / cpl;
  cpl = max(lux1, lux2);
  return max(cpl, 0.0);
}

// print illuminance value to SerialMonitorInterface
void printLight() {
  SerialMonitorInterface.println("\nILLUMINANCE DATA");
  SerialMonitorInterface.print(AmbientLightLux);
  SerialMonitorInterface.println(" lux");
}
