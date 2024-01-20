/* MS5607.cpp
   Author: Amit Ate
   Email: amit@uravulabs.com
   Company: Uravu Labs
   Website: http://www.uravulabs.com
   Created: 2016-01-01
   Edited: 2021 by Adam Marciniak
   Modified the library to be non-blocking. Original has a delay of 3 millisec.
   This is not acceptable for a flight controller. The library is now using the
   millis() function to check if the conversion is done. This way the main loop
   can do other things while the conversion is running.
*/

#include <math.h>
#include "MS5607.h"
#include <Wire.h>

unsigned long convElapsed = 0;

// Function: MS5607 (constructor)
// Input: float *altitudeVariable (pointer to a variable where altitude data will be stored)
// Return: None
// Description: Constructor for MS5607 class, initializes internal variables.
MS5607::MS5607(float *altitudeVariable)
{
  this->internal_altitude = altitudeVariable; // Pointer to store altitude data.
}

// Function: MS5607 (constructor)
// Input: short address (I2C address of the MS5607 sensor)
// Return: None
// Description: Constructor for MS5607 class, sets the I2C address of the sensor.
MS5607::MS5607(short address)
{
  this->MS5607_ADDR = address; // I2C address of the MS5607 sensor.
}

// Function: begin
// Input: None
// Return: char (0 for failure, 1 for success)
// Description: Initializes the MS5607 sensor by reading calibration data.
char MS5607::begin()
{
  Wire1.begin();
  return (readCalibration());
}

// Function: resetDevice
// Input: None
// Return: char (0 for failure, 1 for success)
// Description: Resets the MS5607 device and waits for internal register reload.
char MS5607::resetDevice(void)
{
  Wire1.beginTransmission(MS5607_ADDR);
  Wire1.write(RESET);
  char error = Wire1.endTransmission();
  if (error == 0)
  {
    delay(3); // wait for internal register reload
    return (1);
  }
  else
  {
    return (0);
  }
}

// Function: readCalibration
// Input: None
// Return: char (0 for failure, 1 for success)
// Description: Reads calibration data from the PROM of the MS5607 sensor.
char MS5607::readCalibration()
{
  if (resetDevice() &&
      readUInt_16(PROM_READ + 2, C1) &&
      readUInt_16(PROM_READ + 4, C2) &&
      readUInt_16(PROM_READ + 6, C3) &&
      readUInt_16(PROM_READ + 8, C4) &&
      readUInt_16(PROM_READ + 10, C5) &&
      readUInt_16(PROM_READ + 12, C6))
  {
    return (1);
  }
  else
  {
    return (0);
  }
}

// Function: readUInt_16
// Input: char address, unsigned int &value
// Return: char (0 for failure, 1 for success)
// Description: Reads an unsigned 16-bit integer from the specified address.
char MS5607::readUInt_16(char address, unsigned int &value)
{
  unsigned char data[2]; // 4bit
  data[0] = address;
  if (readBytes(data, 2))
  {
    value = (((unsigned int)data[0] * (1 << 8)) | (unsigned int)data[1]);
    return (1);
  }
  value = 0;
  return (0);
}

// Function: readBytes
// Input: unsigned char *values, char length
// Return: char (0 for failure, 1 for success)
// Description: Reads a specified number of bytes over I2C from the MS5607 sensor.
char MS5607::readBytes(unsigned char *values, char length)
{
  char x;

  Wire1.beginTransmission(MS5607_ADDR);
  Wire1.write(values[0]);

  char error = Wire1.endTransmission();
  if (error == 0)
  {
    Wire1.requestFrom(MS5607_ADDR, length);
    while (!Wire1.available())
      ; // wait until bytes are ready
    for (x = 0; x < length; x++)
    {
      values[x] = Wire1.read();
    }
    return (1);
  }
  return (0);
}

// Function: startMeasurment
// Input: None
// Return: char (0 for failure, 1 for success)
// Description: Sends a command to start measurement and records the start time.
char MS5607::startMeasurment(void)
{
  Wire1.beginTransmission(MS5607_ADDR);
  Wire1.write(R_ADC);
  char error = Wire1.endTransmission();
  if (error == 0)
  {
    convElapsed = millis(); // Record the start time of the measurement.
    return (1);
  }
  else
  {
    return (0);
  }
}

// Function: startConversion
// Input: char CMD
// Return: char (0 for failure, 1 for success)
// Description: Sends a command to start conversion of temperature or pressure.
char MS5607::startConversion(char CMD)
{
  Wire1.beginTransmission(MS5607_ADDR);
  Wire1.write(CMD);
  char error = Wire1.endTransmission();
  if (error == 0)
  {
    convElapsed = millis(); // Record the start time of the conversion.
    return (1);
  }
  else
  {
    return (0);
  }
}

// Function: isDataAvailable
// Input: None
// Return: bool (true if data is available, false otherwise)
// Description: Checks if data is available and resets the flag.
bool MS5607::isDataAvailable()
{
  if (dataAvailable)
  {
    dataAvailable = false; // Reset the data availability flag.
    return true;
  }
  else
  {
    return false;
  }
}

// Function: handleAltimeter
// Input: None
// Return: int (1 for success, 0 for failure)
// Description: Handles the altimeter data processing in stages.
int MS5607::handleAltimeter(void)
{
  if (state == 0)
  {
    startConversion(CONV_D1);
    state = 1;
  }

  if (state == 1 && (millis() - convElapsed) > 10)
  {
    startMeasurment();
    state = 2;
  }

  if (state == 2 && (millis() - convElapsed) > 3)
  {
    getDigitalValue(DP);
    state = 3;
  }

  if (state == 3)
  {
    startConversion(CONV_D2);
    state = 4;
  }

  if (state == 4 && (millis() - convElapsed) > 10)
  {
    startMeasurment();
    state = 5;
  }

  if (state == 5 && (millis() - convElapsed) > 3)
  {
    getDigitalValue(DT);
    state = 6;
  }

  if (state == 6)
  {
    *internal_altitude = getAltitude(); // Store calculated altitude in the provided variable.
    dataAvailable = true; // Set the data availability flag.
    state = 0;
    return 1;
  }
  return 0;
}

// Function: readDigitalValue
// Input: None
// Return: char (0 for failure, 1 for success)
// Description: Reads raw digital values of temperature and pressure from MS5607.
char MS5607::readDigitalValue(void)
{
  if (startConversion(CONV_D1))
  {
    if (startMeasurment())
    {
      if (getDigitalValue(DP))
        ;
    }
  }
  else
  {
    return 0;
  }
  if (startConversion(CONV_D2))
  {
    if (startMeasurment())
    {
      if (getDigitalValue(DT))
        ;
    }
  }
  else
  {
    return 0;
  }
  return 1;
}

// Function: getDigitalValue
// Input: unsigned long &value
// Return: char (0 for failure, 1 for success)
// Description: Reads raw digital values of temperature or pressure from MS5607.
char MS5607::getDigitalValue(unsigned long &value)
{
  char x, length = 3;
  unsigned char data[3];
  Wire1.requestFrom(MS5607_ADDR, length);
  while (!Wire1.available())
    ; // wait until bytes are ready
  for (x = 0; x < length; x++)
  {
    data[x] = Wire1.read();
  }
  value = (unsigned long)data[0] * 1 << 16 | (unsigned long)data[1] * 1 << 8 | (unsigned long)data[2];
  return (1);
}

// Function: getTemperature
// Input: None
// Return: float (temperature value)
// Description: Calculates and returns the temperature from raw digital values.
float MS5607::getTemperature(void)
{
  dT = (float)DT - ((float)C5) * ((int)1 << 8);
  TEMP = 2000.0 + dT * ((float)C6) / (float)((long)1 << 23);
  return TEMP / 100;
}

// Function: getPressure
// Input: None
// Return: float (pressure value)
// Description: Calculates and returns the pressure from raw digital values.
float MS5607::getPressure(void)
{
  dT = (float)DT - ((float)C5) * ((int)1 << 8);
  TEMP = 2000.0 + dT * ((float)C6) / (float)((long)1 << 23);
  OFF = (((int64_t)C2) * ((long)1 << 17)) + dT * ((float)C4) / ((int)1 << 6);
  SENS = ((float)C1) * ((long)1 << 16) + dT * ((float)C3) / ((int)1 << 7);
  float pa = (float)((float)DP / ((long)1 << 15));
  float pb = (float)(SENS / ((float)((long)1 << 21)));
  float pc = pa * pb;
  float pd = (float)(OFF / ((float)((long)1 << 15)));
  P = pc - pd;
  return P / 100;
}

// Function: setOSR
// Input: short OSR_U
// Return: None
// Description: Sets the oversampling rate and corresponding values for conversion commands & delay.
void MS5607::setOSR(short OSR_U)
{
  this->OSR = OSR_U; // Set the oversampling rate.
  switch (OSR)
  {
  case 256:
    CONV_D1 = 0x40;
    CONV_D2 = 0x50;
    Conv_Delay = 1;
    break;
  case 512:
    CONV_D1 = 0x42;
    CONV_D2 = 0x52;
    Conv_Delay = 2;
    break;
  case 1024:
    CONV_D1 = 0x44;
    CONV_D2 = 0x54;
    Conv_Delay = 3;
    break;
  case 2048:
    CONV_D1 = 0x46;
    CONV_D2 = 0x56;
    Conv_Delay = 5;
    break;
  case 4096:
    CONV_D1 = 0x48;
    CONV_D2 = 0x58;
    Conv_Delay = 10;
    break;
  default:
    CONV_D1 = 0x40;
    CONV_D2 = 0x50;
    Conv_Delay = 1;
    break;
  }
}

// Function: getAltitude
// Input: None
// Return: float (altitude value)
// Description: Calculates and returns the altitude from temperature and pressure values.
float MS5607::getAltitude(void)
{
  float h, t, p;
  t = getTemperature();
  p = getPressure();
  p = P0 / p;
  h = 153.84615 * (pow(p, 0.19) - 1) * (t + 273.15);
  return h;
}
/**
How to Use:
The provided code is an Arduino library for interfacing with the MS5607 pressure sensor, specifically designed for altitude measurement. Below is a step-by-step explanation of how to use this code:

1. Include the Library
cpp
Copy code
#include <math.h>
#include "MS5607.h"
#include <Wire.h>
Include the necessary libraries for mathematical operations, the MS5607 library, and the Wire library for I2C communication.

2. Initialize the Sensor
cpp
Copy code
MS5607 barometer;
Create an instance of the MS5607 class (object) named barometer.

3. Initialize Altitude Variable
cpp
Copy code
float altitude;
Create a variable to store the calculated altitude.

4. Setup Function
cpp
Copy code
void setup() {
  Serial.begin(9600);
  if (barometer.begin()) {
    Serial.println("MS5607 initialized successfully!");
  } else {
    Serial.println("Error initializing MS5607");
  }
}
In the setup() function, initialize the serial communication and check if the MS5607 sensor is initialized successfully using the begin() method.

5. Loop Function
cpp
Copy code
void loop() {
  if (barometer.isDataAvailable()) {
    // Read altitude data when available
    altitude = barometer.getAltitude();
    Serial.print("Altitude: ");
    Serial.print(altitude);
    Serial.println(" meters");
  }

  // Perform other tasks as needed
  // ...
}
In the loop() function, check if altitude data is available using the isDataAvailable() method. If available, retrieve the altitude using the getAltitude() method and print it to the serial monitor. You can perform other tasks in the loop as needed.

Note:
The library is designed to be non-blocking, allowing the main loop to perform other tasks while waiting for sensor measurements.
The library uses I2C communication, and the sensor must be connected properly to the Arduino board.
Ensure that the MS5607 sensor is connected to the correct I2C pins on the Arduino board.




*/