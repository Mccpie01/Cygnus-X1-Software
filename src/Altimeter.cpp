
/* Handeling the Altemiter  
 * MS5607 - Barometric Pressure Sensor
 */
#include "Altimeter.h"

float rawAltitude = 0;
float altitudeBias = 0;
float altitude = 0;
const int biasCount = 100;

MS5607 Altimeter(&rawAltitude);

bool initAltimeter()
{
  char beginState = Altimeter.begin();
  if (!beginState)
  {
    return 0;
  }
  Altimeter.setOSR(4096);

  while (rawAltitude == 0)
  {
    Altimeter.handleAltimeter();
    delay(10);
  }

  Serial.println();
  Serial.print("Init Raw Altitude: ");
  Serial.println(rawAltitude);
  delay(200);
  getAltitudeBias();
  delay(200);
  return true;
}

void handleAltimeter()
{
  Altimeter.handleAltimeter();
}

bool isNewAltimeterData()
{
  if (Altimeter.isDataAvailable())
  {
    return true;
  }
  else
  {
    return false;
  }
}

float getAltitude()
{
  data.altitude = rawAltitude - data.biasAltitude;
  return rawAltitude - altitudeBias;
}

// Updates altitude bias value. This is blocking. Must wait a bit..
void getAltitudeBias()
{
  Serial.println("Biasing Altimeter.");
  int count = 0;
  while (count < biasCount)
  {
    Altimeter.handleAltimeter();
    if (isNewAltimeterData())
    {

      altitudeBias += rawAltitude;
      Serial.print(".");
      count += 1;
    }
  }
  altitudeBias = altitudeBias / ((float)biasCount);
  data.biasAltitude = altitudeBias;
  return;
}

float getMovingAverage(float altitude)
{
  const static int WINDOW_SIZE = 20;
  static int INDEX = 0;
  static float SUM = 0;
  static float READINGS[WINDOW_SIZE];
  static float AVERAGED = 0;
  SUM -= READINGS[INDEX];
  READINGS[INDEX] = altitude;
  SUM += altitude;
  INDEX = (INDEX + 1) % WINDOW_SIZE;
  if (INDEX < 19)
  {
    return 0.0;
  }

  AVERAGED = SUM / WINDOW_SIZE;

  return AVERAGED;
}
