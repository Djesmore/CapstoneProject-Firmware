#ifndef MAGNETOMETER_H
#define MAGNETOMETER_H
#include <Arduino.h>
#include <HMC5883L.h>

class Magnetometer {
public:
  Magnetometer();
  void begin();
  void loop();
  
private:
  HMC5883L compass;
  int error;
  float declinationAngle;
  
  void Output(MagnetometerRaw raw, MagnetometerScaled scaled, float heading, float headingDegrees);
};

#endif