#include "magnetometer.h"

Magnetometer::Magnetometer() {
  error = 0;
  declinationAngle = -0.0457; // Replace with your declination angle
}

void Magnetometer::begin() {
  Serial.begin(9600);
  Wire.begin();
  
  Serial.println("Constructing new HMC5883L");
    
  Serial.println("Setting scale to +/- 1.3 Ga");
  error = compass.setScale(1.3);
  if (error != 0)
    Serial.println(compass.getErrorText(error));
  
  Serial.println("Setting measurement mode to continuous.");
  error = compass.setMeasurementMode(MEASUREMENT_CONTINUOUS);
  if (error != 0)
    Serial.println(compass.getErrorText(error));
}

void Magnetometer::loop() {
  MagnetometerRaw raw = compass.readRawAxis();
  MagnetometerScaled scaled = compass.readScaledAxis();
  float heading = atan2(scaled.YAxis, scaled.XAxis);
  
  heading += declinationAngle;
  
  if (heading < 0)
    heading += 2 * PI;
    
  if (heading > 2 * PI)
    heading -= 2 * PI;
   
  float headingDegrees = heading * 180 / M_PI; 

  Output(raw, scaled, heading, headingDegrees);
  delay(66);
}

void Magnetometer::Output(MagnetometerRaw raw, MagnetometerScaled scaled, float heading, float headingDegrees) {
   Serial.print("Raw:\t");
   Serial.print(raw.XAxis);
   Serial.print("   ");   
   Serial.print(raw.YAxis);
   Serial.print("   ");   
   Serial.print(raw.ZAxis);
   Serial.print("   \tScaled:\t");
   
   Serial.print(scaled.XAxis);
   Serial.print("   ");   
   Serial.print(scaled.YAxis);
   Serial.print("   ");   
   Serial.print(scaled.ZAxis);

   Serial.print("   \tHeading:\t");
   Serial.print(heading);
   Serial.print(" Radians   \t");
   Serial.print(headingDegrees);
   Serial.println(" Degrees   \t");
}