// sensors.cpp

#include "sensors.h"
#include <Arduino.h>

#include <Wire.h>
#include <QMC5883L.h> 

QMC5883L compass;

void setup(){
    Wire.begin();
    Serial.begin(9600);
    compass.init();
}

void initializeSensors() {
    // Initialize sensor pins and settings
}

void loop(){
    int x, y, z;

    compass.read(&x, &y, &z);

    
  float headingRadians = atan2(y, x);
  float headingDegrees = headingRadians * 180 / PI;
  float declinationAngle = 11.41666666666667;

  headingDegrees += declinationAngle;

  if (headingDegrees < 0) {
    headingDegrees += 360;
  }

  if (headingDegrees > 348.75 || headingDegrees < 11.25) {
    cardinal = " N";
  }
  else if (headingDegrees > 11.25 && headingDegrees < 33.75) {
    cardinal = " NNE";
  }
  else if (headingDegrees > 33.75 && headingDegrees < 56.25) {
    cardinal = " NE";
  }
  else if (headingDegrees > 56.25 && headingDegrees < 78.75) {
    cardinal = " ENE";
  }
  else if (headingDegrees > 78.75 && headingDegrees < 101.25) {
    cardinal = " E";
  }
  else if (headingDegrees > 101.25 && headingDegrees < 123.75) {
    cardinal = " ESE";
  }
  else if (headingDegrees > 123.75 && headingDegrees < 146.25) {
    cardinal = " SE";
  }
  else if (headingDegrees > 146.25 && headingDegrees < 168.75) {
    cardinal = " SSE";
  }
  else if (headingDegrees > 168.75 && headingDegrees < 191.25) {
    cardinal = " S";
  }
  else if (headingDegrees > 191.25 && headingDegrees < 213.75) {
    cardinal = " SSW";
  }
  else if (headingDegrees > 213.75 && headingDegrees < 236.25) {
    cardinal = " SW";
  }
  else if (headingDegrees > 236.25 && headingDegrees < 258.75) {
    cardinal = " WSW";
  }
  else if (headingDegrees > 258.75 && headingDegrees < 281.25) {
    cardinal = " W";
  }
  else if (headingDegrees > 281.25 && headingDegrees < 303.75) {
    cardinal = " WNW";
  }
  else if (headingDegrees > 303.75 && headingDegrees < 326.25) {
    cardinal = " NW";
  }
  else if (headingDegrees > 326.25 && headingDegrees < 348.75) {
    cardinal = " NNW";
  }

  Serial.print("Heading: ");
  Serial.print(headingDegrees);
  Serial.println(cardinal);

  delay(250);
}

float readUltrasonicSensor() {
    // Code to read data from the ultrasonic sensor
}

float readMagnetometer() {
    // Code to read data from the magnetometer
}