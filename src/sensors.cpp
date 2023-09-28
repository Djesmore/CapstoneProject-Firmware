// sensors.cpp
#include "sensors.h"
#include <Arduino.h>

/*
void initializeSensors() {
    // Initialize sensor pins
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
} */

//Constructor
UltrasonicSensor::UltrasonicSensor(int trigPin, int echoPin) {
    this->trigPin = trigPin;
    this->echoPin = echoPin;
}

void UltrasonicSensor::initialize() {
    // Initialize sensor pins
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
}

float UltrasonicSensor::readDistance() {
    // Trigger the ultrasonic sensor by sending a 10us pulse on the trigger pin
    digitalWrite(trigPin, LOW);
    delayMicroseconds(20);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(40);
    digitalWrite(trigPin, LOW);

    // Read the echo pulse duration in microseconds
    long duration = pulseIn(echoPin, HIGH);

    // Calculate the distance in centimeters
    // Speed of sound is approximately 343 m/s or 34,300 cm/s
    long distance = (duration * 0.034 / 2);
    
    return distance;
}
 /*
float readMagnetometer() {
    // Code to read data from the magnetometer
    return distance;
} */