// sensors.cpp
#include "sensors.h"
#include <Arduino.h>

// Define the pins for the ultrasonic sensor
const int trigPin = 0;  // Trig pin
const int echoPin = 1;  // Echo pin

void initializeSensors() {
    // Initialize sensor pins
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
}

float readUltrasonicSensor() {
    // Trigger the ultrasonic sensor by sending a 10us pulse on the trigger pin
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Read the echo pulse duration in microseconds
    long duration = pulseIn(echoPin, HIGH);

    // Calculate the distance in centimeters
    // Speed of sound is approximately 343 m/s or 34,300 cm/s
    float distance = (duration * 0.034 / 2);

    return distance;
}