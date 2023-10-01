// sensors.h
#ifndef ULTRASONIC_H
#define ULTRASONIC_H
#include <Arduino.h>

class UltrasonicSensor{
    public:
        UltrasonicSensor(int trigPin, int echoPin);
        void initialize();
        float readDistance();

    private: 
        int trigPin;
        int echoPin;
};
#endif