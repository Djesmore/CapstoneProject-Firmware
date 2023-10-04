// sensors.h
#ifndef ULTRASONIC_H
#define ULTRASONIC_H
#include <Arduino.h>

class UltrasonicSensor{
    public:
        UltrasonicSensor(int trigPin, int echoPin);

        //Ultrasonic Functions
        void initialize();
        float readDistance();

    private: 
        //Ultrasonic Pins
        int trigPin;
        int echoPin;
};
#endif