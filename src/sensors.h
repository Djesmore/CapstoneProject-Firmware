// sensors.h
#ifndef SENSORS_H
#define SENSORS_H
#include <Arduino.h>

//void initializeSensors();
float readMagnetometer();

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