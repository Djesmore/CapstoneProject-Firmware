#ifndef COMPASS_H
#define COMPASS_H
#include <Arduino.h>
#include <Wire.h>
#include <HMC5883L.h>

class Compass{
    public:
        Compass(uint8_t address = 0x1E);

        //Compass Functions
        void initializeCompass();
        void readCompass(int &x, int &y, int &z);

    private: 
        //Compass Variabless
        uint8_t deviceAddress;
};

#endif