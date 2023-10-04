#include "magnetometer.h"
#include <HMC5883L.h>
#include <Arduino.h>

HMC5883L Compass;

void initCompass(){
    Compass.SetDeclination(-0, 23, 'W');
    Compass.SetSamplingMode(COMPASS_SINGLE);
    Compass.SetScale(COMPASS_SCALE_130);
    Compass.SetOrientation(COMPASS_HORIZONTAL_X_NORTH);
}

