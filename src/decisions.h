// decision.h
#ifndef DECISION_H
#define DECISION_H
#include "ultrasonic.h"  // Include the UltrasonicSensor class and any other necessary headers

class Decisions {
public:
    Decisions(UltrasonicSensor& frontSensor, float obstacleThreshold);

    bool avoidObstacle();

private:
    UltrasonicSensor& frontSensor;
    float obstacleThreshold;
};
#endif