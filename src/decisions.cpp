// decision.cpp
#include "decisions.h"
#include "ultrasonic.h"
#include "motor_control.h"
#include "decisions.h"

Decisions::Decisions(UltrasonicSensor& frontSensor, float obstacleThreshold)
    : frontSensor(frontSensor), obstacleThreshold(obstacleThreshold) {}

bool Decisions::avoidObstacle() {
    // Read distance from the front ultrasonic sensor
    float frontDistance = frontSensor.readDistance();

    // Check if the front distance is less than the obstacle threshold
    if (frontDistance < obstacleThreshold) {
        return true;  // An obstacle is too close
    } else {
        return false; // No obstacle is too close
    }
}