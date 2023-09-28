// motor_control.h
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

class MotorController {
public:
    MotorController();
    void initializeMotors();
    void moveForward();
    void moveBackward();
    void turnLeft();
    void turnRight();
};
#endif