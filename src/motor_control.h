// motor_control.h
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

class MotorControl {
public:
    MotorControl(int enA, int in1, int in2, 
                 int enB, int in3, int in4);
    void initializeMotors();
    void moveForward();
    void moveBackward();
    void turnLeft();
    void turnRight();
    void fullStop();
private:
    // Drive Motor A
    int enA;
    int in1;
    int in2;
    // Drive Motor B
    int enB;
    int in3;
    int in4; 
};
#endif
