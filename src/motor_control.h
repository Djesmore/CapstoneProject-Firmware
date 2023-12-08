// motor_control.h
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

class MotorControl {
public:
    MotorControl(int enA, int in1, int in2, 
                 int enB, int in3, int in4,
                 int lsenA, int lsin1, int lsin2);

    // Stop Function
    void fullStop();
    
    //Raises and lowers the plow after each row
    void endOfRowPush();

    // Demonstrates turning ability
    void demoTurning();

    // Drive Motor Functions             
    void initializeMotors();
    void moveForward();
    void moveBackward();
    void turnLeft();
    void turnRight();

    // Leadscrew Functions
    void plowUp();
    void plowDown();

private:
    // Pins for Motor A, B and Leadscrew
    // Drive Motor A
    int enA; //PWM
    int in1; //+++
    int in2; //---
    // Drive Motor B
    int enB; //PWM
    int in3; //+++
    int in4; //---

    // Leadscrew
    int lsenA; //PWM
    int lsin1; //+++
    int lsin2; //---
};
#endif
