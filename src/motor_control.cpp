/*
// motor_control.cpp
#include "motor_control.h"
#include <Arduino.h> 

// Define pin numbers for the motor control and encoder connections
const int MOTOR_PWM_PIN = 9; 
const int MOTOR_DIRECTION_PIN = 8; 
const int ENCODER_A_PIN = 2; 
const int ENCODER_B_PIN = 3; 

// Define encoder variables
volatile long encoderCount = 0;
volatile bool encoderAState = 0;
volatile bool encoderBState = 0;

MotorController::MotorController() {
    // Constructor
}

void MotorController::initializeMotors() {
    // Initialize motor pins
    pinMode(MOTOR_PWM_PIN, OUTPUT);
    pinMode(MOTOR_DIRECTION_PIN, OUTPUT);

    // Initialize encoder pins
    pinMode(ENCODER_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER_B_PIN, INPUT_PULLUP);

    // Attach interrupts for encoder reading
    attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), updateEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), updateEncoder, CHANGE);
}

void MotorController::moveForward() {
    // Code to move the robot forward
    digitalWrite(MOTOR_DIRECTION_PIN, HIGH); // Set direction forward
    analogWrite(MOTOR_PWM_PIN, 255); // Set motor speed (0-255)
}

void MotorController::moveBackward() {
    // Code to move the robot backward
    digitalWrite(MOTOR_DIRECTION_PIN, LOW); // Set direction backward
    analogWrite(MOTOR_PWM_PIN, 255); // Set motor speed (0-255)
}

void MotorController::turnLeft() {
    // Code to turn the robot left
    // Implement motor control logic for turning left
}

void MotorController::turnRight() {
    // Code to turn the robot right
    // Implement motor control logic for turning right
}

void MotorController::updateEncoder() {
    // Update encoder count based on A and B phase signals
    encoderAState = digitalRead(ENCODER_A_PIN);
    encoderBState = digitalRead(ENCODER_B_PIN);

    if (encoderAState == HIGH) {
        if (encoderBState == LOW) {
            encoderCount++;
        } else {
            encoderCount--;
        }
    } else {
        if (encoderBState == HIGH) {
            encoderCount++;
        } else {
            encoderCount--;
        }
    }
} */