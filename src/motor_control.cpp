
//motor_control.cpp
#include "motor_control.h"
#include <Arduino.h>

//Constructor
MotorControl::MotorControl(int enA, int in1, int in2, int enB, int in3, int in4){
    this->enA = enA;
    this->in1 = in1;
    this->in2 = in2;
    this->enB = enB;
    this->in3 = in3;
    this->in4 = in4;
}

void MotorControl::initializeMotors() {
    // Set all the motor control pins to outputs
	pinMode(enA, OUTPUT);
	pinMode(enB, OUTPUT);
	pinMode(in1, OUTPUT);
	pinMode(in2, OUTPUT);
	pinMode(in3, OUTPUT);
	pinMode(in4, OUTPUT);
	
	// Turn off motors - Initial state
	digitalWrite(in1, LOW);
	digitalWrite(in2, LOW);
	digitalWrite(in3, LOW);
	digitalWrite(in4, LOW);
}

void MotorControl::moveForward() {
    analogWrite(enA, 255); // Set PWM
    analogWrite(enB, 255); // Set PWM

    digitalWrite(in1, HIGH);
	digitalWrite(in2, LOW);
	digitalWrite(in3, HIGH);
	digitalWrite(in4, LOW);
}

void MotorControl::moveBackward() {
    analogWrite(enA, 255);
    analogWrite(enB, 255);

    digitalWrite(in1, LOW);
	digitalWrite(in2, HIGH);
	digitalWrite(in3, LOW);
	digitalWrite(in4, HIGH);
}

void MotorControl::turnLeft() {
    analogWrite(enA, 255);
    analogWrite(enB, 255);

    digitalWrite(in1, LOW);
	digitalWrite(in2, HIGH);
	digitalWrite(in3, HIGH);
	digitalWrite(in4, LOW);
}

void MotorControl::turnRight() {
    analogWrite(enA, 255);
    analogWrite(enB, 255);

    digitalWrite(in1, HIGH);
	digitalWrite(in2, LOW);
	digitalWrite(in3, LOW);
	digitalWrite(in4, HIGH);
}

void MotorControl::fullStop() {
    analogWrite(enA, 255);
    analogWrite(enB, 255);

    digitalWrite(in1, LOW);
	digitalWrite(in2, LOW);
	digitalWrite(in3, LOW);
	digitalWrite(in4, LOW);
}
