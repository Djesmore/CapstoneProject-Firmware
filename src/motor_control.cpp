
//motor_control.cpp
#include "motor_control.h"
#include <Arduino.h>

//Constructor
MotorControl::MotorControl(int enA, int in1, int in2, int enB, int in3, int in4, int lsenA, int lsin1, int lsin2) {
	//MTR A
    this->enA = enA;
    this->in1 = in1;
    this->in2 = in2;
	//MTR B
    this->enB = enB;
    this->in3 = in3;
    this->in4 = in4;
	//Leadscrew
	  
	this->lsenA = lsenA;
	this->lsin1 = lsin1;
	this->lsin2 = lsin2;
}

void MotorControl::initializeMotors() {
    // Set all the motor control pins to outputs
	//MTR A
	pinMode(enA, OUTPUT);
	pinMode(in1, OUTPUT);
	pinMode(in2, OUTPUT);
	//MTR B
	pinMode(enB, OUTPUT);
	pinMode(in3, OUTPUT);
	pinMode(in4, OUTPUT);

	//Leadscrew
	pinMode(lsenA, OUTPUT);
	pinMode(lsin1, OUTPUT);
	pinMode(lsin2, OUTPUT);
	
	//Turn off motors - Initial state
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

void MotorControl::leadscrewForward() {
	analogWrite(lsenA, 255); // Set PWM

	digitalWrite(lsin1, HIGH); //+++
	digitalWrite(lsin2, LOW); //---
}

void MotorControl::leadscrewBackward() {
	analogWrite(lsenA, 255); // Set PWM

	digitalWrite(lsin1, LOW); //+++
	digitalWrite(lsin2, HIGH); //---
}
