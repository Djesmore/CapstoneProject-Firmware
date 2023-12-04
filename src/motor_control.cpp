
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
    analogWrite(enA, 215); // Set PWM **Speed is 23.37cm/s
    analogWrite(enB, 255); // Set PWM **Speed is 23.37cm/s

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

//2x 90DEG TURNS
void MotorControl::turnLeft() {
    analogWrite(enA, 0);
    analogWrite(enB, 255);

    digitalWrite(in1, HIGH);
	digitalWrite(in2, LOW);
	
	digitalWrite(in3, HIGH);
	digitalWrite(in4, LOW);
	delay(1600);

	fullStop();
	delay(2000);

	analogWrite(enA, 0);
    analogWrite(enB, 255);

	digitalWrite(in1, HIGH);
	digitalWrite(in2, LOW);
	
	digitalWrite(in3, HIGH);
	digitalWrite(in4, LOW);
	delay(1625);


}

void MotorControl::turnRight() {
    analogWrite(enA, 255);
    analogWrite(enB, 0);

    digitalWrite(in1, HIGH);
	digitalWrite(in2, LOW);
	
	digitalWrite(in3, HIGH);
	digitalWrite(in4, LOW);
	delay(1600);

	fullStop();
	delay(2000);

	analogWrite(enA, 255);
    analogWrite(enB, 0);

	digitalWrite(in1, HIGH);
	digitalWrite(in2, LOW);
	
	digitalWrite(in3, HIGH);
	digitalWrite(in4, LOW);
	delay(1625);

}

void MotorControl::fullStop() {
    analogWrite(enA, 255);
    analogWrite(enB, 255);
	analogWrite(lsenA, 255); // Set PWM

    digitalWrite(in1, LOW);
	digitalWrite(in2, LOW);

	digitalWrite(in3, LOW);
	digitalWrite(in4, LOW);


	digitalWrite(lsin1, LOW); 
	digitalWrite(lsin2, LOW);
}

void MotorControl::plowUp() {
	analogWrite(lsenA, 255); // Set PWM

	digitalWrite(lsin1, HIGH); //+++
	digitalWrite(lsin2, LOW); //---
}

void MotorControl::plowDown() {
	analogWrite(lsenA, 255); // Set PWM

	digitalWrite(lsin1, LOW); //+++
	digitalWrite(lsin2, HIGH); //---
}

void MotorControl::endOfRowPush() {
	//Stop 
	fullStop();
	delay(2000);

	//Move forward 1s then stop
	moveForward();
	delay(1000);
	fullStop();

	//Raise plow and stop
	plowUp();
	delay(3000);
	fullStop();

	//Move backward 2s then stop
	moveBackward();
	delay(1000);
	fullStop();

	//lower plow and stop
	plowDown();
	delay(3000);
	fullStop();
	fullStop();
	fullStop();
}
