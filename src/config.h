// config.h
#ifndef CONFIG_H
#define CONFIG_H

// Define other pins, constants, and settings here

// Define the pins for the four ultrasonic sensors
const int frontTrigPin = 52;  // Front sensor Trig pin
const int frontEchoPin = 53;  // Front sensor Echo pin
const int leftTrigPin = 50;  // Left sensor Trig pin
const int leftEchoPin = 51;  // Left sensor Echo pin
const int rightTrigPin = 22;  // Right sensor Trig pin
const int rightEchoPin = 23;  // Right sensor Echo pin
const int backTrigPin = 26;  // Back sensor Trig pin
const int backEchoPin = 27;  // Back sensor Echo pin


// Drive Motor A
const int enA = 8; //PWM 20
const int in1 = 9; //+++ 19 
const int in2 = 10; //--- 18

// Drive Motor B
const int enB = 13; //PWM 4
const int in3 = 11; //+++2
const int in4 = 12; //---3

// Leadscrew
const int lsenA = 98; //PWM
const int lsin1 = 99; //+++ 
const int lsin2 = 97; //---

#endif