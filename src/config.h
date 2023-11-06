// config.h
#ifndef CONFIG_H
#define CONFIG_H

// Define other pins, constants, and settings here

//Speed of Robot (SoR) in cm/s
const float sor = 23.37;

// Define the pins for the four ultrasonic sensors
const int frontTrigPin = 50;  // Front sensor Trig pin BLACK
const int frontEchoPin = 51;  // Front sensor Echo pin BLACK
const int leftTrigPin = 24;  // Left sensor Trig pin PINK
const int leftEchoPin = 25;  // Left sensor Echo pin PINK
const int rightTrigPin = 53;  // Right sensor Trig pin GREEN
const int rightEchoPin = 52;  // Right sensor Echo pin GREEN
const int backTrigPin = 22;  // Back sensor Trig pin PURPLE
const int backEchoPin = 23;  // Back sensor Echo pin PURPLE

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