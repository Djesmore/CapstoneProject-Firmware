// config.h
#ifndef CONFIG_H
#define CONFIG_H

// Define other pins, constants, and settings here

// Define the pins for the four ultrasonic sensors
const int frTrigPin = 28;  // Front-right sensor Trig pin
const int frEchoPin = 27;  // Front-right sensor Echo pin
const int flTrigPin = 0;  // Front-left sensor Trig pin
const int flEchoPin = 1;  // Front-left sensor Echo pin
const int brTrigPin = 16;  // Back-right sensor Trig pin
const int brEchoPin = 17;  // Back-right sensor Echo pin
const int blTrigPin = 14;  // Back-left sensor Trig pin
const int blEchoPin = 15;  // Back-left sensor Echo pin


// Drive Motor A
const int enA = 20; //PWM
const int in1 = 19; //+++
const int in2 = 18; //---

// Drive Motor B
const int enB = 4; //PWM
const int in3 = 2; //+++
const int in4 = 3; //---

// Leadscrew
const int lsenA = 98; //PWM
const int lsin1 = 99; //+++ 
const int lsin2 = 97; //---

#endif