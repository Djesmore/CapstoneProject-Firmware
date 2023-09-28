#include <Arduino.h>
#include <config.h>
#include <sensors.h>
//#include <motor_control.h>
//#include <decisions.h>

/*
// Define motor control pins on the L298n Motor Driver
const int motorA1 = 2; // Motor A input 1
const int motorA2 = 3; // Motor A input 2
const int motorB1 = 4; // Motor B input 1
const int motorB2 = 5; // Motor B input 2
*/

// Create instances of the UltrasonicSensor class for each sensor
UltrasonicSensor frSensor(frTrigPin, frEchoPin);
UltrasonicSensor flSensor(flTrigPin, flEchoPin);
UltrasonicSensor brSensor(brTrigPin, brEchoPin);
UltrasonicSensor blSensor(blTrigPin, blEchoPin);

void setup() {

    pinMode(LED_BUILTIN, OUTPUT);

    // Initialize robot components here
    frSensor.initialize();  // Initialize the front-right sensor
    flSensor.initialize();  // Initialize the front-left sensor
    brSensor.initialize();  // Initialize the back-right sensor
    blSensor.initialize();  // Initialize the back-left sensor
    Serial.begin(9600);     // Initialize serial communication for debugging
}

void loop() {
    // Robot's main control logic goes here
    digitalWrite(LED_BUILTIN, HIGH); 
    //
    //
    // Read distances from the ultrasonic sensors
    float frDistance = frSensor.readDistance();
    delay(10);
    float flDistance = flSensor.readDistance();
    delay(10);
    float brDistance = brSensor.readDistance();
    delay(10);
    float blDistance = blSensor.readDistance();
    delay(10);

    // Print the distances to the serial monitor
    Serial.print("Front-Right Distance: ");
    Serial.print(frDistance);
    Serial.println(" cm");

    Serial.print("Front-Left Distance: ");
    Serial.print(flDistance);
    Serial.println(" cm");

    Serial.print("Back-Right Distance: ");
    Serial.print(brDistance);
    Serial.println(" cm");

    Serial.print("Back-Left Distance: ");
    Serial.print(blDistance);
    Serial.println(" cm");

    // Add your robot's control logic here based on the distances
    // For example, implement collision avoidance using 'frDistance', 'flDistance', etc.
    digitalWrite(LED_BUILTIN, LOW);
    // Add any necessary delays or control logic for your robot's behavior
    delay(1000);  // Adjust the delay as needed
}