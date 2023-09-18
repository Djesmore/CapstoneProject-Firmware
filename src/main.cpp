#include <Arduino.h>
#include <config.h>
#include <sensors.h>
#include <motor_control.h>
#include <decisions.h>

// put function declarations here:
int myFunction(int, int);

void setup() {
  initializeSensors();  // Initialize the ultrasonic sensor
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  float distance = readUltrasonicSensor();

  // Print the distance to the serial monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

   
    delay(1000);  // Adjust the delay as needed
}
