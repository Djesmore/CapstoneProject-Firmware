#include <Arduino.h>
#include <config.h>
#include <ultrasonic.h>
#include <motor_control.h>
#include <magnetometer.h>
//#include <decisions.h>

//Create instances of the UltrasonicSensor class for each sensor
UltrasonicSensor frSensor(frTrigPin, frEchoPin);
UltrasonicSensor flSensor(flTrigPin, flEchoPin);
UltrasonicSensor brSensor(brTrigPin, brEchoPin);
UltrasonicSensor blSensor(blTrigPin, blEchoPin);

//Create instance of the MotorControl class
MotorControl mtrctrl(enA, in1, in2, enB, in3, in4, lsenA, lsin1, lsin2);

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);

    //Initialize robot components here
    frSensor.initialize();  //Initialize the front-right sensor
    flSensor.initialize();  //Initialize the front-left sensor
    brSensor.initialize();  //Initialize the back-right sensor
    blSensor.initialize();  //Initialize the back-left sensor

    mtrctrl.initializeMotors(); //Initialize drive motors

    Serial.begin(9600);     //Initialize serial communication
}
/*
void determineWorkArea(){
    //Record initial distances
    float frDistance = frSensor.readDistance();
    float flDistance = flSensor.readDistance();
    float brDistance = brSensor.readDistance();
    float blDistance = blSensor.readDistance();
    delay(10);
}
*/
void loop() {
    //Robot's main control logic goes here

    //Turn on Pico's Onboard LED
    digitalWrite(LED_BUILTIN, HIGH); 

    mtrctrl.moveForward();
    delay(1000); 
    mtrctrl.moveBackward();
    delay(1000);
    mtrctrl.turnLeft();
    delay(1000);
    mtrctrl.turnRight();
    delay(1000);
    mtrctrl.fullStop();

    //Read distances from the ultrasonic sensors
    float frDistance = frSensor.readDistance();
    delay(10);
    float flDistance = flSensor.readDistance();
    delay(10);
    float brDistance = brSensor.readDistance();
    delay(10);
    float blDistance = blSensor.readDistance();
    delay(10);

    //Print the distances to the serial monitor
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
  
    //Turn off Pico's Onboard LED
    digitalWrite(LED_BUILTIN, LOW);
  
    delay(1000);
}