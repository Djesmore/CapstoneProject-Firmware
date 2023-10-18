#include <Arduino.h>
#include <magnetometer.h>
#include <config.h>
#include <ultrasonic.h>
#include <motor_control.h>
#include <wire.h>
//#include <decisions.h>

//Magnetometer magnetometer;

//Create instances of the UltrasonicSensor class for each sensor
UltrasonicSensor frontSensor(frontTrigPin, frontEchoPin);
UltrasonicSensor leftSensor(leftTrigPin, leftEchoPin);
UltrasonicSensor rightSensor(rightTrigPin, rightEchoPin);
UltrasonicSensor backSensor(backTrigPin, backEchoPin);

//Create instance of the MotorControl class
MotorControl mtrctrl(enA, in1, in2, enB, in3, in4, lsenA, lsin1, lsin2);

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);

    //Initialize robot components here

    frontSensor.initialize();  //Initialize the front-right sensor
    leftSensor.initialize();  //Initialize the front-left sensor
    rightSensor.initialize();  //Initialize the back-right sensor
    backSensor.initialize();  //Initialize the back-left sensor

    mtrctrl.initializeMotors(); //Initialize drive motors

 // magnetometer.begin();

    Wire.begin();
    
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
    digitalWrite(LED_BUILTIN, LOW); 

    //magnetometer.loop();

    mtrctrl.fullStop();
    delay(5000);
    mtrctrl.moveForward();
    Serial.println("Motors Forward");
    delay(5000); 
    mtrctrl.fullStop();

    delay(3000);

    mtrctrl.moveBackward();
    Serial.println("Motors Backwards");
    delay(4000);
    mtrctrl.fullStop();
    delay(1000);

    //Read distances from the ultrasonic sensors
    float frontDistance = frontSensor.readDistance();
    delay(10);
    float leftDistance = leftSensor.readDistance();
    delay(10);
    float rightDistance = rightSensor.readDistance();
    delay(10);
    float backDistance = backSensor.readDistance();
    delay(10);

    //Print the distances to the serial monitor
    Serial.print("Front Distance: ");
    Serial.print(frontDistance);
    Serial.println(" cm");

    Serial.print("Left Distance: ");
    Serial.print(leftDistance);
    Serial.println(" cm");

    Serial.print("Right Distance: ");
    Serial.print(rightDistance);
    Serial.println(" cm");

    Serial.print("Back Distance: ");
    Serial.print(backDistance);
    Serial.println(" cm");

    //Turn off Pico's Onboard LED
    digitalWrite(LED_BUILTIN, LOW);
  
    delay(1000);
    delay(1000);
    delay(1000);
}