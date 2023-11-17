#include <Arduino.h>
//#include <compass.h>
#include <config.h>
#include <ultrasonic.h>
#include <motor_control.h>
#include <wire.h>
#include <DFRobot_QMC5883.h>
//#include <decisions.h>

//Robot speed is 23.37cm/s on carpet

//Create instance of Compass Class
//Compass compass;
DFRobot_QMC5883 compass(&Wire, /*I2C addr*/QMC5883_ADDRESS);

//Create instances of the UltrasonicSensor class for each sensor
UltrasonicSensor frontSensor(frontTrigPin, frontEchoPin);
UltrasonicSensor leftSensor(leftTrigPin, leftEchoPin);
UltrasonicSensor rightSensor(rightTrigPin, rightEchoPin);
UltrasonicSensor backSensor(backTrigPin, backEchoPin);

//Create instance of the MotorControl class
MotorControl mtrctrl(enA, in1, in2, enB, in3, in4, lsenA, lsin1, lsin2);

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(9600);     //Initialize serial communication

    //Initialize robot components here
    frontSensor.initialize();  //Initialize the front-right sensor
    leftSensor.initialize();  //Initialize the front-left sensor
    rightSensor.initialize();  //Initialize the back-right sensor
    backSensor.initialize();  //Initialize the back-left sensor
    Serial.println("Initializing Ultrasonic Sensors");

    mtrctrl.initializeMotors(); //Initialize drive motors
    Serial.println("Initializing Drive Motors");

    //compass.initializeCompass(); //Initialize compass
/*
Serial.print("test3");
    Serial.println("Initialize QMC5883");
    compass.setRange(QMC5883_RANGE_2GA);
Serial.print("test4");
    Serial.print("compass range is:");
    Serial.println(compass.getRange());
Serial.print("test5");
    compass.setMeasurementMode(QMC5883_CONTINOUS);
    Serial.print("compass measurement mode is:");
    Serial.println(compass.getMeasurementMode());
Serial.print("test6");

    compass.setDataRate(QMC5883_DATARATE_50HZ);
    Serial.print("compass data rate is:");
    Serial.println(compass.getDataRate());

    compass.setSamples(QMC5883_SAMPLES_8);
    Serial.print("compass samples is:");
    Serial.println(compass.getSamples());
*/
    delay(3000);
}

void demoMode(){
    
    //Turn on Pico's Onboard LED
    digitalWrite(LED_BUILTIN, LOW); 

    //*********************** Compass
/*
    int x, y, z;
    compass.readCompass(x, y, z);

    Serial.print("x: ");
    Serial.println(x);
    Serial.print("y: ");
    Serial.println(y);
    Serial.print("z: ");
    Serial.println(z);

    float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / PI);
    compass.setDeclinationAngle(declinationAngle);

    sVector_t mag = compass.readRaw();

    compass.getHeadingDegrees();

    Serial.print("X:");
    Serial.print(mag.XAxis);

    Serial.print(" Y:");
    Serial.print(mag.YAxis);

    Serial.print(" Z:");
    Serial.println(mag.ZAxis);

    Serial.print("Degress = ");

    Serial.println(mag.HeadingDegress);
    delay(100);
*/
    //********************** Drive Motors
    
    mtrctrl.fullStop();
    delay(2000);
    mtrctrl.moveForward();
    Serial.println("Motors Forward");
    delay(5000); 
    mtrctrl.fullStop();

    delay(1500);

    mtrctrl.moveBackward();
    Serial.println("Motors Backwards");
    delay(4000);
    mtrctrl.fullStop();
    delay(1000);
    //46in 5s
    //********************** Ultrasonic Sensors
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


void loop() {
    //Robot's main control logic goes here

    //Read Distance and Drive to Distance

    Serial.println("Reading front distance...");
    float frontDistance = frontSensor.readDistance();

    //Divide frontDistance by Speed of Robot (SOR), 23.37cm/s
    float travelTime = frontDistance / sor;
    float calTravelTime = (travelTime * 1000) - 250; // timing adjustment, arrive just before distance

    Serial.print(" Travel Distance: ");
    Serial.println(frontDistance);

    Serial.print(" Travel Time: ");
    Serial.println(calTravelTime);

    if (frontDistance < 500 && frontDistance > 35){
    Serial.println("Driving!");
    //mtrctrl.moveForward();
    delay(calTravelTime);
    mtrctrl.fullStop();
    }else{
        Serial.print("Distance invalid!");
    }
    delay(2000);
}

