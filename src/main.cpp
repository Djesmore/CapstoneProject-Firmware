#include <Arduino.h>
//#include <compass.h>
#include <config.h>
#include <ultrasonic.h>
#include <motor_control.h>
#include <wire.h>
#include <DFRobot_QMC5883.h>
#include <decisions.h>

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

// Create an instance of the Decisions class
Decisions decision(frontSensor, 25.0); // Adjust the obstacle threshold as needed

float preMeasurements(float, float, float);

float initFrontDist = 0;  // Variable to store the initial front distance
float initLeftDist = 0;   // Variable to store the initial left distance
float calTravelTime; // Variable to store the calibrated travel time
float initFrntMeasureFlag = 0; // Initial Front measurement flag
float initLeftMeasureFlag = 0; // Initial Left measurement flag

bool travelMode = true;     // Initially in travel mode
unsigned long startTime = 0; // Variable to store the start time
unsigned long elapsedTime = 0; // Variable to store the elapsed time


float preMeasurements(float initFrontDist, float initLeftDist, float calTravelTime){

    for(int i = 0; i < 5; i++){
        initFrontDist = frontSensor.readDistance();
        initLeftDist = leftSensor.readDistance();
    }
    Serial.println("Starting Initial Measurements...");
    Serial.print("Test front distance: ");
    Serial.println(initFrontDist);
    Serial.print("Test left distance: ");
    Serial.println(initLeftDist);

//************FRONT SENSOR***************************************************************************

    //Read Front Distance and check to see if measurements were good
    Serial.println("Reading Front distance...");
    initFrontDist = frontSensor.readDistance();
    delay(500);
    float testDist1 = frontSensor.readDistance();
    
    if (initFrontDist < 30) // If initial front distance is less than 30cm, set error flag to 1
    {
        initFrntMeasureFlag = 1;
        Serial.print("Front Measurement less than 30cm!");
    }
    if (initFrontDist > 500) // If initial front distance is greater than 500cm, set error flag to 1
    {
        initFrntMeasureFlag = 1;
        Serial.println("Front Measurement more than 500cm!");
    }
/*
    if (initFrontDist - testDist1 < 10) // If initial front distance is not within 5cm of the test distance, set flag, bad data
    {
        initFrntMeasureFlag = 1;
        Serial.println("Front Measurement does not match test");
    }
*/
    while (initFrntMeasureFlag == 1){
        
        Serial.println("Error with front measurement, retaking measurements now...");
        float testFlag1 = 0;

        initFrontDist = frontSensor.readDistance();
        delay(500);
        float testDist2 = frontSensor.readDistance();
        
        while (testFlag1 == 0){
            if (initFrontDist < 30) // If initial front distance is less than 30cm, set error flag to 1
            {
                testFlag1 = 1;
                Serial.println("New Front Measurement less than 30cm!");
            }
            if (initFrontDist > 500) // If initial front distance is greater than 500cm, set error flag to 1
            {
                testFlag1 = 1;
                Serial.println("New Front Measurement more than 500cm!");
            }
        }
/*
        // Compares new measurement with a test value, to ensure we have a good value. 
        // If the 2 measurements are within 5cm, clear flag
        if (initFrontDist - testDist2 < 10)
        {
            initFrntMeasureFlag = 0;
            Serial.println("New Front Measurement did not match test measurement");
        }
*/
    }

    //************LEFT SENSOR***************************************************************************
/*
    //Read Left Distance and check to see if measurements were good
    Serial.println("Reading Left distance...");
    initLeftDist = leftSensor.readDistance();
    float testDist3 = leftSensor.readDistance();
    
    if (initLeftDist < 30) // If initial Left distance is less than 30cm, set error flag to 1
    {
        initLeftMeasureFlag = 1;
        Serial.print("Left Measurement less than 30cm!");
    }
    if (initLeftDist > 500) // If initial Left distance is greater than 500cm, set error flag to 1
    {
        initLeftMeasureFlag = 1;
        Serial.print("Left Measurement more than 500cm!");
    }
    if (initLeftDist - testDist3 < 5) // If initial Left distance is not within 5cm of the test distance, set flag to 1, bad data
    {
        initLeftMeasureFlag = 1;
        Serial.print("Left Measurement did not match test measurement");
    }

    while (initLeftMeasureFlag == 1){
        
        Serial.print("Error with left measurement, retaking measurements now...");
        float testFlag2 = 0;

        initLeftDist = leftSensor.readDistance();
        delay(500);
        float testDist4 = leftSensor.readDistance();
        
        while (testFlag2 == 0){
            if (initLeftDist < 30) // If initial left distance is less than 30cm, set error flag to 1
            {
                testFlag2 = 1;
                Serial.print("New Left Measurement less than 30cm!");
            }
            if (initLeftDist > 500) // If initial left distance is greater than 500cm, set error flag to 1
            {
                testFlag2 = 1;
                Serial.print("New Left Measurement more than 500cm!");
            }
        }

        // Compares new measurement with a test value, to ensure we have a good value. 
        // If the 2 measurements are within 5cm, clear flag
        if (initFrontDist - testDist4 < 5)
        {
            initLeftMeasureFlag = 0;
            Serial.print("New Left Measurement did not match test");
        }
    }
*/
//*****************************************************************************************************
    //Divide frontDistance by Speed of Robot (SOR), 23.37cm/s
    float travelTime = initFrontDist / sor;
    calTravelTime = (travelTime * 1000) - 250; // timing adjustment, arrive just before distance

    Serial.println("Initial Measurements: Good");

    //Print travel distance and time
    Serial.print(" Travel Distance: ");
    Serial.println(initFrontDist);
    Serial.print(" Travel Time: ");
    Serial.println(travelTime);

    //Return initial front and left distances
    //Return calibrated travel time
    return initFrontDist, initLeftDist, calTravelTime;
}

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

    preMeasurements(initFrontDist, initLeftDist, calTravelTime); //determine initial measurements

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

    //Obstacle Avoidance
    bool obstacleDetected = decision.avoidObstacle();

    if (obstacleDetected == false){

        travelMode = true;
    }
    if (obstacleDetected == true){
        travelMode = false;
    }

    while (travelMode == true){

        Serial.println("Moving Forward!");

        obstacleDetected = decision.avoidObstacle();

        if (startTime == 0) {
            // Set the start time when starting the travel
            startTime = millis();
        }

        mtrctrl.moveForward();

        // Calculate elapsed time
        elapsedTime = millis() - startTime;

        Serial.print("Elapsed Time: ");
        Serial.println(elapsedTime);

         if (elapsedTime >= calTravelTime) {
            mtrctrl.fullStop();
            startTime = 0; // Reset the start time
            Serial.println("Reached the end!");
            } else {
                mtrctrl.moveForward();
                Serial.println("Still Moving!");
            }
    }

    while (travelMode == false){
        mtrctrl.fullStop();
        Serial.println("Obstacle in the way...");

        obstacleDetected = decision.avoidObstacle();

        if (!obstacleDetected) {
            // Switch back to travel mode
            travelMode = true;
            Serial.println("Obstacle Cleared. Switching back to Travel Mode.");
        } else {
            // Continue avoidance behavior
            Serial.println("Avoiding Obstacle...");
        }
    }

    delay(5000);
}

