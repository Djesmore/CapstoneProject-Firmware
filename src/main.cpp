#include <Arduino.h>
//#include <compass.h>
#include <config.h>
#include <ultrasonic.h>
#include <motor_control.h>
#include <wire.h>
#include <DFRobot_QMC5883.h>

//Robot speed is 23.37cm/s on carpet

//Create instance of Compass Class
//Compass compass;
//DFRobot_QMC5883 compass(&Wire, /*I2C addr*/QMC5883_ADDRESS);

//Create instances of the UltrasonicSensor class for each sensor
UltrasonicSensor frontSensor(frontTrigPin, frontEchoPin);
UltrasonicSensor leftSensor(leftTrigPin, leftEchoPin);
UltrasonicSensor rightSensor(rightTrigPin, rightEchoPin);
UltrasonicSensor backSensor(backTrigPin, backEchoPin);

//Create instance of the MotorControl class
MotorControl mtrctrl(enA, in1, in2, enB, in3, in4, lsenA, lsin1, lsin2);

//Create enumerator for robot states
enum RobotState { DRIVING, OBSTACLE_DETECTED };
//Set current (Initial) robot state to planning
RobotState currentState = DRIVING;

// Variables for measurements and navigation
float frontDistance; // Variable to store front distance measurement
float leftDistance; // Variable to store left distance measurement
bool obstacleDetected = false; // Flag to track obstacle presence
unsigned long startTime = 0; // Variable to store the start time of movement
float elapsedTime; // Variable to store the elapsed time during movement

const int NUM_READINGS = 5;
float readings[NUM_READINGS];
int index = 0;
int minDistance = 20;

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

/*
    //compass.initializeCompass(); //Initialize compass
    compass.begin();
    compass.setRange(QMC5883_RANGE_2GA);
    Serial.print("compass range is:");
    Serial.println(compass.getRange());

    compass.setMeasurementMode(QMC5883_CONTINOUS);
    Serial.print("compass measurement mode is:");
    Serial.println(compass.getMeasurementMode());

    compass.setDataRate(QMC5883_DATARATE_50HZ);
    Serial.print("compass data rate is:");
    Serial.println(compass.getDataRate());

    compass.setSamples(QMC5883_SAMPLES_8);
    Serial.print("compass samples is:");
    Serial.println(compass.getSamples());
    float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / PI);
    compass.setDeclinationAngle(declinationAngle);
    sVector_t mag = compass.readRaw();
    compass.getHeadingDegrees();
    Serial.print("Initial Heading: Degress = ");
    Serial.println(mag.HeadingDegress);
*/
}

void obstacleCheck() {
    float sum = 0;
    float average = 0;

    // Collect readings
    readings[index] = frontSensor.readDistance();
    index = (index + 1) % NUM_READINGS;

    // Calculate sum of readings
    for (int i = 0; i < NUM_READINGS; i++) {
        sum += readings[i];
    }

    // Calculate average
    average = sum / NUM_READINGS;

    // Check if the average reading is within a valid range
    if (average <= minDistance) {
        Serial.println("Invalid or too close/far reading!");
        Serial.println(" Distance: ");
        Serial.println(average);
        return;
    }

    // Proceed if the average reading is within the valid range
    if (average <= obstacleThreshold) {
        Serial.println("Obstacle detected!");
        obstacleDetected = true;
        mtrctrl.fullStop();
    }
}

void driveForSeconds(float seconds) {
    Serial.print("Entering DFS...");

    startTime = millis();
    elapsedTime = 0;
    mtrctrl.moveForward(); // Start moving forward
    
    //Seconds variable here is the Drive Time
    while (elapsedTime < seconds * 1000) {
        obstacleCheck();
        if (obstacleDetected == true) {
            mtrctrl.fullStop();
            float currentDistance = frontSensor.readDistance();
            unsigned long stopTime = millis();

            Serial.println("Obstacle detected(dfs)!");
            Serial.println("Obstacle distance: ");
            Serial.print(currentDistance);

            // Wait for obstacle clearance
            while (currentDistance <= obstacleThreshold) {
                delay(1000); // Check every second
                currentDistance = frontSensor.readDistance();
            }

            // Update elapsed time considering the obstacle delay
            elapsedTime += millis() - stopTime;

            Serial.println("Resuming after obstacle. Remaining time: ");
            Serial.println((seconds * 1000 - elapsedTime) / 1000);
            
            mtrctrl.moveForward(); // Resume movement after obstacle delay
            startTime = millis();
        }
        elapsedTime = millis() - startTime;
    }
    mtrctrl.fullStop(); // Stop after the specified duration
    Serial.print("Exiting DFS...");
}

void plowDriveway() {

    // Preset distances for testing
    // Read distances here
    //frontDistance = frontSensor.readDistance();
    //leftDistance = leftSensor.readDistance();

    frontDistance = 121; // Preset front distance for testing
    leftDistance = 220; // Preset left distance for testing
    bool isLeftTurn = true;

    Serial.print("Initial Front Distance: ");
    Serial.println(frontDistance);
    Serial.print("Initial left Distance: ");
    Serial.println(leftDistance);

    int numRows = int(leftDistance / widthOfSnowplow); // Conversion truncates decimal part
    Serial.print("Initial Rows: ");
    Serial.println(numRows);

    obstacleCheck();

    //**** Logic for tracking rows****//
    for (int row = 0; row < numRows; ++row) {
        Serial.print("Remaining rows: ");
        Serial.println(numRows - row);

        // Calculate drive time based on front distance and robot speed
        float timeToDrive = frontDistance / sor;
        Serial.print("Driving forward for time: ");
        Serial.println(timeToDrive);
        driveForSeconds(timeToDrive); //sends drive time to the driveForSeconds function, which takes a float value in seconds 

        delay(1000);

        obstacleCheck();
        Serial.println("Small Push!");
        mtrctrl.endOfRowPush();

        delay(1000);

        Serial.println("Checking for Obstacles before turn...");
        obstacleCheck();
        Serial.print("Path is Clear!");

        if (isLeftTurn) {
            Serial.println("Performing left turn...");
            mtrctrl.turnLeft();
            mtrctrl.fullStop();
            delay(1000);
        } else {
            Serial.println("Performing right turn...");
            mtrctrl.turnRight();
            mtrctrl.fullStop();
            delay(1000);
        }
        isLeftTurn = !isLeftTurn;

        // Check for obstacles after the turn
        delay(1000); // Give time for turn completion
        Serial.print("Checking for Obstacles after turn...");
        obstacleCheck();
        Serial.print("Path is Clear, begin next row!");
    }
}

void demoMode(){

    mtrctrl.fullStop();
    mtrctrl.moveForward();
    delay(2000); // 2s
    mtrctrl.fullStop();
    delay(1000); //1s
    mtrctrl.moveBackward();
    delay(2000);
    mtrctrl.fullStop();

    mtrctrl.plowUp();
    delay(2000);
    mtrctrl.fullStop();
    delay(1000);
    mtrctrl.plowDown();

    mtrctrl.fullStop();

    mtrctrl.demoTurning();

    mtrctrl.fullStop();
    mtrctrl.fullStop();
   
}

void loop() {
//*****Test Code*******

//*****************************************
    switch (currentState) {

        case DRIVING:
            Serial.println("Beginning DRIVING mode...");
            // Any initialization/setup before starting plowDriveway()

            currentState = DRIVING;
            startTime = millis(); // Start time after planning
            plowDriveway();

            Serial.println("Finished Plowing!");
            Serial.println("Awaiting User Intervention...");

            delay(100000000000); // Long delay for end of plowing
            break;

        case OBSTACLE_DETECTED:
            Serial.println("Obstacle Detected!");
            float currentDistance = frontSensor.readDistance();

            if (currentDistance > obstacleThreshold) {
                // Obstacle removed, resume previous state (DRIVING)
                currentState = DRIVING;
                Serial.println("Switching back to DRIVING mode");
            }
            break;
    }
}