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

//Create enumerator for robot states
enum RobotState { DRIVING, OBSTACLE_DETECTED };
//Set current (Initial) robot state to planning
RobotState currentState = DRIVING;

// Constants for measurements and calculations
const float widthOfSnowplow = 30.00; // Width of the snowplow in centimeters (1 foot = 30.48 cm)
const float obstacleThreshold = 0; // Obstacle distance threshold in centimeters

// Variables for measurements and navigation
float frontDistance; // Variable to store front distance measurement
float leftDistance; // Variable to store left distance measurement
bool obstacleDetected = false; // Flag to track obstacle presence


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

    int x = frontSensor.readDistance();
    int y = leftSensor.readDistance();
    int z = rightSensor.readDistance();
    int zz = backSensor.readDistance();

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


}

unsigned long startTime = 0; // Variable to store the start time of movement
float elapsedTime; // Variable to store the elapsed time during movement

void driveForSeconds(float seconds) {
    startTime = millis();
    elapsedTime = 0;
    mtrctrl.moveForward(); // Start moving forward
    
    //Seconds variable here is the Drive Time
    while (elapsedTime < seconds * 1000) {
        float currentDistance = frontSensor.readDistance();
        if (currentDistance <= obstacleThreshold) {
            mtrctrl.fullStop();
            unsigned long stopTime = millis();
            Serial.println("Obstacle detected(dfs)!");
            Serial.print("Obstacle distance: ");
            Serial.print(currentDistance);

            // Wait for obstacle clearance
            while (currentDistance <= obstacleThreshold) {
                delay(1000); // Check every second
                currentDistance = frontSensor.readDistance();
            }

            // Update elapsed time considering the obstacle delay
            elapsedTime += millis() - startTime;

            Serial.print("Resuming after obstacle. Remaining time: ");
            Serial.println((seconds * 1000 - elapsedTime) / 1000);
            
            mtrctrl.moveForward(); // Resume movement after obstacle delay
            startTime = millis();
        }
        elapsedTime = millis() - startTime;
    }
    mtrctrl.fullStop(); // Stop after the specified duration
}

void plowDriveway() {

    // Preset distances for testing
    // Read distances here
    frontDistance = 100; // Preset front distance for testing
    leftDistance = 120; // Preset left distance for testing
    bool isLeftTurn = true;

    Serial.print("Initial Front Distance: ");
    Serial.println(frontDistance);
    Serial.print("Initial left Distance: ");
    Serial.println(leftDistance);

    int numRows = int(leftDistance / widthOfSnowplow); // Conversion truncates decimal part
    Serial.print("Initial Rows: ");
    Serial.println(numRows);

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
        mtrctrl.endOfRowPush();
        delay(1000);
        //**************************
        

        float currentDistance = frontSensor.readDistance();
        if (currentDistance <= obstacleThreshold) {
            obstacleDetected = true;
            mtrctrl.fullStop();
            return;
        }

        if (isLeftTurn) {
            Serial.println("Performing left turn...");
            mtrctrl.turnLeft();
            //needs same obstacle avoidance timing as driveForSeconds
        } else {
            Serial.println("Performing right turn...");
            mtrctrl.turnRight();
            //needs same obstacle avoidance timing as driveForSeconds
        }
        isLeftTurn = !isLeftTurn;

        // Check for obstacles after the turn
        delay(1000); // Give time for turn completion

        float obstacleCheckAfterTurn = frontSensor.readDistance();
        
        if (obstacleCheckAfterTurn <= obstacleThreshold) {
            obstacleDetected = true;
            mtrctrl.fullStop();
            return;
        }
    }
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

            delay(100000);

            // Measure front and left distances
            frontDistance = frontSensor.readDistance();
            leftDistance = leftSensor.readDistance();

            if (frontDistance <= obstacleThreshold) {
                currentState = OBSTACLE_DETECTED;
                Serial.println("Switching to OBSTACLE_DETECTED mode");
            }

            // Continue normal operations (driving, plowing)
            // Check for other conditions if needed
            break;

        case OBSTACLE_DETECTED:
            Serial.println("Obstacle Detected!");
            float currentDistance = frontSensor.readDistance();

            if (currentDistance > obstacleThreshold) {
                // Obstacle removed, resume previous state (DRIVING)
                currentState = DRIVING;
                Serial.println("Switching back to DRIVING mode");
            }

            // Wait or perform specific actions related to the obstacle detection
            // Ensure that operations can continue seamlessly after obstacle clearance
            break;
    }

    // Other necessary operations or delays
}