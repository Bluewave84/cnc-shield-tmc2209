#define SPEED 800
#define ACC 800
#define DELAY 100

#include <FlexyStepper.h>

//
// pin assignments
//
const int LED_PIN = 13;
const int MOTOR_X_STEP_PIN = 2;
const int MOTOR_Y_STEP_PIN = 3;
const int MOTOR_Z_STEP_PIN = 4;
const int MOTOR_X_DIR_PIN = 5;
const int MOTOR_Y_DIR_PIN = 6;
const int MOTOR_Z_DIR_PIN = 7;
const int STEPPERS_ENABLE_PIN = 8;
const int LIMIT_SWITCH_X_PIN = 9;
const int LIMIT_SWITCH_Y_PIN = 10;
const int LIMIT_SWITCH_Z_PIN = 11;

// const int JOYSTICK_X = 12;
// const int JOYSTICK_Y = 13;
#define X_LIMIT_LOWER -3000
#define X_HOME 0
#define X_LIMIT_UPPER 3000

#define Y_LIMIT_LOWER -3000
#define Y_HOME 0
#define Y_LIMIT_UPPER 3000

int position_x = X_HOME;
int position_y = Y_HOME;
//
// create the stepper motor objects
//
FlexyStepper stepperX;
FlexyStepper stepperY;

void setup()
{
    pinMode(12, INPUT);
    pinMode(13, INPUT);
    //
    // setup the LED pin and enable print statements
    //
    pinMode(LED_PIN, OUTPUT);
    pinMode(STEPPERS_ENABLE_PIN, OUTPUT); // be sure to do this
    Serial.begin(9600);

    //
    // connect and configure the stepper motor to there IO pins
    //
    stepperX.connectToPins(MOTOR_X_STEP_PIN, MOTOR_X_DIR_PIN);
    stepperY.connectToPins(MOTOR_Y_STEP_PIN, MOTOR_Y_DIR_PIN);

    //
    // enable the stepper motors
    //
    digitalWrite(STEPPERS_ENABLE_PIN, LOW); // be sure to do this
}

void runStepperX(int steps, int speed = SPEED, int acceleration = ACC, int del = DELAY)
{
    Serial.print("Running X (STEPS: ");
    Serial.print(steps);
    Serial.print(" , SPEED: ");
    Serial.print(speed);
    Serial.print(" , ACCEL: ");
    Serial.print(acceleration);
    Serial.println(")");
    stepperX.setSpeedInStepsPerSecond(speed);
    stepperX.setAccelerationInStepsPerSecondPerSecond(acceleration);
    stepperX.moveRelativeInSteps(steps);

    delay(del);
}

void runStepperY(int steps, int speed = SPEED, int acceleration = ACC, int del = DELAY)
{
    Serial.print("Running Y (STEPS: ");
    Serial.print(steps);
    Serial.print(" , SPEED: ");
    Serial.print(speed);
    Serial.print(" , ACCEL: ");
    Serial.print(acceleration);
    Serial.println(")");
    stepperY.setSpeedInStepsPerSecond(speed);
    stepperY.setAccelerationInStepsPerSecondPerSecond(acceleration);
    stepperY.moveRelativeInSteps(steps);

    delay(del);
}

int mode = 1;
int analog_x, analog_y, control_x, control_y;
void loop()
{

    analog_x = analogRead(4);
    analog_y = analogRead(5);

    // Serial.print("Analog: X=");
    // Serial.print(analog_x);
    // Serial.print(", Y=");
    // Serial.println(analog_y);

    control_x = map(analog_x, 0, 1023, X_LIMIT_LOWER, X_LIMIT_UPPER);
    control_y = map(analog_y, 0, 1023, Y_LIMIT_LOWER, Y_LIMIT_UPPER);

    // Serial.print("Control: X=");
    // Serial.print(control_x);
    // Serial.print(", Y=");
    // Serial.println(control_y);

    int x_move = abs(control_x) - 2 > 0
                     ? control_x * 100
                     : 0;

    int y_move = abs(control_y) - 2 > 0
                     ? control_y * 100
                     : 0;

    moveXYWithCoordination(x_move, y_move, SPEED, ACC, DELAY);
    return;

    switch (mode)
    {
    case 1:
        runStepperX(1000, SPEED, ACC, DELAY);
        runStepperX(-1000, SPEED, ACC, DELAY);
        mode += 1;
        break;

    case 2:
        runStepperY(1000, SPEED, ACC, DELAY);
        runStepperY(-1000, SPEED, ACC, DELAY);
        mode += 1;
        break;

    case 3:
        moveXYWithCoordination(-200, 1000, SPEED, ACC, DELAY);
        moveXYWithCoordination(1000, -200, SPEED, ACC, DELAY);
        moveXYWithCoordination(200, -1000, SPEED, ACC, DELAY);
        moveXYWithCoordination(-1000, 200, SPEED, ACC, DELAY);
        mode = 1;
        break;
    }

    yield;
}

//
// move both X & Y motors together in a coordinated way, such that they each
// start and stop at the same time, even if one motor moves a greater distance
//
void moveXYWithCoordination(long stepsX, long stepsY, float speedInStepsPerSecond, float accelerationInStepsPerSecondPerSecond, int del)
{
    float speedInStepsPerSecond_X;
    float accelerationInStepsPerSecondPerSecond_X;
    float speedInStepsPerSecond_Y;
    float accelerationInStepsPerSecondPerSecond_Y;
    long absStepsX;
    long absStepsY;

    //
    // setup initial speed and acceleration values
    //
    speedInStepsPerSecond_X = speedInStepsPerSecond;
    accelerationInStepsPerSecondPerSecond_X = accelerationInStepsPerSecondPerSecond;

    speedInStepsPerSecond_Y = speedInStepsPerSecond;
    accelerationInStepsPerSecondPerSecond_Y = accelerationInStepsPerSecondPerSecond;

    //
    // determine how many steps each motor is moving
    //
    if (stepsX >= 0)
        absStepsX = stepsX;
    else
        absStepsX = -stepsX;

    if (stepsY >= 0)
        absStepsY = stepsY;
    else
        absStepsY = -stepsY;

    //
    // determine which motor is traveling the farthest, then slow down the
    // speed rates for the motor moving the shortest distance
    //
    if ((absStepsX > absStepsY) && (stepsX != 0))
    {
        //
        // slow down the motor traveling less far
        //
        float scaler = (float)absStepsY / (float)absStepsX;
        speedInStepsPerSecond_Y = speedInStepsPerSecond_Y * scaler;
        accelerationInStepsPerSecondPerSecond_Y = accelerationInStepsPerSecondPerSecond_Y * scaler;
    }

    if ((absStepsY > absStepsX) && (stepsY != 0))
    {
        //
        // slow down the motor traveling less far
        //
        float scaler = (float)absStepsX / (float)absStepsY;
        speedInStepsPerSecond_X = speedInStepsPerSecond_X * scaler;
        accelerationInStepsPerSecondPerSecond_X = accelerationInStepsPerSecondPerSecond_X * scaler;
    }

    //
    // setup the motion for the X motor
    //
    stepperX.setSpeedInStepsPerSecond(speedInStepsPerSecond_X);
    stepperX.setAccelerationInStepsPerSecondPerSecond(accelerationInStepsPerSecondPerSecond_X);
    stepperX.setTargetPositionRelativeInSteps(stepsX);

    //
    // setup the motion for the Y motor
    //
    stepperY.setSpeedInStepsPerSecond(speedInStepsPerSecond_Y);
    stepperY.setAccelerationInStepsPerSecondPerSecond(accelerationInStepsPerSecondPerSecond_Y);
    stepperY.setTargetPositionInSteps(stepsY);

    //
    // now execute the moves, looping until both motors have finished
    //
    while ((!stepperX.motionComplete()) || (!stepperY.motionComplete()))
    {
        stepperX.processMovement();
        stepperY.processMovement();
    }

    delay(del);
}
