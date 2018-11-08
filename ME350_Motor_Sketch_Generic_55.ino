// ME350 Ball Handling Sketch - Version 03
// created 3-02-2017

//////////////////////////////////////////////
// DEFINE CONSTANTS AND GLOBAL VARIABLES:   //
//////////////////////////////////////////////

//** Color Sensor: **//
// Include the necessary code headers:
#include "Adafruit_TCS34725.h"
#include <Wire.h>
// CONSTANTS: 
// Definition of ball -types:
const int MAIZE = 1; 
const int BLUE  = 2; 
const int RED   = 3; 
const int WHITE = 4; 
const int NONE  = 5; 
// VARIABLES:
// Create a variable that allows us to access the color sensor:
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
// Return values from the sensor
uint16_t red;
uint16_t green;
uint16_t blue;
uint16_t clear;

//** Computation of position and velocity: **//
// CONSTANTS: 
// Settings for velocity computation:
const int MIN_VEL_COMP_COUNT = 2;    // [encoder counts] Minimal change in motor position that must happen between two velocity measurements
const long MIN_VEL_COMP_TIME = 10000; // [microseconds] Minimal time that must pass between two velocity measurements
// VARIABLES:
volatile int motorPosition = 0; // [encoder counts] Current motor position (Declared 'volatile', since it is updated in a function called by interrupts)
volatile int encoderStatus = 0; // [binary] Past and Current A&B values of the encoder  (Declared 'volatile', since it is updated in a function called by interrupts)
// The rightmost two bits of encoderStatus will store the encoder values from the current iteration (A and B).
// The two bits to the left of those will store the encoder values from the previous iteration (A_old and B_old).
float motorVelocity        = 0; // [encoder counts / seconds] Current motor velocity 
int previousMotorPosition  = 0; // [encoder counts] Motor position the last time a velocity was computed 
long previousVelCompTime   = 0; // [microseconds] System clock value the last time a velocity was computed 

//** High-level behavior of the controller:  **//
// CONSTANTS:
// Target positions:
const int WAIT_POSITION       = 520;             // [encoder counts] Motor position corresponding to a wait position near the two chutes
const int CHUTE_1_POSITION    = 0;               // [encoder counts] Motor position corresponding to first chute
const int CHUTE_2_POSITION    = 722;             // [encoder counts] Motor position corresponding to second chute
const int PUT_POSITION        = 1136;             // [encoder counts] Motor position corresponding to basket lane
const int LOWER_BOUND         = CHUTE_1_POSITION; // [encoder counts] Position of the left end stop
const int UPPER_BOUND         = PUT_POSITION;     // [encoder counts] Position of the right end stop
const int TARGET_BAND         = 20;               // [encoder counts] "Close enough" range when moving towards a target.
//FF Balance Encoder Value: 593

// Timing:
const long  WAIT_TIME         = 2000000; // [microseconds] Time waiting for the ball to drop.
// VARIABLES:
int activeChutePosition;     // [encoder counts] position of the currently active chute
unsigned long startWaitTime; // [microseconds] System clock value at the moment the WAIT_FOR_BALL state started

//** Motor Constants  **//
// CONSTANTS:
const float SUPPLY_VOLTAGE = 10.0;     // [Volt] Supply voltage at the HBridge
const float BASE_CMD       = -3.3;     // [Volt] Voltage needed to overcome friction
// VARIABLES:
float desiredVoltage = 0;   // [Volt] Desired motor voltage
int   motorCommand   = 0;   // [0-255] PWM signal sent to the motor
unsigned long executionDuration = 0;      // [microseconds] Time between this and the previous loop execution.  Used for integrals and derivatives
unsigned long lastExecutionTime = 0;  // [microseconds] System clock value at the moment the loop was started the last time

//** Gravity Compensation Lookup Table: **//
// CONSTANTS: 
const float FF_BALANCED_POSITION = 593; // [encoder counts] Position at which the device is fully balanced. 
const float FF_VOLTAGE_LOWER_BOUND = 0;   // [Volt] Voltage to be applied at the left endstop 
const float FF_VOLTAGE_UPPER_BOUND = 0;  // [Volt] Voltage to be applied at the right endstop 

//** Pin assignment: **//
// CONSTANTS:
const int PIN_NR_ENCODER_A        = 2;  // Never change these, since the interrupts are attached to pin 2 and 3
const int PIN_NR_ENCODER_B        = 3;  // Never change these, since the interrupts are attached to pin 2 and 3
const int PIN_NR_DROP_REQ         = 13;
const int PIN_NR_ON_OFF_SWITCH    = 5;
const int PIN_NR_CHUTE_1_READY    = 12;
const int PIN_NR_CHUTE_2_READY    = 11;
const int PIN_NRL_LIMIT_SWITCH    = 8;
const int PIN_NR_PWM_OUTPUT       = 9;
const int PIN_NR_PWM_DIRECTION_1  = 10;
const int PIN_NR_PWM_DIRECTION_2  = 6;
// End of CONSTANTS AND GLOBAL VARIABLES


//////////////////////////////////////////////////////////////////////////////////////////
// The setup() function is called when a sketch starts. Use it to initialize variables, //
// pin modes, start using libraries, etc. The setup function will only run once, after  //
// each powerup or reset of the Arduino board:                                          //
//////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  // Declare which digital pins are inputs and which are outputs:
  pinMode(PIN_NR_ENCODER_A,        INPUT_PULLUP);
  pinMode(PIN_NR_ENCODER_B,        INPUT_PULLUP);
  pinMode(PIN_NR_CHUTE_1_READY,    INPUT); 
  pinMode(PIN_NR_CHUTE_2_READY,    INPUT); 
  pinMode(PIN_NR_ON_OFF_SWITCH,    INPUT);
  pinMode(PIN_NRL_LIMIT_SWITCH,    INPUT);
  pinMode(PIN_NR_DROP_REQ,         OUTPUT);
  pinMode(PIN_NR_PWM_OUTPUT,       OUTPUT);
  pinMode(PIN_NR_PWM_DIRECTION_1,  OUTPUT);
  pinMode(PIN_NR_PWM_DIRECTION_2,  OUTPUT);

  // Turn on the pullup resistors on the encoder channels
  // (the other sensors already have physical resistors on the breadboard) 
  digitalWrite(PIN_NR_ENCODER_A, HIGH);  
  digitalWrite(PIN_NR_ENCODER_B, HIGH);

  // Activate interrupt for encoder pins.
  // If either of the two pins changes, the function 'updateMotorPosition' is called:
  attachInterrupt(0, updateMotorPositionAndVelocity, CHANGE);  // Interrupt 0 is always attached to digital pin 2
  attachInterrupt(1, updateMotorPositionAndVelocity, CHANGE);  // Interrupt 1 is always attached to digital pin 3

  // Begin serial communication for monitoring.
  Serial.begin(115200);
  Serial.println("Start Executing Program.");

  // Begin the operation of the color sensor and check if it works.
  if (tcs.begin()) {
    Serial.println("Color sensor found");
  } else {
    Serial.println("Color sensor not found.  Please check your connections");
    while (1); // infinite loop to halt the program
  }
  
  // Initialize outputs:
  // Set the dropRequestSignal to low:
  digitalWrite(PIN_NR_DROP_REQ, LOW);
  // Set initial output to the motor to 0
  analogWrite(PIN_NR_PWM_OUTPUT, 0);
}
// End of function setup()


////////////////////////////////////////////////////////////////////////////////////////////////
// After going through the setup() function, which initializes and sets the initial values,  //
// the loop() function does precisely what its name suggests, and loops consecutively,        //
// allowing your program to sense and respond. Use it to actively control the Arduino board.  //
//////////////////////////////////////////////////////////////////////////////////////////////// 
void loop() {
  // Determine the duration it took to execute the last loop. This time is used 
  // for integration and for monitoring the loop time via the serial monitor.
  executionDuration = micros() - lastExecutionTime;
  lastExecutionTime = micros();

  // Speed Computation:
  if ((abs(motorPosition - previousMotorPosition) > MIN_VEL_COMP_COUNT) || (micros() - previousVelCompTime) > MIN_VEL_COMP_TIME){
    // If at least a minimum time interval has elapsed or
    // the motor has travelled through at least a minimum angle ... 
    // .. compute a new value for speed:
    // (speed = delta angle [encoder counts] divided by delta time [seconds])
    motorVelocity = (double)(motorPosition - previousMotorPosition) * 1000000 / 
                            (micros() - previousVelCompTime);
    // Remember this encoder count and time for the next iteration:
    previousMotorPosition = motorPosition;
    previousVelCompTime   = micros();
  }

    //****************************************************************************//
     // Check for the ball color and output to serial:
        int ballColor = evaluateColorSensor();
        Serial.print("Ball color is: ");
        switch (ballColor) {
          case MAIZE: Serial.println("MAIZE."); break;
          case BLUE:  Serial.println("BLUE.");  break;
          case RED:   Serial.println("RED.");   break;
          case WHITE: Serial.println("WHITE."); break;
          case NONE:  Serial.println("NONE."); break;
        }


  //******************************************************************************//
  // Position Controller
  if (digitalRead(PIN_NR_ON_OFF_SWITCH)==HIGH) {
    // If the toggle switch is on run the controller:
 
    //** Feedforward terms: **//
    // Compensate for friction.  That is, if we now the direction of 
    // desired motion, add a base command that helps with moving in this
    // direction.  Note, BASE_CMD can be positive or negative here depending on the desired direction.
      desiredVoltage =  BASE_CMD;

    // Gravity compensation lookup.  Here we record which voltage we need
    // to keep the device balanced at the left and at the right, and note 
    // where it is balanced passively.  The feedforward value is determined
    // by linear interpolation between these three points.
    if (motorPosition<FF_BALANCED_POSITION) {
        desiredVoltage = desiredVoltage + (FF_BALANCED_POSITION-motorPosition)/(FF_BALANCED_POSITION-LOWER_BOUND)*FF_VOLTAGE_LOWER_BOUND;
    }
    if (motorPosition>FF_BALANCED_POSITION) {
        desiredVoltage = desiredVoltage + (motorPosition-FF_BALANCED_POSITION)/(UPPER_BOUND-FF_BALANCED_POSITION)*FF_VOLTAGE_UPPER_BOUND;
    }
    
  } else { 
    // Otherwise, the toggle switch is off, so do not run the controller, 
    // stop the motor...
    desiredVoltage = 0; 
    // Produce some debugging output:
    Serial.println("The toggle switch is off.  Motor Stopped.");
  } 
  
  //** Send signal to motor **//
  // Convert from voltage to PWM cycle:
  motorCommand = int(abs(desiredVoltage * 255 / SUPPLY_VOLTAGE));
  // Clip values larger than 255
  if (motorCommand > 255) {
    motorCommand = 255;
  }
  // Send motor signals out
  analogWrite(PIN_NR_PWM_OUTPUT, motorCommand);
  // Determine rotation direction
  if (desiredVoltage >= 0) {
    // If voltage is positive ...
    // ... turn forward
    digitalWrite(PIN_NR_PWM_DIRECTION_1,LOW);  // rotate forward
    digitalWrite(PIN_NR_PWM_DIRECTION_2,HIGH); // rotate forward
  } else {
    // ... otherwise turn backward:
    digitalWrite(PIN_NR_PWM_DIRECTION_1,HIGH); // rotate backward
    digitalWrite(PIN_NR_PWM_DIRECTION_2,LOW);  // rotate backward
  }
  // End of Position Controller
  //*********************************************************************//


  //*********************************************************************//
  // Send a status of the controller to the serial monitor.  
  // Each character will take 85 microseconds to send, so be
  // selective in what you write out:
  
////  //Serial.print("      Raw signals from the color sensor: ");
////  Serial.print("  R: "); 
////  Serial.print(red);
////  Serial.print("  G: "); 
////  Serial.print(green);
////  Serial.print("  B: "); 
////  Serial.print(blue);
////  Serial.print("  C: "); 
////  Serial.print(clear);
//  Serial.println(); // new line
  Serial.print("      Position [encoder counts]: ");
  Serial.print("  P: "); 
  Serial.println(motorPosition);
  Serial.print("      Desired Voltage:  ");
  Serial.println(desiredVoltage);
//  // End of Serial Out
  //*********************************************************************//
}
// End of main loop


//////////////////////////////////////////////////////////////////////
// This is a function that returns the type of ball found in the    //
// cup.  It is called from the loop()-routine.  It returns one of   //
// the following values:                                            //
// 'MAIZE', 'BLUE', 'RED', 'WHITE', 'NONE'.                         //
// This function is not completed.  It currently always returns     //
// 'NONE'                                                           //
//////////////////////////////////////////////////////////////////////
int evaluateColorSensor(){
  // initialize ball type with 'NONE'.  Override later if a ball color was detected
  int ballType = NONE;
  
  // Read color values from sensor:
  tcs.setInterrupt(false);      // turn on LED
  delay(100); // Takes 0.1s to turn on the LED and stablize it
  tcs.getRawData(&red, &green, &blue, &clear);
  tcs.setInterrupt(true);

  // Check if the ball is MAIZE
  // NOTE THAT THIS MUST BE UPDATED WITH REALISTIC VALUES!
  if ((red>1000) && (red <1000) && (green>1000) && (green <1000)&& (blue>1000) && (blue <1000) && (clear>1000) && (clear <1000)){
    ballType = MAIZE;
  }
  // Check if the ball is BLUE
  // NOTE THAT THIS MUST BE UPDATED WITH REALISTIC VALUES!
  if ((red>1000) && (red <1000) && (green>1000) && (green <1000)&& (blue>1000) && (blue <1000) && (clear>1000) && (clear <1000)){
    ballType = BLUE;
  }
  // Check if the ball is RED
  // NOTE THAT THIS MUST BE UPDATED WITH REALISTIC VALUES!
  if ((red>1000) && (red <1000) && (green>1000) && (green <1000)&& (blue>1000) && (blue <1000) && (clear>1000) && (clear <1000)){
    ballType = RED;
  }
  // Check if the ball is WHITE
  // NOTE THAT THIS MUST BE UPDATED WITH REALISTIC VALUES!
  if ((red>1000) && (red <1000) && (green>1000) && (green <1000)&& (blue>1000) && (blue <1000) && (clear>1000) && (clear <1000)){
    ballType = WHITE;
  }
  // Note: If none of these if-statements is true, ballType remains 'NONE'
  return ballType;
} 
// End of function evaluateColorSensor()


//////////////////////////////////////////////////////////////////////
// This is a function to update the encoder count in the Arduino.   //
// It is called via an interrupt whenever the value on encoder      //
// channel A or B changes.                                          //
//////////////////////////////////////////////////////////////////////
void updateMotorPositionAndVelocity() {
  // Bitwise shift left by one bit, to make room for a bit of new data:
  encoderStatus <<= 1;   
  // Use a compound bitwise OR operator (|=) to read the A channel of the encoder (pin 2)
  // and put that value into the rightmost bit of encoderStatus:
  encoderStatus |= digitalRead(2);   
  // Bitwise shift left by one bit, to make room for a bit of new data:
  encoderStatus <<= 1;
  // Use a compound bitwise OR operator  (|=) to read the B channel of the encoder (pin 3)
  // and put that value into the rightmost bit of encoderStatus:
  encoderStatus |= digitalRead(3);
  // encoderStatus is truncated to only contain the rightmost 4 bits by  using a 
  // bitwise AND operator on mstatus and 15(=1111):
  encoderStatus &= 15;
  if (encoderStatus==2 || encoderStatus==4 || encoderStatus==11 || encoderStatus==13) {
    // the encoder status matches a bit pattern that requires counting up by one
    motorPosition++;         // increase the encoder count by one
  } 
  else if (encoderStatus == 1 || encoderStatus == 7 || encoderStatus == 8 || encoderStatus == 14) {
    // the encoder status does not match a bit pattern that requires counting up by one.  
    // Since this function is only called if something has changed, we have to count downwards
    motorPosition--;         // decrease the encoder count by one
  }
}
// End of function updateMotorPosition()



