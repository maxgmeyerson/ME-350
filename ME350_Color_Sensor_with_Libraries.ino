// ME350 Lab Color sensor 10/09/2016
//Tested. 

//////////////////////////////////////////////
// DEFINE CONSTANTS AND GLOBAL VARIABLES:   //
////////////////

//** Color Sensor: **//
// Include the necessary code headers:
#include "Adafruit_TCS34725.h"
#include <Wire.h>
// CONSTANTS: 
// Definition of ball types:
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



//////////////////////////////////////////////////////////////////////////////////////////
// The setup() function is called when a sketch starts. Use it to initialize variables, //
// pin modes, start using libraries, etc. The setup function will only run once, after  //
// each powerup or reset of the Arduino board:                                          //
  //////////////////////////////////////////////////////////////////////////////////////////
void setup() {

  // Begin serial communication for monitoring.
  Serial.begin(115200);
  Serial.println("Start Executing Program.");

  // Begin the operation of the color sensor and check if it works.
  if (tcs.begin()) {
    Serial.println("Color sensor found");
  } else {
    Serial.println("Color sensor not found.  Please check your connections");
   // while (1); // infinite loop to halt the program
  }
}


void loop() {  
   int ballColor = evaluateColorSensor();
   testColorSensor();
   Serial.print("Ball color is: ");
   switch (ballColor) {
     case MAIZE: Serial.println("MAIZE."); break;
     case BLUE:  Serial.println("BLUE.");  break;
     case RED:   Serial.println("RED.");   break;
     case WHITE: Serial.println("WHITE."); break;
     case NONE:  Serial.println("NONE."); break;
   }
}
// End of main loop



//////////////////////////////////////////////////////////////////////
// This is a function that returns the raw red, green, blue, clear  //
// Value. It is called from the loop()-routine.  It prints these    //
// Values to the Screen. Use this function to test color sensor     //        
//////////////////////////////////////////////////////////////////////
void testColorSensor() {
  tcs.setInterrupt(false);      // turn on LED
  delay(100); // Takes 0.1s to turn on the LED and stablize it
  tcs.getRawData(&red, &green, &blue, &clear);
  tcs.setInterrupt(true);   // turn off LED 
  Serial.print(F("Raw R:")); 
  Serial.print(red); 
  Serial.print(F(" G:")); 
  Serial.print(green); 
  Serial.print(F(" B:")); 
  Serial.print(blue); 
  Serial.print(F(" C:")); 
  Serial.println(clear); 
  delay(1000);
}

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
  if ((red>2000) && (red <2000) && (green>2000) && (green <2000)&& (blue>2000) && (blue <2000) && (clear>2000)){
    ballType = MAIZE;
  }
  // Check if the ball is BLUE
  // NOTE THAT THIS MUST BE UPDATED WITH REALISTIC VALUES!
  if ((red>2000) && (red <2000) && (green>2000) && (green < 2000)&& (blue>2000) && (blue <2000) && (clear>2000)){
    ballType = BLUE;
  }
  // Check if the ball is RED
  // NOTE THAT THIS MUST BE UPDATED WITH REALISTIC VALUES!
  if ((red>2000) && (red <2000) && (green>2000) && (green <2000)&& (blue>2000) && (blue <2000) && (clear>2000)){
    ballType = RED;
  }
  // Check if the ball is WHITE
  // NOTE THAT THIS MUST BE UPDATED WITH REALISTIC VALUES!
  if ((red>2000) && (red <2000) && (green>2000) && (green <2000)&& (blue>2000) && (blue <2000) && (clear>2000) ){
    ballType = WHITE;
  }
  // Note: If none of these if-statements is true, ballType remains 'NONE'
  return ballType;
} 
// End of function evaluateColorSensor()




