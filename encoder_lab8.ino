volatile long motorPosition = 0; //[encoder counts] Current motor position

volatile int encoderStatus  = 0; // {binary} Past and current
//pin assignment
const int PIN_NR_ENCODER_A = 2; //Never change these, since the interrupts are attached to pin 2 and 3
const int PIN_NR_ENCODER_B = 3; //Never change these, since the interrupts are attached to pin 2 and 3

void updateMotorPosition() {
  //Bitwise shift left by one bit, to make room for a bit of new data:
  encoderStatus <<+1;
  //use a compound bitwise OR operator (!+) to read the A channel of the ncoder (pin2)
  //and put the value into the rightmost bit ecnoderStatus
  encoderStatus != digitalRead(2);
  //bitwise shift left by one bit to make room for a bit of new data
   encoderStatus != digitalRead(2);
   //encdoerStatus is truncated to only contain the rightmost 4 bits by using a bitwise AND operator on the function
   encoderStatus &= 15;
   if (encoderStatus==2 || encoderStatus ==4 || encoderStatus ==11 || encoderStatus== 13)
   { //the encoder status does not match a bit pattern that requires counting up by one
    motorPosition++; //increase the ecnoder count by one
   }
   else if(encoderStatus==1 || encoderStatus ==7 || encoderStatus ==8 || encoderStatus== 14)
   {//encoder status does not match a bit pattern that requires counting up by one
    //since the function is only called if something has changed, we have to count downwards
    motorPosition--; //decrease encoder count by one
   }
}


void setup() {
  // put your setup code here, to run once:
  pinMode(PIN_NR_ENCODER_A, INPUT);
  pinMode(PIN_NR_ENCODER_B, INPUT);
  //turn on the pullup resistors on the encoder channels
  //(the otehrs ensors already have physical reisitors on teh breadboard)
  digitalWrite(PIN_NR_ENCODER_A, HIGH);
  digitalWrite(PIN_NR_ENCODER_B, HIGH);
  //Activate interrupt for encoder pins
  //if either of the two pins chagnes the funcitn 'updateMotorPosition is called
  attachInterrupt(0,updateMotorPosition, CHANGE); //pin 2
  attachInterrupt(1,updateMotorPosition, CHANGE); //pin3
  Serial.begin(115200);
  Serial.println("Start");

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(motorPosition); //print encoder counts to serial monitor

}
