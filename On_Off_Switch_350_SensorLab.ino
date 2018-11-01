int onoffSwitch = 0;

const int onoffSwitchPin = 5;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Start");
  pinMode(onoffSwitchPin, INPUT);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  onoffSwitch = digitalRead(onoffSwitchPin);

  if (onoffSwitch == HIGH) {

    Serial.println("Switch on");
  } 
  else{
    Serial.println("Switch off");
  }
  
  
}
