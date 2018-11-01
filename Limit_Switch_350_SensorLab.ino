int limSwitch1 = 0;

const int limSwitch1Pin = 5;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Start");
  pinMode(limSwitch1Pin, INPUT);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  limSwitch1 = digitalRead(limSwitch1Pin);

  Serial.print("\t");
  Serial.print("limSwitch 1=");
  Serial.print("\t");
  Serial.println(limSwitch1);
  
  
}
