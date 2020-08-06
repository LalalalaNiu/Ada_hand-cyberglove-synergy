
byte states = 0; //set the indicator for the motor status.

int pin_switch =6;
int dirPin1 = 3;
int stepperPin1 = 2;

int pd = 500; // pulse delay period
boolean setdir = LOW; //set Direction

boolean switchstate1 = LOW;
boolean switchstate2 = LOW;




void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
Serial.println(" Stepper Condition: ");

pinMode(dirPin1, OUTPUT);
pinMode(stepperPin1, OUTPUT);

pinMode(pin_switch, INPUT);

}
void step1(boolean dir,int steps){
  digitalWrite(dirPin1,dir);


  for(int i=0;i<steps;i++){
    digitalWrite(stepperPin1, HIGH);
    delayMicroseconds(1400);
    digitalWrite(stepperPin1, LOW);
    delayMicroseconds(1400);
  }
}

void step2(boolean dir,int steps){
  digitalWrite(dirPin1,dir);

  for(int i=0;i<steps;i++){
    digitalWrite(stepperPin1, HIGH);
    delayMicroseconds(1300);
    digitalWrite(stepperPin1, LOW);
    delayMicroseconds(1300);
  }
}

void loop() {
  // put your main code here, to run repeatedly:

  switchstate1 = digitalRead(pin_switch);
  
  if (switchstate1 != switchstate2)
  { 
  
    states++;
   
    Serial.println(states);
    delay(500);
  }
  else{
    if(states == 1)
    {step1(true,10);}

    if(states == 3)
    {step2(false,10);}
    if(states == 4)
    {states = 0;}
    
  }
}
