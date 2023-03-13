#define pwm1         10
#define button1      8
#define pwm2         11
#define button2      9

boolean switch1 = 0;
int buttonState1 = 0;
int level1 = 100;
int level2 = 100;

boolean switch2 = 0;
int buttonState2 = 0;

void setup() {
  pinMode(pwm1,   OUTPUT);
  pinMode(button1, INPUT_PULLUP);
  pinMode(pwm2,   OUTPUT);
  pinMode(button2, INPUT_PULLUP);
  Serial.begin(9600);
}

void loop() {

   buttonState1 = digitalRead(button1);
   buttonState2 = digitalRead(button2);
   
  if (buttonState1 == 0){ 
    switch1 = !switch1;
  }

  if (buttonState2 == 0){ 
    switch2 = !switch2;
  }
      
  delay(250);


  if (switch1 == 1)  {
    analogWrite(pwm1, level1);

  }
  else if (switch1 == 0){
   analogWrite(pwm1, 0); 
  }
  
  if (switch2 == 1)  {
    analogWrite(pwm2, level2);

  }
  else if (switch2 == 0){
   analogWrite(pwm2, 0);   
  
    

}
}
