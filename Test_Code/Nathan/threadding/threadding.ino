
///// PORTS /////

// Debug LED Ports:
#define GREEN 9
#define RED 10
#define YELLOW 11

/////////////////

// E-STOP Function:
void checkEstop() {
  
}


// SETUP:
void setup() {
  Serial.begin(9600); // Sets up serial port
  
  // Initialize tester pins:
  pinMode(GREEN, OUTPUT);
  pinMode(RED, OUTPUT);
  pinMode(YELLOW, OUTPUT);
  // Initialize output pins:
  digitalWrite(GREEN, HIGH);
  digitalWrite(RED, LOW);
  digitalWrite(YELLOW, LOW);
  

}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(GREEN, 50);

}
