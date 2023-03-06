// Include necessary libraries:
#include <Thread.h>


///// PORTS /////

// Debug LED Ports:
#define GREEN 9
#define RED 10
#define YELLOW 11

// Define E-stop Port
#define ESTOP 12

/////////////////


///// Threads /////
Thread eSTOP = Thread();
///////////////////


///// Global Variables /////
bool runMain = true;
////////////////////////////


// E-STOP Function:
void checkEstop() {
    Serial.println("Check ESTOP");
    if(!digitalRead(ESTOP)) {
      runMain = false;
      Serial.println("FALSE");
    }
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

  // Initialize pin
  pinMode(ESTOP, INPUT_PULLUP);

  // Initialize Threads
  eSTOP.onRun(checkEstop);
  eSTOP.setInterval(500);

}


void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(GREEN, 50);

  // Checks if ESTOP thread could run:
  if(eSTOP.shouldRun()) {
    eSTOP.run();
  }

  // Gets Estop value:
  //Serial.println(runMain);
  
  }
