// Include necessary libraries:
#include <Thread.h>


///// PORTS /////

// If DEBUG is defined, more print statements will go to the Serial
#define DEBUG

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

-----///// Functions /////------

// Debug print function
void debug_print(String output) {
  #ifdef DEBUG
    Serial.print(output);
  #endif
}

// debug println function
void debug_println(String output) {
  #ifdef DEBUG
    Serial.println(output);
  #endif
}

// E-STOP Function:
void checkEstop() {
    debug_println("Check ESTOP");
    if(!digitalRead(ESTOP)) {
      runMain = false;
      debug_println("FALSE");
      killAll();
    }
}

// shutdown function
void killAll() {
  digitalWrite(GREEN, LOW);
  digitalWrite(RED, LOW);
  digitalWrite(YELLOW, LOW);
}

-----/////////////////////------

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

  while(runMain) {
    
    // put your main code here, to run repeatedly:
    analogWrite(GREEN, 50);
  
    // Checks if ESTOP thread could run:
    if(eSTOP.shouldRun()) { // (NOTE: Consider adding a thread controller at a later date)
      eSTOP.run();
    }
  }

  // Insert restart code
  
}
