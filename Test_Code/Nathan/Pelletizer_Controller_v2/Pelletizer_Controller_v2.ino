// Include necessary libraries:
#include "max6675.h"
#include "PID_v1.h"
#include <Thread.h>


///// PORTS /////

// If DEBUG is defined, more print statements will go to the Serial
#define DEBUG

// Debug LED Ports:
#define GREEN 9
#define RED 10
#define YELLOW 11
#define HEATER 52
#define HEATON 22

// Thermcouple Uno
#define THERMO_UNO_DO 4
#define THEMRO_UNO_CS 5
#define THERMO_UNO_CLK 6
MAX6675 thermoUno(THERMO_UNO_CLK, THEMRO_UNO_CS, THERMO_UNO_DO);

// Thermcouple Dos

// Motor Ports:
// #define SHAFT 20
// #define CUTTER 21

// Define E-stop Port
#define ESTOP 12

/////////////////

///// Threads /////
Thread CHECK_BUTTONS = Thread();
///////////////////

///// Global Variables /////
bool runMain = true; // true so long as the primary code should run in main (E-stop makes this false)
bool isHot = false; // true if heater is hot
////////////////////////////

// PID
double Setpoint, unoTemp, Output;
double Kp=2, Ki=.5, Kd=0;
PID unoPID(&unoTemp, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
int WindowSize = 1000;
unsigned long windowStartTime;

//-----///// Functions /////------//

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
void checkButtons() {
    debug_println("Check ESTOP");
    if(!digitalRead(ESTOP)) {
      runMain = false;
      debug_println("FALSE");
      killAll();
    }

    // Calculates PWM:
    unoTemp = thermoUno.readCelsius();
    unoPID.Compute();
      
    // Prints PID details:
    Serial.print("C = ");
    Serial.print(unoTemp);
    Serial.print(" | P(");
    Serial.print(unoPID.GetKp());
    Serial.print(")*I(");
    Serial.print(unoPID.GetKi());
    Serial.print(")*D(");
    Serial.print(unoPID.GetKd());
    Serial.print(") = ");
    Serial.println(Output);
}


// shutdown function
void killAll() {
  digitalWrite(GREEN, LOW);
  digitalWrite(RED, LOW);
  digitalWrite(YELLOW, LOW);
  digitalWrite(HEATER, LOW);
}

//-----/////////////////////------//

// SETUP:
void setup() {
  Serial.begin(9600); // Sets up serial port

  Serial.println("PELLETIZER INITIALIZED");
  
  // Initialize tester pins:
  pinMode(GREEN, OUTPUT);
  pinMode(RED, OUTPUT);
  pinMode(YELLOW, OUTPUT);
  
  // Initialize output pins:
  digitalWrite(GREEN, HIGH);
  digitalWrite(RED, LOW);
  digitalWrite(YELLOW, LOW);
  digitalWrite(HEATER, LOW);

  // Initialize pin
  pinMode(ESTOP, INPUT_PULLUP);

  // Initialize Threads
  CHECK_BUTTONS.onRun(checkButtons);
  CHECK_BUTTONS.setInterval(300);


  ////// PID: //////
  windowStartTime = millis();

  //initialize the variables we're linked to
  Setpoint = 30 ;

  //tell the PID to range between 0 and the full window size
  unoPID.SetOutputLimits(0, WindowSize);

  //turn the PID on
  unoPID.SetMode(AUTOMATIC);
   
  ///////////////
}


void loop() {
    
  while(runMain) {
    
    // put your main code here, to run repeatedly:
    analogWrite(GREEN, 50);

    // PID PWM:
    if (millis() - windowStartTime > WindowSize)
    { //time to shift the Relay Window
      windowStartTime += WindowSize;
      debug_print("Increase Window Size: ");
      Serial.println(windowStartTime);
    }
    if (Output < millis() - windowStartTime) digitalWrite(RED, HIGH);
    else digitalWrite(RED, LOW);


    // Checks if ESTOP thread could run:
    if(CHECK_BUTTONS.shouldRun()) { // (NOTE: Consider adding a thread controller at a later date)
      CHECK_BUTTONS.run();
    }
  }

  // Insert restart code
  
}