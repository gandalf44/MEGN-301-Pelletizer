// Include necessary libraries:
#include <Thread.h>
#include "max6675.h"


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

// Define E-stop Port
#define ESTOP 12

/////////////////


///// Threads /////
Thread userInput = Thread();
///////////////////

///// Global Variables /////
bool runMain = true; // true so long as the primary code should run in main (E-stop makes this false)
bool isHot = false; // true if heater is hot
////////////////////////////

// class to create a custom PID
class PID {
  private:
    double pkP;
    double pkI;
    double pkD;
    double pPrevInput;
    double pPrevTime;
    double pImax;
    double pItot;
    double pSetpoint;
  public:
    // default counstructor
    PID(){
      pkP = 0;
      pkI = 0;
      pkD = 0;
      pPrevInput = 0;
      pPrevTime = 0;
      pImax = 0;
      pItot = 0;
      pSetpoint = 0;
    }
    
    // Overloaded constructor
    PID(double P, double I, double D, double Dmax) {
      pkP = P;
      pkI = I;
      pkD = D;
      pPrevInput = 0;
      pPrevTime = 0;
      pImax = Dmax;
      pItot = 0;
      pSetpoint = 0;
    }
  
    // setters:
    void setP(double P){ pkP = P; }
    void setI(double I){ pkP = I; }
    void setD(double D){ pkD = D; }
    void setDmax(double Dmax) { pImax = Dmax; }
    void setSetpoint(double setpoint) { pSetpoint = setpoint; }
    
    // getters:
    double getP(){ return pkP; }
    double getI(){ return pkI; }
    double getD(){ return pkD; }
    double getDmax(){ return pImax; }
    void getSetpoint() { return pSetpoint; }

    double startOutput(double prevInput, double prevTime) {
      pPrevInput = prevInput;
      pPrevTime = prevTime;
    }

    double output(double curInput, double curTime){ 
      // Calculate the integral
      pItot = pItot + (curTime-pPrevTime)*curInput;

      // Check if I total is greater than the max:
      if(pItot > pImax) {
        pItot = pImax;
      } else if (pItot < -pImax) {
        pItot = -pImax;
      }
      
      // Evaluate P, i, and D terms:
      double P = pkP*(pSetpoint-curInput);
      double I = pkI*pItot;
      double D = pkD*(pPrevInput-curInput)/(pPrevTime-curTime);

      // reinitializes previous variables
      pPrevInput = curInput;
      pPrevTime = curTime;
      
      return (P + I + D);
    }
};

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
void checkInput() {
    debug_println("Checking for input");

    // Check ESTOP
    if(!digitalRead(ESTOP)) {
      runMain = false;
      debug_println("ESTOP TRIGGERED!");
      killAll();
    }

    Serial.println(thermoUno.readCelsius());
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
  userInput.onRun(checkInput);
  userInput.setInterval(300);

}


void loop() {

  while(runMain) {
    
    // put your main code here, to run repeatedly:
    analogWrite(GREEN, 50);

    if(userInput.shouldRun()) { // (NOTE: Consider adding a thread controller at a later date)
      userInput.run();
    }
    
  }

  // Insert restart code
  
}
