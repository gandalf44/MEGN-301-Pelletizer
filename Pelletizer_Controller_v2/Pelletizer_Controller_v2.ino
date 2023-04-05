// Include necessary libraries:
#include <max6675.h>
//#include <PID_v1.h>
#include <Thread.h>

#include  <Wire.h>
#include  <LiquidCrystal_I2C.h>


///// PORTS /////

// If DEBUG is defined, more print statements will go to the Serial
#define DEBUG

// Debug LED Ports:
#define GREEN 9
#define RED 10
#define YELLOW 11
#define WHITE   12
#define HEATER_UNO 23
#define HEATER_DOS 22
#define HEATON 22 // delete
#define UNO_INDICATOR 25

#define SHAFTON 2
#define CUTON 3

// Thermcouple Uno
#define THERMO_UNO_SO 27
#define THEMRO_UNO_CS 29
#define THERMO_UNO_SCK 31
MAX6675 thermoUno(THERMO_UNO_SCK, THEMRO_UNO_CS, THERMO_UNO_SO);

// Thermcouple Dos
#define THERMO_DOS_SO 35
#define THEMRO_DOS_CS 37
#define THERMO_DOS_SCK 39
MAX6675 thermoDos(THERMO_DOS_SCK, THEMRO_DOS_CS, THERMO_DOS_SO);

// Motor Ports:
#define SHAFT 7
#define CUTTER 8

// Define E-stop Port
#define ESTOP 13

// LCD
LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

/////////////////

///// Threads /////
Thread CHECK_BUTTONS = Thread();
Thread PID_COMPUTE = Thread();
///////////////////

///// Global Variables /////
bool runMain = true; // true so long as the primary code should run in main (E-stop makes this false)
bool isHot = false; // true if HEATER_UNO is hot
short shaftPWM = 50;
short cutPWM = 50;
bool shaftOn = false; // shaft defaults to turned off
bool cutOn = false; // shaft defaults to turned off
////////////////////////////

// class to create a custom PID
class PID {
  private:
    double pkP;
    double pkI;
    double pkD;
    double pPrevInput;
    long pPrevTime;
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
    PID(double P, double I, double D, double Imax) {
      pkP = P;
      pkI = I;
      pkD = D;
      pPrevInput = 0;
      pPrevTime = 0;
      pImax = Imax;
      pItot = 0;
      pSetpoint = 0;
    }
  
    // setters:
    void setP(double P){ pkP = P; }
    void setI(double I){ pkP = I; }
    void setD(double D){ pkD = D; }
    void setImax(double Imax) { pImax = Imax; }
    void setSetpoint(double setpoint) { pSetpoint = setpoint; }
    
    // getters:
    double getP(){ return pkP; }
    double getI(){ return pkI; }
    double getD(){ return pkD; }
    double getDmax(){ return pImax; }
    void getSetpoint() { return pSetpoint; }

    double startOutput(double prevInput) {
      pPrevInput = prevInput;
      pPrevTime = millis();
    }

    double output(double curInput){ 
      long curTime = millis();
      /*Serial.print("T(");
      Serial.print(curTime);
      Serial.print(" ms) ");*/
      // Calculate the integral
      pItot += (curTime-pPrevTime)*curInput;

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
      /*Serial.print("P(");
      Serial.print(P);
      Serial.print(")*I(");
      Serial.print(I);
      Serial.print(")*D(");
      Serial.print(D);
      Serial.print(") = ");*/
      return (P + I + D);
    }
};

// PID UNO
double unoSetpoint, unoTemp, unoOutput;
double Kp=2, Ki=0.15, Kd=0;
PID unoPID(Kp, Ki, Kd, 500);
int WindowSize = 1000; // CONSIDER REDUCING WINDOW SIZE TO 300 ms (the thermocouple's refresh rate)
unsigned long windowStartTime;

// PID DOS
#define DOSPID // Defines compiler variable to run DOS PID and keep UNO PID from turning on DOS heater
double dosSetpoint, dosTemp, dosOutput;
double DOS_Kp=10, DOS_Ki=0.2, DOS_Kd=0;
PID dosPID(DOS_Kp, DOS_Ki, DOS_Kd, 500);

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
    //debug_println("Check ESTOP");
    if(!digitalRead(ESTOP)) {
      runMain = false;
      debug_println("FALSE");
      killAll();
    }

    // Check if Shaft button is pressted
    if(!digitalRead(SHAFTON)) {
      shaftOn = !shaftOn;
    }
    // Check if cut button is pressed
    if(!digitalRead(CUTON)) {
      cutOn = !cutOn;
    }

    // Check's if PWM's are on
    if(shaftOn){
      analogWrite(WHITE, shaftPWM);
      analogWrite(SHAFT, shaftPWM);
    } else {
      analogWrite(WHITE, 0);
      analogWrite(SHAFT, 0);
    }
    if(cutOn) {
      analogWrite(YELLOW, cutPWM);
      analogWrite(CUTTER, cutPWM);
    } else {
      analogWrite(YELLOW, 0);
      analogWrite(CUTTER, 0);
    }

}

void pidCompute() {
    // Calculates PWM:
    unoTemp = thermoUno.readCelsius();
    unoOutput = unoPID.output(unoTemp);
    dosTemp = thermoDos.readCelsius();
    dosOutput = dosPID.output(dosTemp);

    // Prints temp readings to LCD screen:
    lcd.clear();
    lcd.print("UNO = ");
    lcd.print(unoTemp);
    lcd.print(" C");
    lcd.setCursor(0,1);
    lcd.print("DOS = ");
    lcd.print(dosTemp);
    lcd.print(" C");
    
      
    // Prints PID details:
    Serial.print("UNO C = ");
    Serial.print(unoTemp);
    Serial.print(" | P(");
    Serial.print(unoPID.getP());
    Serial.print(")*I(");
    Serial.print(unoPID.getI());
    Serial.print(")*D(");
    Serial.print(unoPID.getD());
    Serial.print(") = ");
    Serial.println(unoOutput);
    Serial.print("DOS C = ");
    Serial.print(dosTemp);
    Serial.print(" | P(");
    Serial.print(dosPID.getP());
    Serial.print(")*I(");
    Serial.print(dosPID.getI());
    Serial.print(")*D(");
    Serial.print(dosPID.getD());
    Serial.print(") = ");
    Serial.println(unoOutput);
    /*Serial.print(" DIRECT = ");
    Serial.print(DIRECT);
    Serial.print(" DIRECTION = ");
    Serial.println(unoPID.GetDirection());*/
    
}


// shutdown function
void killAll() {
  digitalWrite(GREEN, LOW);
  digitalWrite(RED, LOW);
  digitalWrite(YELLOW, LOW);
  digitalWrite(WHITE, LOW);
  digitalWrite(HEATER_UNO, LOW);
  digitalWrite(UNO_INDICATOR, LOW);
  digitalWrite(HEATER_DOS, LOW);
  digitalWrite(HEATER_DOS, LOW);
  digitalWrite(SHAFT, LOW);
  digitalWrite(CUTTER, LOW);
}

//-----/////////////////////------//

// SETUP:
void setup() {
  Serial.begin(9600); // Sets up serial port

  // Prints startup message:
  Serial.println("PELLETIZER INITIALIZED");
  
  // Initialize tester pins:
  pinMode(GREEN, OUTPUT);
  pinMode(RED, OUTPUT);
  pinMode(YELLOW, OUTPUT);
  pinMode(HEATER_UNO, OUTPUT);
  digitalWrite(HEATER_DOS, LOW);
  pinMode(UNO_INDICATOR, OUTPUT);
  pinMode(HEATER_DOS, OUTPUT);
  
  // Initialize output pins:
  digitalWrite(GREEN, HIGH);
  digitalWrite(RED, LOW);
  digitalWrite(YELLOW, LOW);
  digitalWrite(HEATER_UNO, LOW);
  digitalWrite(HEATER_DOS, LOW);
  digitalWrite(UNO_INDICATOR, LOW);
  digitalWrite(HEATER_DOS, LOW);
  
  digitalWrite(HEATER_DOS, LOW);

  // Initialize pin
  pinMode(ESTOP, INPUT_PULLUP);
  pinMode(SHAFTON, INPUT_PULLUP);
  pinMode(CUTON, INPUT_PULLUP);

  // Initialize Threads
  CHECK_BUTTONS.onRun(checkButtons);
  CHECK_BUTTONS.setInterval(300);
  PID_COMPUTE.onRun(pidCompute);
  PID_COMPUTE.setInterval(WindowSize);

  ////// PID: //////
  windowStartTime = millis();

  //initialize the variables we're linked to
  unoSetpoint = 0;
  dosSetpoint = 0;

  //tell the PID to range between 0 and the full window size
  //unoPID.SetOutputLimits(0, WindowSize);
  //dosPID.SetOutputLimits(0, WindowSize);

  //turn the PID on
  //unoPID.SetMode(AUTOMATIC);
  //dosPID.SetMode(AUTOMATIC);
  ////////////////

  // LCD
  lcd.init(); // initialize the lcd 
  lcd.backlight();
  lcd.clear();
}


void loop() {
  String serialValue;
  while(runMain) {
    digitalWrite(GREEN, 60);
    // put your main code here, to run repeatedly:
    
    // Checks for new string in serial:
    if(Serial.available() > 0){
      serialValue = Serial.readString();
      Serial.println("USER INPUT");
      Serial.println(serialValue.charAt(0));
      
      // if P, set P.
      if(serialValue.charAt(0) == 'P') {
        Serial.print("UNO P set to . . . ");
        Kp = (serialValue.substring(1)).toDouble();
        Serial.println(Kp);
        unoPID.setP(Kp);
      }
      // if I, set I.
      else if(serialValue.charAt(0) == 'I') {
        Serial.print("UNO I set to . . . ");
        Ki = (serialValue.substring(1)).toDouble();
        Serial.println(Ki);
        unoPID.setI(Ki);
      }
      // if D, set D.
      else if(serialValue.charAt(0) == 'D') {
        Serial.print("UNO D set to . . . ");
        Kd = (serialValue.substring(1)).toDouble();
        Serial.println(Kd);
        unoPID.setD(Kd);
      }
      // if C, set cutter speed
      else if(serialValue.charAt(0) == 'C') {
        Serial.print("cutPWM set to . . . ");
        cutPWM = (serialValue.substring(1)).toInt();
        Serial.println(cutPWM);
      }
      // if S, set shaft speed
      else if(serialValue.charAt(0) == 'S') {
        Serial.print("shaftPWM set to . . . ");
        shaftPWM = (serialValue.substring(1)).toInt();
        Serial.println(shaftPWM);
      }
      // if X, change uno setpoint
      else if(serialValue.charAt(0) == 'X') {
        Serial.print("UNO setpoint changed to . . . ");
        unoSetpoint = (serialValue.substring(1)).toInt();
        unoPID.setSetpoint(unoSetpoint);
        Serial.println(unoSetpoint);
      }
      // if Y, change dos Setpoint
      else if(serialValue.charAt(0) == 'Y') {
        Serial.print("DOS setpoint changed to . . . ");
        dosSetpoint = (serialValue.substring(1)).toInt();
        dosPID.setSetpoint(dosSetpoint);
        Serial.println(dosSetpoint);
      }
      // if Q change DOS_kP
      else if(serialValue.charAt(0) == 'Q') {
        Serial.print("DOS P changed to . . . ");
        DOS_Kp = (serialValue.substring(1)).toDouble();
        Serial.println(DOS_Kp);
        dosPID.setP(DOS_Kp);
      }
      // if Q change DOS_kI
      else if(serialValue.charAt(0) == 'W') {
        Serial.print("DOS I changed to . . . ");
        DOS_Ki = (serialValue.substring(1)).toDouble();
        Serial.println(DOS_Ki);
        dosPID.setI(DOS_Ki);
      }
      // if Q change DOS_kD
      else if(serialValue.charAt(0) == 'E') {
        Serial.print("DOS D changed to . . . ");
        DOS_Kd = (serialValue.substring(1)).toDouble();
        Serial.println(DOS_Kd);
        dosPID.setD(DOS_Kd);
      }
    }
    
    

    // PID PWM:
    if (millis() - windowStartTime > WindowSize)
    { //time to shift the Relay Window
      windowStartTime += WindowSize;
      debug_print("Increase Window Size: ");
      Serial.println(windowStartTime);
    }
    if (unoOutput > millis() - windowStartTime) { // < changed from default
      digitalWrite(HEATER_UNO, HIGH);
      #ifndef DOSPID
        digitalWrite(HEATER_DOS, HIGH);
      #endif
      digitalWrite(UNO_INDICATOR, HIGH);
    }
    else {
      digitalWrite(HEATER_UNO, LOW);
      #ifndef DOSPID
        digitalWrite(HEATER_DOS, LOW);
      #endif
      digitalWrite(UNO_INDICATOR, LOW);
    }

    #ifdef DOSPID
      if (dosOutput > millis() - windowStartTime) { // < changed from default
        digitalWrite(HEATER_DOS, HIGH);
      }
      else {
        digitalWrite(HEATER_DOS, LOW);
      }
    #endif


    // Checks if button thread should run:
    if(CHECK_BUTTONS.shouldRun()) { // (NOTE: Consider adding a thread controller at a later date)
      CHECK_BUTTONS.run();
    }
    
    // Checks if PID thread should run
    if(PID_COMPUTE.shouldRun()) { // (NOTE: Consider adding a thread controller at a later date)
      PID_COMPUTE.run();
    }
  }

  // Insert restart code
  digitalWrite(RED, HIGH);

  // Prints temp readings to LCD screen:
  lcd.clear();
  lcd.print("UNO = ");
  lcd.print(thermoUno.readCelsius());
  lcd.print(" C");
  lcd.setCursor(0,1);
  lcd.print("DOS = ");
  lcd.print(thermoDos.readCelsius());
  lcd.print(" C");

  delay(300);
  
  
}
