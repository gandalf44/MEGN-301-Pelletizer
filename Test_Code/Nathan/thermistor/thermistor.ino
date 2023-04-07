// this example is public domain. enjoy! https://learn.adafruit.com/thermocouple/

#include "max6675.h"

int thermoDO = 27;
int thermoCS = 29;
int thermoCLK = 31;

// Thermcouple Dos
#define THERMO_DOS_SO 35
#define THEMRO_DOS_CS 37
#define THERMO_DOS_SCK 39
MAX6675 thermoDos(THERMO_DOS_SCK, THEMRO_DOS_CS, THERMO_DOS_SO);

MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

void setup() {
  Serial.begin(9600);

  Serial.println("MAX6675 test");
  // wait for MAX chip to stabilize
  delay(500);
}

void loop() {
  // basic readout test, just print the current temp
  
  Serial.print("C = "); 
  Serial.print(thermocouple.readCelsius());
  Serial.print(" C = ");
  Serial.println(thermoDos.readCelsius());
 
  // For the MAX6675 to update, you must delay AT LEAST 250ms between reads!
  delay(300);
}
