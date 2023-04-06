// this example is public domain. enjoy! https://learn.adafruit.com/thermocouple/

#include "max6675.h"

int thermoDO = 27;
int thermoCS = 29;
int thermoCLK = 31;

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
  Serial.println(thermocouple.readCelsius());
 
  // For the MAX6675 to update, you must delay AT LEAST 250ms between reads!
  delay(250);
}
