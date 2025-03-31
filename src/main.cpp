#include <Arduino.h>
#include "Emm42_V5.0_Driver.hpp"

// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Hello, world!");
  delay(1000);
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}