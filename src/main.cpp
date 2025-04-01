#include <Arduino.h>
#include "Emm42_V5.0_Driver.hpp"

// put function declarations here:
Emm42_V5_0_Driver Emm42(&Serial2,1);

void setup() {
  Serial.begin(115200);
  Emm42.sync_Motor_enable();
  
}

void loop() {
  delay(10000);
  Emm42.Emm_V5_Pos_Control(0,3000,320000);
  Serial.println("Motor 1 is moving to 3000");
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}