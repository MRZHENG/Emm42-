#include <Arduino.h>
#include "Emm42_V5.0_Driver.hpp"
#include <HardwareSerial.h>

// put function declarations here:
Emm42_V5_0_Driver Emm42(&Serial2,1);

void setup() {
  Serial.begin(115200);
  Emm42.Emm_V5_Motor_enable();
  Emm42.Emm_V5_Vel_Control(0,3000);
}

void loop() {
  delay(1000);
  //Emm42.Emm_V5_Pos_Control(0,3000,320000);
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}