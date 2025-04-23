#include <Arduino.h>
#include "Emm42_V5.0_Driver.hpp"
#include <HardwareSerial.h>



void setup()
{
    Serial.begin(115200);
}

float t = 0;
void loop() 
{
    t += 0.1;
    Serial.printf("d: %f, %f\n", sin(t), sin(2*t));
    delay(100);
}

