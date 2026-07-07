#include <Arduino.h>

#include "testpipeline.h"

void setup()
{
    // put your setup code here, to run once:
    initializePipeline();
    // Serial.begin(115200);
}

void loop()
{
    // put your main code here, to run repeatedly:
    // drawCube();
    Serial.println("Stayin' alive");
    delay(1000);
}
