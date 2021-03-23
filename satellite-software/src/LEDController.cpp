#include "Arduino.h"
#include "LEDController.h"

LEDController::LEDController(const uint8_t pinNumber) 
{
    this->pinNumberForLED = pinNumber;
}

void LEDController::switchOn() 
{
    pinMode(this->pinNumberForLED, OUTPUT);
    digitalWrite(this->pinNumberForLED, LOW);
    digitalWrite(this->pinNumberForLED, HIGH);
}

void LEDController::switchOff()
{
    digitalWrite(this->pinNumberForLED, LOW);
}

