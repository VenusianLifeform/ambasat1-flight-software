#include <Arduino.h>
#include "Logging.h"
#include "KorellianP1SatApp.h"

KorellianP1SatApp* satelliteApp;

void setup()
{
  Serial.begin(9600);

  satelliteApp = new KorellianP1SatApp();
  satelliteApp->setup();
}

void loop()
{
  satelliteApp->loop();

}