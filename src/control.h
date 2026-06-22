#ifndef CONTROL_H
#define CONTROL_H

#include <AccelStepper.h>
#include <WebServer.h>
#include <HX711.h>
#include "enums.h"

void stepperSetSpeed(AccelStepper& stepper, MotorMode& mode, WebServer& server);
void stepperMoveTo(AccelStepper& stepper, MotorMode& mode, WebServer& server);
void stepperGetPositionAndSpeed(AccelStepper& stepper, WebServer& server);
void forkBeltSetSpeed(WebServer& server, int in1Pin, int in2Pin, int ledc);
void homeLeadScrew(bool& isHomingMode, AccelStepper& lead_screw);

#endif
