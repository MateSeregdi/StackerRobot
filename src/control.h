#ifndef CONTROL_H
#define CONTROL_H

#include <AccelStepper.h>
#include <WebServer.h>
#include <HX711.h>
#include "enums.h"

void stepperSetSpeed(AccelStepper& stepper, MotorMode& mode, WebServer& server);
void stepperMoveTo(AccelStepper& stepper, MotorMode& mode, WebServer& server);
void stepperGetPositionAndSpeed(AccelStepper& stepper, WebServer& server);
void forkBeltSetSpeed(WebServer& server, int dirPin, int ledc);
void loadCellCalibrate(WebServer& server, HX711& scale, bool& calibration_mode, float& calibration_factor, float& calibration_step);
void loadCellRead(HX711& scale, WebServer& server, bool& calibration_mode, float& calibration_factor);

#endif
