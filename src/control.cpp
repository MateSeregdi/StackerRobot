#include <Arduino.h>
#include "control.h"

void stepperSetSpeed(AccelStepper& stepper, MotorMode& mode, WebServer& server) {
    if (!server.hasArg("steps_per_second")) {
        server.send(400, "text/plain", "Missing steps_per_second");
        return;
    }
    float sps = server.arg("steps_per_second").toFloat();
    mode = SPEED_MODE;
    stepper.setSpeed(sps);
    server.send(200, "text/plain",
        String(sps, 1) + " steps/sec (" + String(sps, 3) + " steps/sec)");
}

void stepperMoveTo(AccelStepper& stepper, MotorMode& mode, WebServer& server) {
    if (!server.hasArg("position")) {
        server.send(400, "text/plain", "Missing position");
        return; 
    }
    long target = server.arg("position").toInt();
    mode = POSITION_MODE;
    stepper.moveTo(target);
    server.send(200, "text/plain",
        "Moving to " + String(target) + "  (from " + String(stepper.currentPosition()) + ")");
}

void stepperGetPositionAndSpeed(AccelStepper& stepper, WebServer& server) {
    server.send(200, "text/plain", "position:" + String(stepper.currentPosition()) + ", speed:" + String(stepper.speed()));
}

void homeLeadScrew(bool& isHomingMode, AccelStepper& lead_screw) {
    isHomingMode = true;
    lead_screw.setSpeed(50);
}

void forkBeltSetSpeed(WebServer& server, int in1Pin, int in2Pin, int ledc) {
    if (!server.hasArg("speed") || !server.hasArg("direction")) {
        server.send(400, "text/plain", "Missing arguements for fork belt");
        return;
    }
    int spd  = constrain(server.arg("speed").toInt(), 0, 255);
    bool fwd = server.arg("direction") == "forward";

    if (spd == 0) {
        ledcDetachPin(in1Pin);
        ledcDetachPin(in2Pin);
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, LOW);
    } else if (fwd) {
        ledcDetachPin(in2Pin);
        digitalWrite(in2Pin, LOW);
        ledcAttachPin(in1Pin, ledc);
        ledcWrite(ledc, spd);
    } else {
        ledcDetachPin(in1Pin);
        digitalWrite(in1Pin, LOW);
        ledcAttachPin(in2Pin, ledc);
        ledcWrite(ledc, spd);
    }

    server.send(200, "text/plain",
        "speed=" + String(spd) + " direction=" + (fwd ? "forward" : "backward"));
}

