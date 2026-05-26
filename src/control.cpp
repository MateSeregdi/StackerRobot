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

void forkBeltSetSpeed(WebServer& server, int dirPin, int ledc) {
    if (!server.hasArg("speed")) {
        server.send(400, "text/plain", "Missing speed (0-255)");
        return;
    }
    int spd  = constrain(server.arg("speed").toInt(), 0, 255);
    bool fwd = !server.hasArg("direction") || server.arg("direction") != "backward";

    digitalWrite(dirPin, fwd ? HIGH : LOW);
    ledcWrite(ledc, spd);

    server.send(200, "text/plain",
        "speed=" + String(spd) + " direction=" + (fwd ? "forward" : "backward"));
}

void loadCellCalibrate(WebServer& server, HX711& scale, bool& calibration_mode, float& calibration_factor, float& calibration_step) {
    if (!server.hasArg("action")) {
        server.send(400, "text/plain", "Missing action");
        return;
    }
    String action = server.arg("action");

    if (action == "start") {
        calibration_mode = true;
        scale.tare();
        scale.set_scale(calibration_factor);
        server.send(200, "text/plain", "Calibration started. Factor: " + String(calibration_factor));

    } else if (action == "increase") {
        if (!calibration_mode) { server.send(400, "text/plain", "Calibration not active"); return; }
        calibration_factor += calibration_step;
        scale.set_scale(calibration_factor);
        float reading = scale.get_units(5);
        server.send(200, "text/plain",
            String(calibration_factor) + ":" + String(reading));

    } else if (action == "decrease") {
        if (!calibration_mode) { server.send(400, "text/plain", "Calibration not active"); return; }
        calibration_factor -= calibration_step;
        scale.set_scale(calibration_factor);
        float reading = scale.get_units(5);
        server.send(200, "text/plain",
            String(calibration_factor) + ":" + String(reading));

    } else if (action == "stop") {
        calibration_mode = false;
        scale.set_scale(calibration_factor);
        server.send(200, "text/plain",
            "Calibration stopped. Final factor: " + String(calibration_factor));

    } else {
        server.send(400, "text/plain", "Invalid action");
    }
}

void loadCellRead(HX711& scale, WebServer& server, bool& calibration_mode, float& calibration_factor) {
    if (calibration_mode) {
        server.send(400, "text/plain", "Cannot read while calibration mode is active");
        return;
    }
    scale.set_scale(calibration_factor);
    float value = scale.get_units(10);
    server.send(200, "text/plain", String(value));
}
