#include <Arduino.h>
#include <HX711.h>
#include <WiFi.h>
#include <WebServer.h>
#include <AccelStepper.h>
#include <WebSocketsServer.h>

#include "enums.h"
#include "control.h"

// ── Pin definitions ───────────────────────────────────────────────────────────
#define LOAD_CELL_DAT 4
#define LOAD_CELL_CLK 2

#define OPTICAL_SENSOR 10

#define MAIN_BELT_STEP        14   
#define MAIN_BELT_DIR         18   
#define LEAD_SCREW_STEP       32   
#define LEAD_SCREW_DIR        33  
#define FORK_FORWARD_STEP     26   
#define FORK_FORWARD_DIR      25  

#define FORK_BELT_PWM         27   
#define FORK_BELT_DIR         13   

#define FORK_BELT_LEDC_CH   0
#define FORK_BELT_LEDC_FREQ 5000
#define FORK_BELT_LEDC_RES  8    // 8-bit → duty 0-255

#define STEPPER_MAX_SPEED    300.0f   // steps/sec
#define STEPPER_ACCELERATION 100.0f  // steps/sec²

#define LOAD_CELL_THRESHOLD 100
#define OPTICAL_SENSOR_THRESHOLD 600

#define FORK_END_POSITION 1000

#define DRIVER_MODE AccelStepper::DRIVER

// ── Globals ───────────────────────────────────────────────────────────────────
TaskHandle_t WebServerHandler;
TaskHandle_t ElectronicsHandler;

HX711 scale;

AccelStepper main_belt   (DRIVER_MODE, MAIN_BELT_STEP,    MAIN_BELT_DIR);
AccelStepper lead_screw  (DRIVER_MODE, LEAD_SCREW_STEP,   LEAD_SCREW_DIR);
AccelStepper fork_forward(DRIVER_MODE, FORK_FORWARD_STEP, FORK_FORWARD_DIR);

MotorMode main_belt_mode    = POSITION_MODE;
MotorMode lead_screw_mode   = POSITION_MODE;
MotorMode fork_forward_mode = POSITION_MODE;

float calibration_factor = -7050;
bool  calibration_mode   = false;
float calibration_step   = 10.0;

const char* WIFI_SSID     = "hh";
const char* WIFI_PASSWORD = "asd12345";

WebSocketsServer ws(81);
WebServer server(80);

bool isAutomaticMode = false;

// Routes
void handleRoot() { server.send(200, "text/plain", "Server is running"); }

// main_belt
void handleMainBeltSetSpeed()    { stepperSetSpeed(main_belt, main_belt_mode, server); }
void handleMainBeltMoveTo()      { stepperMoveTo(main_belt, main_belt_mode, server); }
void handleMainBeltPositionAndSpeed()    { stepperGetPositionAndSpeed(main_belt, server); }

// lead_screw
void handleLeadScrewSetSpeed()   { stepperSetSpeed(lead_screw, lead_screw_mode, server); }
void handleLeadScrewMoveTo()     { stepperMoveTo(lead_screw, lead_screw_mode, server); }
void handleLeadScrewPositionAndSpeed()   { stepperGetPositionAndSpeed(lead_screw, server); }

// fork_forward
void handleForkForwardSetSpeed() { stepperSetSpeed(fork_forward, fork_forward_mode, server); }
void handleForkForwardMoveTo()   { stepperMoveTo(fork_forward, fork_forward_mode, server); }
void handleForkForwardPositionAndSpeed() { stepperGetPositionAndSpeed(fork_forward, server); }

void handleLoadCellCalibration() {
    loadCellCalibrate(server, scale, calibration_mode, calibration_factor, calibration_step);
}
void handleLoadCellRead() {
    loadCellRead(scale, server, calibration_mode, calibration_factor);
}
void handleForkBeltSetSpeed() {
    forkBeltSetSpeed(server, FORK_BELT_DIR, FORK_BELT_LEDC_CH);
}


// Auto Mode

bool weightDetected = false;
bool objectPassed = false;

void automaticMode() {
  //Load cell readings
      //start belt
      //read par
      //adjust fork
      //move fork out
      //wait for optical sensor
      //start fork belt and move fork back
    main_belt_mode = POSITION_MODE;
    lead_screw_mode = POSITION_MODE;
    fork_forward_mode = POSITION_MODE;

    long main_belt_pos_at_start;

    float load_cell_value = 0;
    if (load_cell_value < LOAD_CELL_THRESHOLD || !weightDetected) {
      scale.set_scale(calibration_factor);
      load_cell_value = scale.get_units(10);
      main_belt_pos_at_start = main_belt.currentPosition();
      delay(1000);
      return;
    }
    weightDetected = true;
    //do {
    //  main_belt.runSpeed();
    //} while (par reading logic)

    if(main_belt.currentPosition() < main_belt_pos_at_start + 1000) {
      return;
    }

    //tof reading

    float width = 100;
    float length = 100;

    //TODO: calculate how width changes by step sizes in the lead screw

    long lead_screw_target_pos = 1000;
    lead_screw.moveTo(lead_screw_target_pos);

    fork_forward.moveTo(FORK_END_POSITION);
    
    if (lead_screw.currentPosition() != lead_screw_target_pos || fork_forward.currentPosition() != FORK_END_POSITION) {
      return;
    }

    int distance = analogRead(OPTICAL_SENSOR); //calc voltage to distance

    if (distance < OPTICAL_SENSOR_THRESHOLD && !objectPassed) {
      return;
    }
    objectPassed = true;
    if (distance > OPTICAL_SENSOR_THRESHOLD && objectPassed) {
      return;
    }
    fork_forward.moveTo(0);
    if (fork_forward.currentPosition() != 0) {
      ledcWrite(FORK_BELT_LEDC_CH, 255);
      return;
    }
    objectPassed =false;
    weightDetected = false;

}

//Main Loops
void webServerLoop(void *parameter) {
  for (;;) {
    ws.loop();
    server.handleClient();
  }
}

void electronicsLoop(void *parameter) {
  long prev_main_belt    = main_belt.currentPosition();
  long prev_lead_screw   = lead_screw.currentPosition();
  long prev_fork_forward = fork_forward.currentPosition();

  for (;;) {
    if (main_belt_mode == SPEED_MODE)    main_belt.runSpeed();
    else                                 main_belt.run();

    if (lead_screw_mode == SPEED_MODE)   lead_screw.runSpeed();
    else                                 lead_screw.run();

    if (fork_forward_mode == SPEED_MODE) fork_forward.runSpeed();
    else                                 fork_forward.run();

    long pos;
    float spd;

    pos = main_belt.currentPosition();
    spd = main_belt.speed();

    if (pos != prev_main_belt) {
      ws.broadcastTXT("motor/main_belt/speed:" + String(spd) + ",position:" + String(pos));
      prev_main_belt = pos;
    }

    pos = lead_screw.currentPosition();
    spd = lead_screw.speed();

    if (pos != prev_lead_screw) {
      ws.broadcastTXT("motor/lead_screw/speed:" + String(spd) + ",position:" + String(pos));
      prev_lead_screw = pos;
    }

    pos = fork_forward.currentPosition();
    spd = fork_forward.speed();

    if (pos != prev_fork_forward) {
      ws.broadcastTXT("motor/fork_forward/speed:" + String(spd) + ",position:" + String(pos));
      prev_fork_forward = pos;
    }

    if (isAutomaticMode) {
      automaticMode();
    }

    vTaskDelay(1);
  }
}


// ── setup ─────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    delay(1000);

    // Load cell
    scale.begin(LOAD_CELL_DAT, LOAD_CELL_CLK);
    if (scale.wait_ready_timeout(2000)) {
        scale.set_scale(calibration_factor);
        scale.tare();
        Serial.println("HX711 ready");
    } else {
        Serial.println("HX711 not found, continuing without scale");
    }

    // Steppers — max speed and acceleration apply only during position moves;
    // in speed mode setSpeed() overrides these constraints.
    main_belt.setMaxSpeed(STEPPER_MAX_SPEED);
    main_belt.setAcceleration(STEPPER_ACCELERATION);

    lead_screw.setMaxSpeed(STEPPER_MAX_SPEED);
    lead_screw.setAcceleration(STEPPER_ACCELERATION);

    fork_forward.setMaxSpeed(STEPPER_MAX_SPEED);
    fork_forward.setAcceleration(STEPPER_ACCELERATION);

    // DC motor
    ledcSetup(FORK_BELT_LEDC_CH, FORK_BELT_LEDC_FREQ, FORK_BELT_LEDC_RES);
    ledcAttachPin(FORK_BELT_PWM, FORK_BELT_LEDC_CH);
    pinMode(FORK_BELT_DIR, OUTPUT);
    digitalWrite(FORK_BELT_DIR, LOW);

    // WiFi
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to WiFi");
    uint32_t t = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t < 10000) {
      delay(500);
      Serial.print(".");
    }
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("\nWiFi failed — rebooting");
      ESP.restart();
    }
    Serial.println("\nWiFi connected. IP: " + WiFi.localIP().toString());

    // Routes
    server.on("/",                              handleRoot);
    server.on("/load_cell/calibrate",           handleLoadCellCalibration);
    server.on("/load_cell/read",                handleLoadCellRead);

    server.on("/motor/main_belt/set_speed",     handleMainBeltSetSpeed);
    server.on("/motor/main_belt/move_to",       handleMainBeltMoveTo);
    server.on("/motor/main_belt/position",      handleMainBeltPositionAndSpeed);

    server.on("/motor/lead_screw/set_speed",    handleLeadScrewSetSpeed);
    server.on("/motor/lead_screw/move_to",      handleLeadScrewMoveTo);
    server.on("/motor/lead_screw/position",     handleLeadScrewPositionAndSpeed);

    server.on("/motor/fork_forward/set_speed",  handleForkForwardSetSpeed);
    server.on("/motor/fork_forward/move_to",    handleForkForwardMoveTo);
    server.on("/motor/fork_forward/position",   handleForkForwardPositionAndSpeed);

    server.on("/motor/fork_belt/set_speed",     handleForkBeltSetSpeed);

    server.begin();
    ws.begin();
    Serial.println("HTTP/WebSocket server started");

    xTaskCreatePinnedToCore(webServerLoop,    "WebServer",   10000, NULL, 1, &WebServerHandler,    0);
    xTaskCreatePinnedToCore(electronicsLoop,  "Electronics", 10000, NULL, 2, &ElectronicsHandler,  1);
}

// ── loop ──────────────────────────────────────────────────────────────────────

void loop() {
    vTaskDelete(NULL);
}



