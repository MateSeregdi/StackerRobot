#include <Arduino.h>
#include <HX711.h>
#include <WiFi.h>
#include <WebServer.h>
#include <AccelStepper.h>
#include <WebSocketsServer.h>

#include "enums.h"
#include "control.h"

#include "tmf8829_app.h"

#define STEPPER_ENABLE 19

#define LOAD_CELL_DAT 18
#define LOAD_CELL_CLK 12

#define OPTICAL_SENSOR 32

#define MAIN_BELT_STEP        14
#define MAIN_BELT_DIR         13
#define LEAD_SCREW_STEP       25
#define LEAD_SCREW_DIR        33
#define FORK_FORWARD_STEP     27
#define FORK_FORWARD_DIR      26

#define FORK_BELT_IN1         23
#define FORK_BELT_IN2         4

#define UART_BAUD_RATE  115200
#define I2C_CLK_SPEED   400000
#define GRID_SIZE       16   

#define FORK_BELT_LEDC_CH   0
#define FORK_BELT_LEDC_FREQ 5000
#define FORK_BELT_LEDC_RES  8    // 8-bit → duty 0-255

#define STEPPER_MAX_SPEED    100.0f
#define STEPPER_ACCELERATION 30.0f

#define LOAD_CELL_THRESHOLD 100
#define OPTICAL_SENSOR_THRESHOLD 600

#define FORK_END_POSITION 1000

#define FORK_FORWARD_LIMIT_SWITCHES 16
#define LEAD_SCREW_LIMIT_SWITCHES 17

#define FORK_FORWARD_MID_POSITION 200

#define DRIVER_MODE AccelStepper::DRIVER


// ── ToF geometry ──────────────────────────────────────────────────────────────
#define TOF_SENSOR_HEIGHT_MM    800.0f  // sensor mount height above floor (mm) — adjust to your rig
#define TOF_FOV_DEG             44.0f   // TMF8829 FoV per axis in degrees
#define LEAD_SCREW_STEPS_PER_MM  10.0f  // lead screw steps per mm — tune to your hardware
#define HOME_FORK_GAP_MM 0
#define FORK_CLEARANCE_MM 5


#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// WS broadcast queue — all ws calls happen only in webServerLoop (core 0)
#define WS_QUEUE_ITEM_SIZE 300  // must fit "tof/occupancy:" + 256 chars = 270 + margin
#define WS_QUEUE_LENGTH    24

// ── Forward declarations ──────────────────────────────────────────────────────
void wsBroadcast(const String &msg);
void broadcastMotorValues(long &prev_main_belt, long &prev_lead_screw, long &prev_fork_forward);
void runMotors();
void setEnable();
void limitSwitchCheck(long prev_fork_forward);
void loadCellSetup();
void stepperSetup();
void dcMotorSetup();
void wifiSetup();
void serverSetup();
void limitSwitchSetup();

// ── Globals ───────────────────────────────────────────────────────────────────
TaskHandle_t WebServerHandler;
TaskHandle_t ElectronicsHandler;
TaskHandle_t ToFHandler;
TaskHandle_t LoadCellHandler;

HX711 scale;

AccelStepper main_belt   (DRIVER_MODE, MAIN_BELT_STEP,    MAIN_BELT_DIR);
AccelStepper lead_screw  (DRIVER_MODE, LEAD_SCREW_STEP,   LEAD_SCREW_DIR);
AccelStepper fork_forward(DRIVER_MODE, FORK_FORWARD_STEP, FORK_FORWARD_DIR);

MotorMode main_belt_mode    = POSITION_MODE;
MotorMode lead_screw_mode   = POSITION_MODE;
MotorMode fork_forward_mode = POSITION_MODE;

float calibration_factor = -7050;

const char* WIFI_SSID     = "hh";
const char* WIFI_PASSWORD = "asd12345";

WebSocketsServer ws(81);
WebServer server(80);

QueueHandle_t wsQueue;

bool isAutomaticMode = false;

volatile bool limitSwitchTriggered = false;
volatile bool isForkLimitSwitch    = false;

LimitSwitchSource limitSwitchSource = NONE;

bool osEnabled = false;

bool isHomingMode = false;
bool isLeadScrewHomed = false;

volatile bool tarePending = false;



uint16_t  floorMap[GRID_SIZE][GRID_SIZE];
bool      occupancy[GRID_SIZE][GRID_SIZE];
bool      prevOccupancy[GRID_SIZE][GRID_SIZE];
bool      isFloorCalibrated    = false;
bool      tofEnabled           = false;
volatile bool isCalibrationRequested = false;

// ── WebSocket helper (thread-safe via queue) ──────────────────────────────────
void wsBroadcast(const String &msg) {
  char buf[WS_QUEUE_ITEM_SIZE];
  msg.toCharArray(buf, sizeof(buf));
  xQueueSend(wsQueue, buf, 0);
}

// ── HTTP Routes ───────────────────────────────────────────────────────────────
void handleRoot() { server.send(200, "text/plain", "Server is running"); }

// main_belt
void handleMainBeltSetSpeed()         { stepperSetSpeed(main_belt, main_belt_mode, server); }
void handleMainBeltMoveTo()           { stepperMoveTo(main_belt, main_belt_mode, server); }
void handleMainBeltPositionAndSpeed() { stepperGetPositionAndSpeed(main_belt, server); }

// lead_screw
void handleLeadScrewSetSpeed()         { stepperSetSpeed(lead_screw, lead_screw_mode, server); }
void handleLeadScrewMoveTo()           { stepperMoveTo(lead_screw, lead_screw_mode, server); }
void handleLeadScrewPositionAndSpeed() { stepperGetPositionAndSpeed(lead_screw, server); }
void handleLeadScrewHome() {homeLeadScrew(isHomingMode, lead_screw);}

// fork_forward
void handleForkForwardSetSpeed()         { stepperSetSpeed(fork_forward, fork_forward_mode, server); }
void handleForkForwardMoveTo()           { stepperMoveTo(fork_forward, fork_forward_mode, server); }
void handleForkForwardPositionAndSpeed() { stepperGetPositionAndSpeed(fork_forward, server); }

void handleTare() {
  tarePending = true;
  server.send(200, "text/plain", "Tare requested");
}
void handleGetCalibrationFactor() {
  server.send(200, "text/plain", String(calibration_factor));
}
void handleSetCalibrationFactor() {
  if (!server.hasArg("value")) {
    server.send(400, "text/plain", "Missing value");
    return;
  }
  calibration_factor = server.arg("value").toFloat();
  scale.set_scale(calibration_factor);
  server.send(200, "text/plain", String(calibration_factor));
  wsBroadcast("load_cell/calibration_factor/" + String(calibration_factor));
}
void handleForkBeltSetSpeed() {
  forkBeltSetSpeed(server, FORK_BELT_IN1, FORK_BELT_IN2, FORK_BELT_LEDC_CH);
}
void handleOpticalSensorEnable()  { osEnabled = true;  server.send(200, "text/plain", "Optical sensor enabled"); }
void handleOpticalSensorDisable() { osEnabled = false; server.send(200, "text/plain", "Optical sensor disabled"); }

void handleAutoEnable()  { isAutomaticMode = true;  server.send(200, "text/plain", "Automatic mode enabled"); }
void handleAutoDisable() { isAutomaticMode = false; server.send(200, "text/plain", "Automatic mode disabled"); }

// ToF routes
void handleToFEnable() {
  autoStartFn();
  setResolutionFn(TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CFG_16X16);
  tofEnabled = true;
  server.send(200, "text/plain", "ToF sensor enabled");
}

void handleToFDisable() {
  disableFn();
  tofEnabled = false;
  server.send(200, "text/plain", "ToF sensor disabled");
}

void handleToFCalibration() {
  if (!tofEnabled) {
    server.send(400, "text/plain", "ToF sensor not enabled");
    return;
  }
  isCalibrationRequested = true;
  server.send(200, "text/plain", "Calibration triggered");
}

void handleToFData() {
  String json = "{\"calibrated\":";
  json += isFloorCalibrated ? "true" : "false";
  json += ",\"occupancy\":[";
  for (int y = 0; y < GRID_SIZE; y++) {
    json += "[";
    for (int x = 0; x < GRID_SIZE; x++) {
      json += occupancy[y][x] ? "1" : "0";
      if (x < GRID_SIZE - 1) json += ",";
    }
    json += "]";
    if (y < GRID_SIZE - 1) json += ",";
  }
  json += "]}";
  server.send(200, "application/json", json);
}

// ── ISRs ─────────────────────────────────────────────────────────────────────
void IRAM_ATTR onForkForwardLimitSwitch() {
  limitSwitchTriggered = true;
  isForkLimitSwitch    = true;
}
void IRAM_ATTR onLeadScrewLimitSwitch() {
  limitSwitchTriggered = true;
}

void processFrame() {
  for (int y = 0; y < GRID_SIZE; y++) {
    for (int x = 0; x < GRID_SIZE; x++) {
      uint16_t zone = y * GRID_SIZE + x;
      if (tofResults.confidence[zone] == 0) continue;
      int32_t height = (int32_t)floorMap[y][x] - (int32_t)tofResults.distance_mm[zone];
      occupancy[y][x] = (height > 30);
    }
  }
}

void processCalibrateFrame() {
  for (int y = 0; y < GRID_SIZE; y++) {
    for (int x = 0; x < GRID_SIZE; x++) {
      uint16_t zone = y * GRID_SIZE + x;
      if (tofResults.confidence[zone] == 0) continue;
      floorMap[y][x] = tofResults.distance_mm[zone];
    }
  }
  isFloorCalibrated = true;
}

void tofTask(void *parameter) {
  while (true) {
    if (tofResults.ready) {
      tofResults.ready = false;   
      if (isCalibrationRequested) {
        processCalibrateFrame();
        isCalibrationRequested = false;
      } else if (isFloorCalibrated) {
        processFrame();

        bool changed = false;
        for (int y = 0; y < GRID_SIZE && !changed; y++)
          for (int x = 0; x < GRID_SIZE && !changed; x++)
            if (occupancy[y][x] != prevOccupancy[y][x]) changed = true;

        if (changed) {
          String msg = "tof/occupancy:";
          for (int y = 0; y < GRID_SIZE; y++)
            for (int x = 0; x < GRID_SIZE; x++)
              msg += occupancy[y][x] ? "1" : "0";
          wsBroadcast(msg);
          memcpy(prevOccupancy, occupancy, sizeof(occupancy));
        }
      }

    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void loadCellTask(void *parameter) {
  const float THRESHOLD = 0.5f;
  float prev = 0.0f;
  for (;;) {
    if (tarePending) {
      scale.tare(10);
      tarePending = false;
      prev = 0.0f;
      wsBroadcast("load_cell/reading/0.00");
    } else {
      scale.set_scale(calibration_factor);
      float value = scale.get_units(3);
      if (fabsf(value - prev) >= THRESHOLD) {
        wsBroadcast("load_cell/reading/" + String(value, 2));
        prev = value;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(150));
  }
}

// ── ToF width measurement ─────────────────────────────────────────────────────
// Scans occupancy[][], finds the leftmost and rightmost occupied column, then
// computes the physical width of that span at the measured box-top distance.
// Returns 0 if no occupied zones are found.
// Zones seeing the box top are at the minimum distance. Zones seeing the box
// side wall are farther away. We use this tolerance (mm) to separate them.
#define TOF_WALL_TOLERANCE_MM 40

float calculateBoxWidthMm()
{
  // Pass 1: find the minimum distance among all occupied zones (= box top surface)
  uint16_t min_dist = 0xFFFF;
  for (int y = 0; y < GRID_SIZE; y++) {
    for (int x = 0; x < GRID_SIZE; x++) {
      if (!occupancy[y][x]) continue;
      uint16_t zone = (uint16_t)(y * GRID_SIZE + x);
      if (tofResults.distance_mm[zone] < min_dist)
        min_dist = tofResults.distance_mm[zone];
    }
  }
  if (min_dist == 0xFFFF) return 0.0f;

  // Pass 2: use only top-surface zones (distance ≤ min + tolerance)
  // Side-wall zones are always farther than the flat top, so they get excluded.
  uint16_t dist_threshold = min_dist + TOF_WALL_TOLERANCE_MM;
  int   col_min  = GRID_SIZE;
  int   col_max  = -1;
  float sum_dist = 0.0f;
  int   count    = 0;

  for (int y = 0; y < GRID_SIZE; y++) {
    for (int x = 0; x < GRID_SIZE; x++) {
      if (!occupancy[y][x]) continue;
      uint16_t zone = (uint16_t)(y * GRID_SIZE + x);
      if (tofResults.distance_mm[zone] > dist_threshold) continue; // side wall — skip
      if (x < col_min) col_min = x;
      if (x > col_max) col_max = x;
      sum_dist += tofResults.distance_mm[zone];
      count++;
    }
  }

  if (count == 0) return 0.0f;

  float box_dist_mm  = sum_dist / (float)count;
  float deg_per_zone = TOF_FOV_DEG / (float)GRID_SIZE;
  float half_grid    = GRID_SIZE / 2.0f;

  float left_rad  = (col_min        - half_grid) * deg_per_zone * (M_PI / 180.0f);
  float right_rad = (col_max + 1.0f - half_grid) * deg_per_zone * (M_PI / 180.0f);

  return box_dist_mm * (tanf(right_rad) - tanf(left_rad));
}

void measureOpticalSensor() {
  static uint32_t lastSent = 0;
  if (millis() - lastSent < 100) return;
  lastSent = millis();

  int raw = analogRead(OPTICAL_SENSOR);
  float volts = raw * (3.3f / 4095.0f);
  if (volts <= 0.0f) return;
  float distance = (13.5f / volts) - 0.42f;
  if (distance < 4.0f || distance > 28.0f) distance = -1.0f;
  wsBroadcast("optical_sensor/" + String(distance));
}

// ── Auto Mode ─────────────────────────────────────────────────────────────────
bool weightDetected       = false;
bool objectPassed         = false;
long main_belt_pos_at_start;

void automaticMode() {
  main_belt_mode    = POSITION_MODE;
  lead_screw_mode   = POSITION_MODE;
  fork_forward_mode = POSITION_MODE;

  scale.set_scale(calibration_factor);
  float load_cell_value = scale.get_units(10);
  if (load_cell_value < LOAD_CELL_THRESHOLD || !weightDetected) {
    main_belt_pos_at_start = main_belt.currentPosition();
    delay(1000);
    return;
  }
  weightDetected = true;

  if (main_belt.currentPosition() < main_belt_pos_at_start + 1000) return;

  float box_width_mm          = calculateBoxWidthMm();

  float gap_needed_mm = box_width_mm - HOME_FORK_GAP_MM + FORK_CLEARANCE_MM;
  long lead_screw_target_pos = (long)(gap_needed_mm * LEAD_SCREW_STEPS_PER_MM);

  lead_screw.moveTo(lead_screw_target_pos);
  fork_forward.moveTo(FORK_END_POSITION);

  if (lead_screw.currentPosition() != lead_screw_target_pos ||
      fork_forward.currentPosition() != FORK_END_POSITION) return;

  int raw = analogRead(OPTICAL_SENSOR);
  float volts = raw * (5.0 / 1023.0); 
  float distance = (13.5 / volts) - 0.42;

  if (distance < OPTICAL_SENSOR_THRESHOLD && !objectPassed) return;
  objectPassed = true;
  if (distance > OPTICAL_SENSOR_THRESHOLD && objectPassed) return;

  fork_forward.moveTo(0);
  if (fork_forward.currentPosition() != 0) {
    digitalWrite(FORK_BELT_IN2, LOW);
    ledcAttachPin(FORK_BELT_IN1, FORK_BELT_LEDC_CH);
    ledcWrite(FORK_BELT_LEDC_CH, 255);
    return;
  }
  objectPassed  = false;
  weightDetected = false;
}

// ── Main loops ────────────────────────────────────────────────────────────────
void webServerLoop(void *parameter) {
  char buf[WS_QUEUE_ITEM_SIZE];
  for (;;) {
    // Drain the broadcast queue before processing WS events
    while (xQueueReceive(wsQueue, buf, 0) == pdTRUE) {
      ws.broadcastTXT(buf);
    }
    ws.loop();
    server.handleClient();
  }
}

void electronicsLoop(void *parameter) {
  long prev_main_belt    = main_belt.currentPosition();
  long prev_lead_screw   = lead_screw.currentPosition();
  long prev_fork_forward = fork_forward.currentPosition();

  for (;;) {
    setEnable();
    limitSwitchCheck(prev_fork_forward);
    runMotors();
    broadcastMotorValues(prev_main_belt, prev_lead_screw, prev_fork_forward);
    loopFn();

    if (osEnabled) measureOpticalSensor();

    if (isAutomaticMode) automaticMode();

    vTaskDelay(1);
  }
}

void broadcastMotorValues(long &prev_main_belt, long &prev_lead_screw, long &prev_fork_forward) {
  long  pos;
  float spd;

  pos = main_belt.currentPosition();
  spd = main_belt.speed();
  if (pos != prev_main_belt) {
    wsBroadcast("motor/main_belt/speed:" + String(spd) + ",position:" + String(pos));
    prev_main_belt = pos;
  }

  pos = lead_screw.currentPosition();
  spd = lead_screw.speed();
  if (pos != prev_lead_screw) {
    wsBroadcast("motor/lead_screw/speed:" + String(spd) + ",position:" + String(pos));
    prev_lead_screw = pos;
  }

  pos = fork_forward.currentPosition();
  spd = fork_forward.speed();
  if (pos != prev_fork_forward) {
    wsBroadcast("motor/fork_forward/speed:" + String(spd) + ",position:" + String(pos));
    prev_fork_forward = pos;
  }
}

void runMotors() {
  if (main_belt_mode == SPEED_MODE) main_belt.runSpeed(); else main_belt.run();
  if (lead_screw_mode == SPEED_MODE) lead_screw.runSpeed(); else lead_screw.run();
  if (fork_forward_mode == SPEED_MODE) fork_forward.runSpeed(); else fork_forward.run();
}

void setEnable() {
  if (fork_forward.speed() == 0.0f && lead_screw.speed() == 0.0f && main_belt.speed() == 0.0f)
    digitalWrite(STEPPER_ENABLE, HIGH);
  else
    digitalWrite(STEPPER_ENABLE, LOW);
}

void limitSwitchCheck(long prev_fork_forward) {
  if (!limitSwitchTriggered) return;

  if (isForkLimitSwitch) {
    fork_forward.stop();
    isAutomaticMode   = false;
    limitSwitchSource = (prev_fork_forward > FORK_FORWARD_MID_POSITION)
                          ? FORK_FORWARD_END : FORK_FORWARD_START;
  } else if (isHomingMode) {
    lead_screw.stop();
    lead_screw.setCurrentPosition(0);
  } else {
    lead_screw.stop();
    isAutomaticMode   = false;
    limitSwitchSource = LEAD_SCREW;
  }
  if (!isHomingMode) {
      wsBroadcast(String("limit_switch/Limit switch was hit by ") +
              limitSwitchSourceStrings[limitSwitchSource] +
              ", enter manual mode and reset the position");
  }
  else {
    wsBroadcast(String("lead_screw_homed"));
    isHomingMode = false;
  }

  limitSwitchTriggered = false;
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1000);

  wsQueue = xQueueCreate(WS_QUEUE_LENGTH, WS_QUEUE_ITEM_SIZE);

  loadCellSetup();
  stepperSetup();
  dcMotorSetup();
  wifiSetup();
  serverSetup();
  limitSwitchSetup();

  setupFn( 0, UART_BAUD_RATE, I2C_CLK_SPEED );  // sensor stays disabled until /tof/enable


  xTaskCreatePinnedToCore(webServerLoop,   "WebServer",   10000, NULL, 1, &WebServerHandler,   0);
  xTaskCreatePinnedToCore(electronicsLoop, "Electronics", 10000, NULL, 2, &ElectronicsHandler,  1);
  xTaskCreatePinnedToCore(tofTask,         "ToF",         10000, NULL, 3, &ToFHandler,          1);
  xTaskCreatePinnedToCore(loadCellTask,    "LoadCell",     4096, NULL, 1, &LoadCellHandler,     1);
}

void limitSwitchSetup() {
  pinMode(FORK_FORWARD_LIMIT_SWITCHES, INPUT_PULLUP);
  pinMode(LEAD_SCREW_LIMIT_SWITCHES,   INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FORK_FORWARD_LIMIT_SWITCHES), onForkForwardLimitSwitch, FALLING);
  attachInterrupt(digitalPinToInterrupt(LEAD_SCREW_LIMIT_SWITCHES),   onLeadScrewLimitSwitch,   FALLING);
}

void serverSetup() {
  server.on("/",                              handleRoot);
  server.on("/load_cell/tare",                   handleTare);
  server.on("/load_cell/calibration_factor",     handleGetCalibrationFactor);
  server.on("/load_cell/set_calibration_factor", handleSetCalibrationFactor);

  server.on("/motor/main_belt/set_speed",     handleMainBeltSetSpeed);
  server.on("/motor/main_belt/move_to",       handleMainBeltMoveTo);
  server.on("/motor/main_belt/position",      handleMainBeltPositionAndSpeed);

  server.on("/motor/lead_screw/set_speed",    handleLeadScrewSetSpeed);
  server.on("/motor/lead_screw/move_to",      handleLeadScrewMoveTo);
  server.on("/motor/lead_screw/position",     handleLeadScrewPositionAndSpeed);
  server.on("home_lead_screw", handleLeadScrewHome);

  server.on("/motor/fork_forward/set_speed",  handleForkForwardSetSpeed);
  server.on("/motor/fork_forward/move_to",    handleForkForwardMoveTo);
  server.on("/motor/fork_forward/position",   handleForkForwardPositionAndSpeed);

  server.on("/motor/fork_belt/set_speed",     handleForkBeltSetSpeed);

  server.on("/tof/enable",                    handleToFEnable);
  server.on("/tof/disable",                   handleToFDisable);
  server.on("/tof/calibrate",                 handleToFCalibration);
  server.on("/tof/data",                      handleToFData);

  server.on("/optical_sensor/enable",  handleOpticalSensorEnable);
  server.on("/optical_sensor/disable", handleOpticalSensorDisable);

  server.on("/auto/enable",  handleAutoEnable);
  server.on("/auto/disable", handleAutoDisable);




  server.begin();
  ws.begin();
  Serial.println("HTTP/WebSocket server started");
}

void wifiSetup() {
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
}

void dcMotorSetup() {
  ledcSetup(FORK_BELT_LEDC_CH, FORK_BELT_LEDC_FREQ, FORK_BELT_LEDC_RES);
  pinMode(FORK_BELT_IN1, OUTPUT);
  pinMode(FORK_BELT_IN2, OUTPUT);
  digitalWrite(FORK_BELT_IN1, LOW);
  digitalWrite(FORK_BELT_IN2, LOW);
}

void stepperSetup() {
  main_belt.setMaxSpeed(STEPPER_MAX_SPEED);
  main_belt.setAcceleration(STEPPER_ACCELERATION);
  lead_screw.setMaxSpeed(STEPPER_MAX_SPEED);
  lead_screw.setAcceleration(STEPPER_ACCELERATION);
  fork_forward.setMaxSpeed(STEPPER_MAX_SPEED);
  fork_forward.setAcceleration(STEPPER_ACCELERATION);
  pinMode(STEPPER_ENABLE, OUTPUT);
  digitalWrite(STEPPER_ENABLE, HIGH);
}

void loadCellSetup() {
  scale.begin(LOAD_CELL_DAT, LOAD_CELL_CLK);
  if (scale.wait_ready_timeout(2000)) {
    scale.set_scale(calibration_factor);
    scale.tare();
    Serial.println("HX711 ready");
  } else {
    Serial.println("HX711 not found, continuing without scale");
  }
}

// ── loop ──────────────────────────────────────────────────────────────────────
void loop() {
  vTaskDelete(NULL);
}
