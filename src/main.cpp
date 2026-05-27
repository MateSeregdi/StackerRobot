#include <Arduino.h>
#include <HX711.h>
#include <WiFi.h>
#include <WebServer.h>
#include <AccelStepper.h>
#include <WebSocketsServer.h>

#include "enums.h"
#include "control.h"

#include "lightranger14.h"

#define STEPPER_ENABLE 19

#define LOAD_CELL_DAT 18
#define LOAD_CELL_CLK 5

#define OPTICAL_SENSOR 32

#define MAIN_BELT_STEP        14
#define MAIN_BELT_DIR         12
#define LEAD_SCREW_STEP       25
#define LEAD_SCREW_DIR        33
#define FORK_FORWARD_STEP     27
#define FORK_FORWARD_DIR      26

#define FORK_BELT_PWM         15
#define FORK_BELT_DIR         2

// TODO: assign actual ESP32 GPIO pin numbers
#define TOF_SCL   22
#define TOF_SDA   21
#define TOF_EN    4
#define TOF_INT   35

#define LIGHTRANGER14_MAP_WIDTH 16
#define LIGHTRANGER14_MAP_HEIGHT 16

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

// WS broadcast queue — all ws calls happen only in webServerLoop (core 0)
#define WS_QUEUE_ITEM_SIZE 128
#define WS_QUEUE_LENGTH    24

// ── Forward declarations ──────────────────────────────────────────────────────
void wsBroadcast(const String &msg);
void broadcastValues(long &prev_main_belt, long &prev_lead_screw, long &prev_fork_forward);
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

QueueHandle_t wsQueue;

bool isAutomaticMode = false;

volatile bool limitSwitchTriggered = false;
volatile bool isForkLimitSwitch    = false;
volatile bool tofReadingReady      = false;

LimitSwitchSource limitSwitchSource = NONE;

lightranger14_t tof;

struct ToFResult {
  float   distance[LIGHTRANGER14_MAP_WIDTH][LIGHTRANGER14_MAP_HEIGHT];
  uint8_t confidence[LIGHTRANGER14_MAP_WIDTH][LIGHTRANGER14_MAP_HEIGHT];
  bool    valid[LIGHTRANGER14_MAP_WIDTH][LIGHTRANGER14_MAP_HEIGHT];
};

ToFResult tofResult;
uint8_t   frameData[LIGHTRANGER14_FRAME_PAYLOAD_SIZE];
uint16_t  floorMap[LIGHTRANGER14_MAP_WIDTH][LIGHTRANGER14_MAP_HEIGHT];
bool      occupancy[LIGHTRANGER14_MAP_WIDTH][LIGHTRANGER14_MAP_HEIGHT];
bool      prevOccupancy[LIGHTRANGER14_MAP_WIDTH][LIGHTRANGER14_MAP_HEIGHT];
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

// fork_forward
void handleForkForwardSetSpeed()         { stepperSetSpeed(fork_forward, fork_forward_mode, server); }
void handleForkForwardMoveTo()           { stepperMoveTo(fork_forward, fork_forward_mode, server); }
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

// ToF routes
void handleToFEnable() {
  lightranger14_enable_device(&tof);
  if (lightranger14_default_cfg(&tof) != 0) {
    server.send(400, "text/plain", "ToF sensor init failed");
    return;
  }
  tofEnabled = true;
  server.send(200, "text/plain", "ToF sensor enabled");
}

void handleToFDisable() {
  lightranger14_stop_measurement(&tof);
  lightranger14_disable_device(&tof);
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
  for (int y = 0; y < LIGHTRANGER14_MAP_HEIGHT; y++) {
    json += "[";
    for (int x = 0; x < LIGHTRANGER14_MAP_WIDTH; x++) {
      json += occupancy[y][x] ? "1" : "0";
      if (x < LIGHTRANGER14_MAP_WIDTH - 1) json += ",";
    }
    json += "]";
    if (y < LIGHTRANGER14_MAP_HEIGHT - 1) json += ",";
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
void IRAM_ATTR tofInterrupt() {
  tofReadingReady = true;
}

// ── ToF frame processing ──────────────────────────────────────────────────────
void readFrame() {
  if (lightranger14_clear_interrupts(&tof) != 0) {
    Serial.println("Interrupt clear failed");
    return;
  }
  if (lightranger14_read_results(&tof, frameData, (uint16_t)sizeof(frameData)) != 0) {
    Serial.println("Frame reading failed");
    return;
  }
  int pixelIndex = 0;
  for (int y = 0; y < LIGHTRANGER14_MAP_HEIGHT; y++) {
    for (int x = 0; x < LIGHTRANGER14_MAP_WIDTH; x++) {
      uint16_t rawDistance   = ((uint16_t)frameData[pixelIndex + 1] << 8) | frameData[pixelIndex];
      uint8_t  rawConfidence = frameData[pixelIndex + 2];
      tofResult.distance[y][x]   = rawDistance / 4.0f;
      tofResult.confidence[y][x] = rawConfidence;
      tofResult.valid[y][x]      = (rawConfidence > LIGHTRANGER14_CONFIDENCE_THRESHOLD);
      pixelIndex += 3;
    }
  }
}

void processFrame() {
  for (int y = 0; y < LIGHTRANGER14_MAP_HEIGHT; y++) {
    for (int x = 0; x < LIGHTRANGER14_MAP_WIDTH; x++) {
      if (!tofResult.valid[y][x]) continue;
      float height  = floorMap[y][x] - tofResult.distance[y][x];
      occupancy[y][x] = (height > 30);
    }
  }
}

void processCalibrateFrame() {
  for (int y = 0; y < LIGHTRANGER14_MAP_HEIGHT; y++) {
    for (int x = 0; x < LIGHTRANGER14_MAP_WIDTH; x++) {
      if (!tofResult.valid[y][x]) continue;
      floorMap[y][x] = (uint16_t)tofResult.distance[y][x];
    }
  }
  isFloorCalibrated = true;
}

void tofTask(void *parameter) {
  while (true) {
    if (tofEnabled && tofReadingReady) {
      tofReadingReady = false;
      readFrame();

      if (isCalibrationRequested) {
        processCalibrateFrame();
        isCalibrationRequested = false;
      } else if (isFloorCalibrated) {
        processFrame();

        bool changed = false;
        for (int y = 0; y < LIGHTRANGER14_MAP_HEIGHT && !changed; y++)
          for (int x = 0; x < LIGHTRANGER14_MAP_WIDTH && !changed; x++)
            if (occupancy[y][x] != prevOccupancy[y][x]) changed = true;

        if (changed) {
          String msg = "tof/occupancy:";
          for (int y = 0; y < LIGHTRANGER14_MAP_HEIGHT; y++)
            for (int x = 0; x < LIGHTRANGER14_MAP_WIDTH; x++)
              msg += occupancy[y][x] ? "1" : "0";
          wsBroadcast(msg);
          memcpy(prevOccupancy, occupancy, sizeof(occupancy));
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
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

  // TODO: calculate lead_screw target from ToF width measurement
  long lead_screw_target_pos = 1000;
  lead_screw.moveTo(lead_screw_target_pos);
  fork_forward.moveTo(FORK_END_POSITION);

  if (lead_screw.currentPosition() != lead_screw_target_pos ||
      fork_forward.currentPosition() != FORK_END_POSITION) return;

  int distance = analogRead(OPTICAL_SENSOR);

  if (distance < OPTICAL_SENSOR_THRESHOLD && !objectPassed) return;
  objectPassed = true;
  if (distance > OPTICAL_SENSOR_THRESHOLD && objectPassed) return;

  fork_forward.moveTo(0);
  if (fork_forward.currentPosition() != 0) {
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
    broadcastValues(prev_main_belt, prev_lead_screw, prev_fork_forward);

    if (isAutomaticMode) automaticMode();

    vTaskDelay(1);
  }
}

void broadcastValues(long &prev_main_belt, long &prev_lead_screw, long &prev_fork_forward) {
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
  if (!fork_forward.isRunning() && !lead_screw.isRunning() && !main_belt.isRunning())
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
  } else {
    lead_screw.stop();
    isAutomaticMode   = false;
    limitSwitchSource = LEAD_SCREW;
  }
  wsBroadcast(String("limit_switch/Limit switch was hit by ") +
              limitSwitchSourceStrings[limitSwitchSource] +
              ", enter manual mode and reset the position");
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

  lightranger14_cfg_t tof_cfg;
  lightranger14_cfg_setup(&tof_cfg);
  tof_cfg.scl     = TOF_SCL;
  tof_cfg.sda     = TOF_SDA;
  tof_cfg.en      = TOF_EN;
  tof_cfg.int_pin = TOF_INT;
  lightranger14_init(&tof, &tof_cfg);

  // Sensor starts disabled; use /tof/enable to start it
  lightranger14_disable_device(&tof);
  attachInterrupt(digitalPinToInterrupt(TOF_INT), tofInterrupt, FALLING);

  xTaskCreatePinnedToCore(webServerLoop,   "WebServer",   10000, NULL, 1, &WebServerHandler,   0);
  xTaskCreatePinnedToCore(electronicsLoop, "Electronics", 10000, NULL, 2, &ElectronicsHandler,  1);
  xTaskCreatePinnedToCore(tofTask,         "ToF",         10000, NULL, 3, &ToFHandler,          1);
}

void limitSwitchSetup() {
  pinMode(FORK_FORWARD_LIMIT_SWITCHES, INPUT_PULLUP);
  pinMode(LEAD_SCREW_LIMIT_SWITCHES,   INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FORK_FORWARD_LIMIT_SWITCHES), onForkForwardLimitSwitch, FALLING);
  attachInterrupt(digitalPinToInterrupt(LEAD_SCREW_LIMIT_SWITCHES),   onLeadScrewLimitSwitch,   FALLING);
}

void serverSetup() {
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

  server.on("/tof/enable",                    handleToFEnable);
  server.on("/tof/disable",                   handleToFDisable);
  server.on("/tof/calibrate",                 handleToFCalibration);
  server.on("/tof/data",                      handleToFData);

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
  ledcAttachPin(FORK_BELT_PWM, FORK_BELT_LEDC_CH);
  pinMode(FORK_BELT_DIR, OUTPUT);
  digitalWrite(FORK_BELT_DIR, LOW);
}

void stepperSetup() {
  main_belt.setMaxSpeed(STEPPER_MAX_SPEED);
  main_belt.setAcceleration(STEPPER_ACCELERATION);
  lead_screw.setMaxSpeed(STEPPER_MAX_SPEED);
  lead_screw.setAcceleration(STEPPER_ACCELERATION);
  fork_forward.setMaxSpeed(STEPPER_MAX_SPEED);
  fork_forward.setAcceleration(STEPPER_ACCELERATION);
  pinMode(STEPPER_ENABLE, OUTPUT);
  digitalWrite(STEPPER_ENABLE, LOW);
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
