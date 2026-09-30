/*
  Raspberry Pi Pico W - Headband Tracker receiver + motor guidance

  Board/core:
    Arduino IDE -> Raspberry Pi Pico W (Earle Philhower Arduino-Pico core)

  Library:
    ArduinoJson 7.x (Library Manager)

  Network:
    SSID: Headband-Pico
    Password: headband123
    Pico IP: http://192.168.4.1

  Website endpoints:
    GET  /ping
    GET  /status
    POST /pair          first successful browser pairing starts one wiring test
    POST /motor-test    manually repeat wiring test
    POST /detections

  IMPORTANT MOTOR WIRING:
    Do NOT connect motors directly to Pico GPIO pins.
    Each GPIO must drive a suitable transistor/MOSFET motor stage, with a
    flyback diode across the motor and a common ground.

  Default motor mapping:
    GP2 = LEFT motor
    GP3 = CENTER/AUX motor (used in wiring self-test; reserved during guidance)
    GP4 = RIGHT motor

  Pairing self-test:
    LEFT -> CENTER/AUX -> RIGHT, 0.5 seconds each, with a short gap.

  Live guidance:
    target left of center  -> LEFT motor
    target right of center -> RIGHT motor
    target near center     -> LEFT + RIGHT together

  The website sends ONLY confirmed headband targets. Each target contains
  normalized coordinates (0.0 to 1.0) plus pixel centers and confidence.
*/

#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>

// ---------------- Wi-Fi access point ----------------
const char *AP_SSID = "Headband-Pico";
const char *AP_PASSWORD = "headband123"; // 8+ chars. Change before final use.

IPAddress AP_IP(192, 168, 4, 1);
IPAddress AP_GATEWAY(192, 168, 4, 1);
IPAddress AP_SUBNET(255, 255, 255, 0);

WebServer server(80);

// ---------------- Motor outputs ----------------
constexpr uint8_t LEFT_MOTOR_PIN = 2;
constexpr uint8_t CENTER_MOTOR_PIN = 3;
constexpr uint8_t RIGHT_MOTOR_PIN = 4;

constexpr uint8_t MOTOR_PINS[] = {
  LEFT_MOTOR_PIN,
  CENTER_MOTOR_PIN,
  RIGHT_MOTOR_PIN
};
constexpr uint8_t MOTOR_COUNT = sizeof(MOTOR_PINS) / sizeof(MOTOR_PINS[0]);

// MOSFET gate HIGH = motor on. Change only if your driver is active-low.
constexpr uint8_t MOTOR_ON_LEVEL = HIGH;
constexpr uint8_t MOTOR_OFF_LEVEL = LOW;

// Wiring-test timing.
constexpr uint32_t MOTOR_TEST_ON_MS = 500;
constexpr uint32_t MOTOR_TEST_GAP_MS = 150;

// Camera horizontal target zone. 0.0 = far left, 1.0 = far right.
// 0.40 .. 0.60 counts as "near center" and pulses both guidance motors.
constexpr float CENTER_ZONE_MIN = 0.40f;
constexpr float CENTER_ZONE_MAX = 0.60f;

// Vibration-rate model. Apparent head size is used as a rough distance proxy:
// larger normalized head crops are usually closer to the camera. This is not
// true depth sensing, so tune these two values for your camera/crop settings.
constexpr float FAR_HEAD_SIZE = 0.05f;   // normalized max(w,h) -> proximity 0
constexpr float NEAR_HEAD_SIZE = 0.30f;  // normalized max(w,h) -> proximity 1

// Distance dominates the pulse rate, while classification confidence provides
// a smaller contribution. Weights should add to 1.0.
constexpr float DISTANCE_WEIGHT = 0.80f;
constexpr float CONFIDENCE_WEIGHT = 0.20f;

// Guidance pulse range. Motors receive full drive during each pulse; the pulse
// repetition rate changes with proximity/confidence rather than PWM strength.
constexpr float MIN_GUIDANCE_HZ = 1.25f;
constexpr float MAX_GUIDANCE_HZ = 6.75f;
constexpr uint32_t MIN_PULSE_ON_MS = 65;
constexpr uint32_t MAX_PULSE_ON_MS = 140;

bool motorState[MOTOR_COUNT] = { false, false, false };
bool motorTestActive = false;
bool motorTestOnPhase = false;
uint8_t motorTestIndex = 0;
uint32_t motorTestPhaseStartMs = 0;
bool firstPairMotorTestDone = false;

// Live guidance pulse state. Bit 0 = LEFT, bit 2 = RIGHT. CENTER/AUX is kept
// off during tracking and is only used by the sequential wiring self-test.
uint8_t guidanceMotorMask = 0;
bool guidancePulseOn = false;
uint32_t guidancePhaseStartMs = 0;
uint32_t guidanceOnMs = 0;
uint32_t guidanceOffMs = 0;
float guidanceFrequencyHz = 0.0f;
float guidanceProximity = 0.0f;
float guidanceConfidence = 0.0f;

// ---------------- Detection receiver ----------------
constexpr uint8_t MAX_HEADBANDS = 16;
constexpr uint32_t DETECTION_TIMEOUT_MS = 750; // clear targets if browser stops sending

struct HeadbandTarget {
  float confidence = 0.0f;
  float cx = 0.0f; // normalized 0..1, left -> right
  float cy = 0.0f; // normalized 0..1, top -> bottom
  float x = 0.0f;
  float y = 0.0f;
  float w = 0.0f;
  float h = 0.0f;
  int cxPx = 0;
  int cyPx = 0;
};

HeadbandTarget targets[MAX_HEADBANDS];
uint8_t targetCount = 0;
uint32_t lastSequence = 0;
uint32_t lastDetectionPacketMs = 0;
bool detectionLinkAlive = false;
int8_t selectedTargetIndex = -1;

void setMotor(uint8_t index, bool on) {
  if (index >= MOTOR_COUNT) return;
  motorState[index] = on;
  digitalWrite(MOTOR_PINS[index], on ? MOTOR_ON_LEVEL : MOTOR_OFF_LEVEL);
}

void allMotorsOff() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) setMotor(i, false);
}

float clamp01(float value) {
  if (value < 0.0f) return 0.0f;
  if (value > 1.0f) return 1.0f;
  return value;
}

float estimateProximity(const HeadbandTarget &t) {
  // The website sends normalized crop dimensions. For the deployment-matched
  // square head crops, max(w,h) is a useful apparent-size proxy for distance.
  const float apparentSize = max(t.w, t.h);
  const float span = max(0.001f, NEAR_HEAD_SIZE - FAR_HEAD_SIZE);
  return clamp01((apparentSize - FAR_HEAD_SIZE) / span);
}

float targetPriority(const HeadbandTarget &t) {
  // If several confirmed headbands are visible, prefer the one that appears
  // closer, with confidence as a smaller tie-breaker.
  const float proximity = estimateProximity(t);
  return 0.75f * proximity + 0.25f * clamp01(t.confidence);
}

int8_t chooseGuidanceTarget() {
  if (targetCount == 0) return -1;

  uint8_t best = 0;
  float bestPriority = targetPriority(targets[0]);
  for (uint8_t i = 1; i < targetCount; ++i) {
    const float priority = targetPriority(targets[i]);
    if (priority > bestPriority) {
      best = i;
      bestPriority = priority;
    }
  }
  return (int8_t)best;
}

void setGuidanceOutputs(bool on) {
  // CENTER/AUX (GP3) intentionally remains off during directional guidance.
  setMotor(1, false);
  setMotor(0, on && (guidanceMotorMask & 0x01));
  setMotor(2, on && (guidanceMotorMask & 0x04));
}

void stopGuidance() {
  guidanceMotorMask = 0;
  guidancePulseOn = false;
  guidancePhaseStartMs = 0;
  guidanceOnMs = 0;
  guidanceOffMs = 0;
  guidanceFrequencyHz = 0.0f;
  guidanceProximity = 0.0f;
  guidanceConfidence = 0.0f;
  setGuidanceOutputs(false);
}

void applyDirectionalGuidance() {
  // Never overwrite the sequential wiring test while it is running.
  if (motorTestActive) return;

  selectedTargetIndex = -1;
  if (!detectionLinkAlive || targetCount == 0) {
    stopGuidance();
    return;
  }

  selectedTargetIndex = chooseGuidanceTarget();
  if (selectedTargetIndex < 0) {
    stopGuidance();
    return;
  }

  const HeadbandTarget &t = targets[(uint8_t)selectedTargetIndex];
  const float cx = t.cx;
  const uint8_t previousMask = guidanceMotorMask;

  if (cx < CENTER_ZONE_MIN) {
    guidanceMotorMask = 0x01; // LEFT
  } else if (cx > CENTER_ZONE_MAX) {
    guidanceMotorMask = 0x04; // RIGHT
  } else {
    guidanceMotorMask = 0x05; // LEFT + RIGHT
  }

  guidanceProximity = estimateProximity(t);
  guidanceConfidence = clamp01(t.confidence);
  const float intensity = clamp01(
    DISTANCE_WEIGHT * guidanceProximity +
    CONFIDENCE_WEIGHT * guidanceConfidence
  );

  guidanceFrequencyHz = MIN_GUIDANCE_HZ +
    (MAX_GUIDANCE_HZ - MIN_GUIDANCE_HZ) * intensity;

  const uint32_t periodMs = (uint32_t)max(1.0f, 1000.0f / guidanceFrequencyHz);
  // Short, distinct pulses. On-time increases modestly for slower patterns,
  // while high-rate patterns still leave a perceptible gap between pulses.
  uint32_t proposedOnMs = (uint32_t)(periodMs * 0.38f);
  guidanceOnMs = constrain(proposedOnMs, MIN_PULSE_ON_MS, MAX_PULSE_ON_MS);
  if (guidanceOnMs >= periodMs) guidanceOnMs = max((uint32_t)1, periodMs / 2);
  guidanceOffMs = max((uint32_t)1, periodMs - guidanceOnMs);

  // Give immediate feedback when the target moves to a different clock side.
  if (previousMask != guidanceMotorMask || guidancePhaseStartMs == 0) {
    guidancePulseOn = true;
    guidancePhaseStartMs = millis();
    setGuidanceOutputs(true);
  }
}

void serviceGuidancePulse() {
  if (motorTestActive) return;
  if (!detectionLinkAlive || guidanceMotorMask == 0 || selectedTargetIndex < 0) {
    stopGuidance();
    return;
  }

  const uint32_t now = millis();
  if (guidancePhaseStartMs == 0) {
    guidancePulseOn = true;
    guidancePhaseStartMs = now;
    setGuidanceOutputs(true);
    return;
  }

  const uint32_t phaseDuration = guidancePulseOn ? guidanceOnMs : guidanceOffMs;
  if (now - guidancePhaseStartMs < phaseDuration) return;

  guidancePulseOn = !guidancePulseOn;
  guidancePhaseStartMs = now;
  setGuidanceOutputs(guidancePulseOn);
}

void startMotorTest() {
  allMotorsOff();
  guidancePulseOn = false;
  guidancePhaseStartMs = 0;
  motorTestActive = true;
  motorTestOnPhase = true;
  motorTestIndex = 0;
  motorTestPhaseStartMs = millis();
  setMotor(motorTestIndex, true);
  Serial.println("Motor wiring test: LEFT ON");
}

void serviceMotorTest() {
  if (!motorTestActive) return;

  const uint32_t now = millis();
  const uint32_t elapsed = now - motorTestPhaseStartMs;

  if (motorTestOnPhase) {
    if (elapsed < MOTOR_TEST_ON_MS) return;

    setMotor(motorTestIndex, false);
    motorTestOnPhase = false;
    motorTestPhaseStartMs = now;
    return;
  }

  if (elapsed < MOTOR_TEST_GAP_MS) return;

  motorTestIndex++;
  if (motorTestIndex >= MOTOR_COUNT) {
    motorTestActive = false;
    motorTestOnPhase = false;
    allMotorsOff();
    Serial.println("Motor wiring test complete");
    applyDirectionalGuidance();
    return;
  }

  motorTestOnPhase = true;
  motorTestPhaseStartMs = now;
  setMotor(motorTestIndex, true);
  if (motorTestIndex == 1) Serial.println("Motor wiring test: CENTER/AUX ON");
  else if (motorTestIndex == 2) Serial.println("Motor wiring test: RIGHT ON");
}

void clearTargets() {
  targetCount = 0;
  detectionLinkAlive = false;
  selectedTargetIndex = -1;

  if (!motorTestActive) stopGuidance();
}

void onTargetsUpdated() {
  Serial.printf("Headbands: %u  seq=%lu\n", targetCount, (unsigned long)lastSequence);
  for (uint8_t i = 0; i < targetCount; ++i) {
    const HeadbandTarget &t = targets[i];
    Serial.printf("  %u: cx=%.3f cy=%.3f conf=%.3f px=(%d,%d)\n",
                  i, t.cx, t.cy, t.confidence, t.cxPx, t.cyPx);
  }

  applyDirectionalGuidance();

  if (selectedTargetIndex >= 0 && !motorTestActive) {
    const HeadbandTarget &t = targets[(uint8_t)selectedTargetIndex];
    const char *zone = t.cx < CENTER_ZONE_MIN ? "LEFT"
                      : t.cx > CENTER_ZONE_MAX ? "RIGHT"
                      : "CENTER -> BOTH";
    Serial.printf("Guidance target %d: cx=%.3f -> %s | proximity=%.2f conf=%.2f rate=%.2f Hz\n",
                  selectedTargetIndex, t.cx, zone, guidanceProximity,
                  guidanceConfidence, guidanceFrequencyHz);
  }
}

// ---------------- HTTP helpers ----------------
void addCorsHeaders() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
  // Helps browsers implementing Private Network Access preflights.
  server.sendHeader("Access-Control-Allow-Private-Network", "true");
  server.sendHeader("Cache-Control", "no-store");
}

void sendJson(int code, const String &json) {
  addCorsHeaders();
  server.send(code, "application/json", json);
}

void handleOptions() {
  addCorsHeaders();
  server.send(204, "text/plain", "");
}

void handlePing() {
  addCorsHeaders();
  server.send(200, "text/plain", "Pico W OK");
}

void handlePair() {
  bool started = false;
  if (!firstPairMotorTestDone) {
    firstPairMotorTestDone = true;
    startMotorTest();
    started = true;
  }

  String json = "{\"ok\":true,\"paired\":true,\"motor_test_started\":";
  json += started ? "true" : "false";
  json += ",\"motor_test_already_done\":";
  json += started ? "false" : "true";
  json += "}";
  sendJson(200, json);
}

void handleMotorTest() {
  startMotorTest();
  sendJson(200, "{\"ok\":true,\"motor_test_started\":true}");
}

void handleStatus() {
  String json;
  json.reserve(320);
  json += "{\"ok\":true,\"targets\":";
  json += targetCount;
  json += ",\"sequence\":";
  json += lastSequence;
  json += ",\"age_ms\":";
  json += lastDetectionPacketMs ? (millis() - lastDetectionPacketMs) : 0;
  json += ",\"link_alive\":";
  json += detectionLinkAlive ? "true" : "false";
  json += ",\"selected_target\":";
  json += (int)selectedTargetIndex;
  json += ",\"motor_test_active\":";
  json += motorTestActive ? "true" : "false";
  json += ",\"guidance\":{\"pulse_on\":";
  json += guidancePulseOn ? "true" : "false";
  json += ",\"frequency_hz\":";
  json += String(guidanceFrequencyHz, 2);
  json += ",\"proximity\":";
  json += String(guidanceProximity, 3);
  json += ",\"confidence\":";
  json += String(guidanceConfidence, 3);
  json += "}";
  json += ",\"motors\":{\"left\":";
  json += motorState[0] ? "true" : "false";
  json += ",\"center_aux\":";
  json += motorState[1] ? "true" : "false";
  json += ",\"right\":";
  json += motorState[2] ? "true" : "false";
  json += "}}";
  sendJson(200, json);
}

void handleDetections() {
  const String body = server.arg("plain");
  if (body.length() == 0) {
    sendJson(400, "{\"ok\":false,\"error\":\"empty body\"}");
    return;
  }

  JsonDocument doc;
  const DeserializationError err = deserializeJson(doc, body);
  if (err) {
    String response = "{\"ok\":false,\"error\":\"invalid JSON: ";
    response += err.c_str();
    response += "\"}";
    sendJson(400, response);
    return;
  }

  if ((doc["version"] | 0) != 1) {
    sendJson(400, "{\"ok\":false,\"error\":\"unsupported payload version\"}");
    return;
  }

  lastSequence = doc["sequence"] | 0;
  JsonArray list = doc["headbands"].as<JsonArray>();
  targetCount = 0;

  for (JsonObject obj : list) {
    if (targetCount >= MAX_HEADBANDS) break;
    HeadbandTarget &t = targets[targetCount++];
    t.confidence = constrain((float)(obj["confidence"] | 0.0f), 0.0f, 1.0f);
    t.cx = constrain((float)(obj["cx"] | 0.0f), 0.0f, 1.0f);
    t.cy = constrain((float)(obj["cy"] | 0.0f), 0.0f, 1.0f);
    t.x = constrain((float)(obj["x"] | 0.0f), 0.0f, 1.0f);
    t.y = constrain((float)(obj["y"] | 0.0f), 0.0f, 1.0f);
    t.w = constrain((float)(obj["w"] | 0.0f), 0.0f, 1.0f);
    t.h = constrain((float)(obj["h"] | 0.0f), 0.0f, 1.0f);
    t.cxPx = obj["cx_px"] | 0;
    t.cyPx = obj["cy_px"] | 0;
  }

  lastDetectionPacketMs = millis();
  detectionLinkAlive = true;
  onTargetsUpdated();

  String response = "{\"ok\":true,\"received\":";
  response += targetCount;
  response += ",\"sequence\":";
  response += lastSequence;
  response += "}";
  sendJson(200, response);
}

void handleNotFound() {
  if (server.method() == HTTP_OPTIONS) {
    handleOptions();
    return;
  }
  sendJson(404, "{\"ok\":false,\"error\":\"not found\"}");
}

void setup() {
  Serial.begin(115200);
  delay(300);

  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    pinMode(MOTOR_PINS[i], OUTPUT);
    digitalWrite(MOTOR_PINS[i], MOTOR_OFF_LEVEL);
  }
  allMotorsOff();

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_GATEWAY, AP_SUBNET);

  if (!WiFi.softAP(AP_SSID, AP_PASSWORD)) {
    Serial.println("Failed to start Pico W access point.");
    while (true) delay(1000);
  }

  Serial.println();
  Serial.println("Headband Pico W receiver ready");
  Serial.printf("Wi-Fi SSID: %s\n", AP_SSID);
  Serial.printf("Wi-Fi password: %s\n", AP_PASSWORD);
  Serial.printf("Pico address: http://%s\n", WiFi.softAPIP().toString().c_str());
  Serial.println("Motor outputs: LEFT=GP2, CENTER/AUX=GP3, RIGHT=GP4");
  Serial.println("Center guidance zone: cx 0.40 to 0.60 -> LEFT + RIGHT");
  Serial.println("Guidance pulse rate: 80% apparent distance + 20% confidence, 1.25 to 6.75 Hz");

  server.on("/ping", HTTP_GET, handlePing);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/pair", HTTP_POST, handlePair);
  server.on("/motor-test", HTTP_POST, handleMotorTest);
  server.on("/detections", HTTP_POST, handleDetections);

  server.on("/detections", HTTP_OPTIONS, handleOptions);
  server.on("/pair", HTTP_OPTIONS, handleOptions);
  server.on("/motor-test", HTTP_OPTIONS, handleOptions);
  server.on("/ping", HTTP_OPTIONS, handleOptions);
  server.onNotFound(handleNotFound);
  server.begin();

  clearTargets();
}

void loop() {
  server.handleClient();
  serviceMotorTest();
  serviceGuidancePulse();

  // Network/motor safety watchdog.
  if (detectionLinkAlive &&
      lastDetectionPacketMs != 0 &&
      millis() - lastDetectionPacketMs > DETECTION_TIMEOUT_MS) {
    Serial.println("Detection link timeout -> clearing targets and stopping guidance motors");
    clearTargets();
  }
}
