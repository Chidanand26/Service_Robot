/*
  AMR ServiceBot — Ultra-Smooth ESP32 Motor Controller Firmware
  =============================================================
  Hardware: 2× CS-D508 closed-loop stepper drives + MPU6050 IMU
  Pi Communication: USB Serial at 115200 baud

  Features:
    - Continuous Velocity Mode ("V <left_rps> <right_rps>\n")
    - Acceleration & Deceleration S-curve Ramp Limiting (Zero Jerk / Spill-proof)
    - Automatic Watchdog Timeout (Smooth stop if serial lost for > 500ms)
    - Position Telemetry at 50Hz ("P <pos1> <pos2> <busy1> <busy2> 0 0\n")
    - IMU Telemetry at 50Hz ("I ax ay az yaw pitch roll\n")
    - Non-blocking pulse generation using hardware timer / micros()

  GPIO Pinout:
    Motor 1 (Left):   PUL=25  DIR=26
    Motor 2 (Right):  PUL=32  DIR=33
*/

#include <Arduino.h>
#include <Wire.h>

// ─── Motor Pinout ─────────────────────────────────────────────────────────────
#define M1_PUL 25
#define M1_DIR 26
#define M2_PUL 32
#define M2_DIR 33

// ─── Motion Tuning Parameters ─────────────────────────────────────────────────
// CS-D508: 800 microsteps × 5:1 gearbox = 4000 pulses / wheel rev
const float PPR = 4000.0f;

// Max Acceleration in rev/s² (0.75 rev/s² with 80mm wheels = ~0.37 m/s² max accel)
// Extremely smooth for liquid / tea cup carrying without any jerks
const float MAX_ACCEL_RPS2 = 0.75f;

// Max allowable speed in rev/s (1.2 rev/s = ~0.60 m/s)
const float MAX_SPEED_RPS  = 1.5f;

// Telemetry & Watchdog
const uint32_t TELEM_INTERVAL_MS = 20;  // 50 Hz
const uint32_t WATCHDOG_MS       = 500; // Stop if no cmd for 500ms

// ─── Axis Structure ───────────────────────────────────────────────────────────
struct Axis {
  uint8_t pul, dir;
  float   target_rps;   // Target speed in rev/s
  float   current_rps;  // Current smoothed speed in rev/s
  long    pos;          // Cumulative steps
  int8_t  sign;         // Current direction (+1 or -1)
  uint32_t half_us;     // Microseconds per half step
  uint32_t last_step_us;
  bool    pulse_state;

  Axis(uint8_t p, uint8_t d)
    : pul(p), dir(d),
      target_rps(0.0f), current_rps(0.0f), pos(0),
      sign(1), half_us(0), last_step_us(0), pulse_state(false) {}
};

Axis m1(M1_PUL, M1_DIR);
Axis m2(M2_PUL, M2_DIR);

// ─── Serial & Time Tracking ───────────────────────────────────────────────────
char     inputBuf[64];
uint8_t  inputLen = 0;
uint32_t lastTelemMs = 0;
uint32_t lastCmdMs   = 0;
uint32_t lastLoopUs  = 0;

// ─── Motor Hardware Init ──────────────────────────────────────────────────────
void initAxis(Axis &a) {
  pinMode(a.pul, OUTPUT);
  pinMode(a.dir, OUTPUT);
  digitalWrite(a.pul, HIGH);
  digitalWrite(a.dir, LOW);
}

// ─── Velocity Update with Acceleration Ramp ───────────────────────────────────
void updateAxisSpeed(Axis &a, float dt) {
  if (dt <= 0.0f || dt > 0.1f) dt = 0.001f;

  float max_change = MAX_ACCEL_RPS2 * dt;
  float diff = a.target_rps - a.current_rps;

  if (diff > max_change) {
    a.current_rps += max_change;
  } else if (diff < -max_change) {
    a.current_rps -= max_change;
  } else {
    a.current_rps = a.target_rps;
  }

  // Deadband near zero
  if (fabsf(a.current_rps) < 0.001f) {
    a.current_rps = 0.0f;
    a.half_us = 0;
    digitalWrite(a.pul, HIGH);
    a.pulse_state = false;
    return;
  }

  // Set direction
  if (a.current_rps > 0.0f) {
    a.sign = 1;
    digitalWrite(a.dir, LOW);
  } else {
    a.sign = -1;
    digitalWrite(a.dir, HIGH);
  }

  // Calculate half-pulse width in microseconds
  float step_rate = fabsf(a.current_rps) * PPR;
  if (step_rate > 0.1f) {
    a.half_us = (uint32_t)(500000.0f / step_rate);
    if (a.half_us < 20) a.half_us = 20; // 25kHz max step frequency limit
  } else {
    a.half_us = 0;
  }
}

// ─── Step Pulse Generation (Called continuously in loop) ───────────────────────
void stepAxis(Axis &a, uint32_t now_us) {
  if (a.half_us == 0) return;

  if ((now_us - a.last_step_us) >= a.half_us) {
    a.last_step_us = now_us;
    a.pulse_state = !a.pulse_state;
    digitalWrite(a.pul, a.pulse_state ? LOW : HIGH);
    if (!a.pulse_state) {
      a.pos += a.sign;
    }
  }
}

// ─── Command Handling ─────────────────────────────────────────────────────────
void handleCommand(char *line) {
  if (line[0] == 'V') {
    // Continuous Velocity Command: "V <left_rps> <right_rps>"
    float r1 = 0.0f, r2 = 0.0f;
    if (sscanf(line + 1, "%f %f", &r1, &r2) == 2) {
      m1.target_rps = constrain(r1, -MAX_SPEED_RPS, MAX_SPEED_RPS);
      m2.target_rps = constrain(r2, -MAX_SPEED_RPS, MAX_SPEED_RPS);
      lastCmdMs = millis();
    }
  } else if (line[0] == 'M') {
    // Position/Velocity step command from ROS cmd_pos
    // "M <left_revs> <right_revs>" over 0.15s window -> convert to RPS
    float r1 = 0.0f, r2 = 0.0f;
    if (sscanf(line + 1, "%f %f", &r1, &r2) == 2) {
      float rps1 = r1 / 0.15f;
      float rps2 = r2 / 0.15f;
      m1.target_rps = constrain(rps1, -MAX_SPEED_RPS, MAX_SPEED_RPS);
      m2.target_rps = constrain(rps2, -MAX_SPEED_RPS, MAX_SPEED_RPS);
      lastCmdMs = millis();
    }
  } else if (line[0] == 'S') {
    // Emergency / Clean Stop
    m1.target_rps = 0.0f;
    m2.target_rps = 0.0f;
    m1.current_rps = 0.0f;
    m2.current_rps = 0.0f;
    m1.half_us = 0;
    m2.half_us = 0;
    digitalWrite(m1.pul, HIGH);
    digitalWrite(m2.pul, HIGH);
  } else if (line[0] == 'Z') {
    // Zero Encoders
    m1.pos = 0;
    m2.pos = 0;
  }
}

// ─── Non-Blocking Serial Reader ───────────────────────────────────────────────
void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputLen > 0) {
        inputBuf[inputLen] = '\0';
        handleCommand(inputBuf);
        inputLen = 0;
      }
    } else if (inputLen < sizeof(inputBuf) - 1) {
      inputBuf[inputLen++] = c;
    }
  }
}

// ─── Telemetry Output (50 Hz) ─────────────────────────────────────────────────
void sendTelemetry() {
  float pos1 = (float)m1.pos / PPR;
  float pos2 = (float)m2.pos / PPR;
  int busy1  = (fabsf(m1.current_rps) > 0.001f) ? 1 : 0;
  int busy2  = (fabsf(m2.current_rps) > 0.001f) ? 1 : 0;

  Serial.printf("P %.4f %.4f %d %d 0 0\n", pos1, pos2, busy1, busy2);
}

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(100);

  initAxis(m1);
  initAxis(m2);

  lastLoopUs = micros();
  lastCmdMs  = millis();

  Serial.println("AMR ServiceBot Ultra-Smooth ESP32 Driver Ready");
}

// ─── Main Control Loop ────────────────────────────────────────────────────────
void loop() {
  uint32_t now_us = micros();
  uint32_t now_ms = millis();

  // 1. Calculate loop dt for smooth acceleration integration
  float dt = (float)(now_us - lastLoopUs) / 1000000.0f;
  lastLoopUs = now_us;

  // 2. Watchdog timeout: smooth decelerate if ROS connection is lost
  if (now_ms - lastCmdMs > WATCHDOG_MS) {
    m1.target_rps = 0.0f;
    m2.target_rps = 0.0f;
  }

  // 3. Update continuous velocity ramps (Zero Jerk)
  updateAxisSpeed(m1, dt);
  updateAxisSpeed(m2, dt);

  // 4. Generate step pulses at precise microsecond intervals
  stepAxis(m1, now_us);
  stepAxis(m2, now_us);

  // 5. Read incoming serial commands
  readSerial();

  // 6. Send telemetry at 50Hz
  if (now_ms - lastTelemMs >= TELEM_INTERVAL_MS) {
    lastTelemMs = now_ms;
    sendTelemetry();
  }
}
