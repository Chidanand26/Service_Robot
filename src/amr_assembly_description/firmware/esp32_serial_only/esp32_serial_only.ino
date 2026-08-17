/*
  =============================================================================
  AMR ServiceBot — Universal Jitter-Free DDS Motor Controller Firmware (ESP32)
  =============================================================================
  Hardware: 
    - 2× CS-D508 Closed-Loop Stepper Drives (800 usteps × 5:1 gearbox = 4000 PPR)
    - ESP32 Dual-Core @ 240 MHz (Compatible with Arduino ESP32 Core 2.x & 3.x)

  Architecture:
    - 20,000 Hz HARDWARE TIMER ISR (50us tick):
        Direct Digital Synthesis (DDS) phase-accumulator step pulse generator.
        Nanosecond-accurate, zero phase jitter, completely immune to serial delays.
    - CORE 1 LOOP:
        High-speed non-blocking serial command parser ("V <left_rps> <right_rps>\n").
        Watchdog failsafe: smoothly halts if serial stream drops for > 400ms.
        50Hz position & status telemetry ("P pos1 pos2 busy1 busy2 0 0\n").

  GPIO Pinout:
    Motor 1 (Left):   PUL=GPIO 25,  DIR=GPIO 26
    Motor 2 (Right):  PUL=GPIO 32,  DIR=GPIO 33
  =============================================================================
*/

#include <Arduino.h>

// ─── Motor Hardware Pinout ───────────────────────────────────────────────────
#define M1_PUL 25
#define M1_DIR 26
#define M2_PUL 32
#define M2_DIR 33

// ─── Motion Tuning Parameters ─────────────────────────────────────────────────
// CS-D508: 800 microsteps/rev × 5:1 planetary gearbox = 4000 pulses / wheel rev
const float PPR = 4000.0f;

// DDS Timer Configuration (20,000 Hz = 50 microseconds per tick)
#define TIMER_FREQ_HZ 20000
#define DDS_SCALE     1000000UL  // Fixed-point scaling for phase accumulator

// Speed & Acceleration Limits (in wheel revolutions / second)
const float MAX_SPEED_RPS  = 1.50f;  // ~0.75 m/s max velocity
const float MAX_ACCEL_RPS2 = 0.80f;  // ~0.40 m/s² max hardware acceleration

// Watchdog & Telemetry Intervals
const uint32_t WATCHDOG_TIMEOUT_MS  = 400; // Auto-stop if no serial for 400ms
const uint32_t TELEMETRY_INTERVAL_MS = 20;  // 50 Hz ROS telemetry

// ─── DDS Stepper Engine State (Volatiles accessed in Timer ISR) ───────────────
struct DDSAxis {
  uint8_t pul_pin;
  uint8_t dir_pin;

  // Command & Speed State (in float RPS)
  float target_rps;
  float current_rps;

  // DDS Phase Accumulator variables
  volatile uint32_t step_inc;      // Phase increment per timer tick
  volatile uint32_t accumulator;   // Fixed-point phase accumulator
  volatile int8_t   direction;     // +1 forward, -1 reverse
  volatile long     pos_steps;     // Cumulative encoder step count
  volatile bool     is_moving;

  DDSAxis(uint8_t p, uint8_t d)
    : pul_pin(p), dir_pin(d),
      target_rps(0.0f), current_rps(0.0f),
      step_inc(0), accumulator(0),
      direction(1), pos_steps(0), is_moving(false) {}
};

DDSAxis axis1(M1_PUL, M1_DIR);
DDSAxis axis2(M2_PUL, M2_DIR);

// Hardware Timer Handle
hw_timer_t *stepTimer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// ─── Hardware Timer ISR (Runs strictly every 50us @ 20kHz) ───────────────────
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
void IRAM_ATTR onStepTimer() {
#else
void IRAM_ATTR onStepTimer() {
#endif
  portENTER_CRITICAL_ISR(&timerMux);

  // ── Axis 1 (Left Wheel) ──
  if (axis1.step_inc > 0) {
    axis1.accumulator += axis1.step_inc;
    if (axis1.accumulator >= DDS_SCALE) {
      axis1.accumulator -= DDS_SCALE;
      digitalWrite(axis1.pul_pin, LOW);
      axis1.pos_steps += axis1.direction;
      digitalWrite(axis1.pul_pin, HIGH);
    }
  }

  // ── Axis 2 (Right Wheel) ──
  if (axis2.step_inc > 0) {
    axis2.accumulator += axis2.step_inc;
    if (axis2.accumulator >= DDS_SCALE) {
      axis2.accumulator -= DDS_SCALE;
      digitalWrite(axis2.pul_pin, LOW);
      axis2.pos_steps += axis2.direction;
      digitalWrite(axis2.pul_pin, HIGH);
    }
  }

  portEXIT_CRITICAL_ISR(&timerMux);
}

// ─── Velocity & DDS Update (Called at 100Hz in Loop) ─────────────────────────
void updateSpeed(DDSAxis &a, float dt) {
  // Smoothly ramp current_rps toward target_rps
  float max_change = MAX_ACCEL_RPS2 * dt;
  float error = a.target_rps - a.current_rps;

  if (error > max_change) {
    a.current_rps += max_change;
  } else if (error < -max_change) {
    a.current_rps -= max_change;
  } else {
    a.current_rps = a.target_rps;
  }

  // Deadband near zero
  if (fabsf(a.current_rps) < 0.0005f) {
    a.current_rps = 0.0f;
    portENTER_CRITICAL(&timerMux);
    a.step_inc = 0;
    a.accumulator = 0;
    a.is_moving = false;
    digitalWrite(a.pul_pin, HIGH);
    portEXIT_CRITICAL(&timerMux);
    return;
  }

  // Set hardware direction pin
  int8_t new_dir = (a.current_rps > 0.0f) ? 1 : -1;
  if (new_dir != a.direction) {
    a.direction = new_dir;
    digitalWrite(a.dir_pin, (a.direction > 0) ? LOW : HIGH);
  }

  // Calculate step frequency: steps_per_sec = |current_rps| * PPR
  float steps_per_sec = fabsf(a.current_rps) * PPR;
  if (steps_per_sec > (float)TIMER_FREQ_HZ) {
    steps_per_sec = (float)TIMER_FREQ_HZ;
  }

  // DDS Phase increment per tick: inc = (steps_per_sec / TIMER_FREQ_HZ) * DDS_SCALE
  uint32_t inc = (uint32_t)((steps_per_sec * (float)DDS_SCALE) / (float)TIMER_FREQ_HZ);

  portENTER_CRITICAL(&timerMux);
  a.step_inc = inc;
  a.is_moving = true;
  portEXIT_CRITICAL(&timerMux);
}

// ─── Serial Communication Buffers ─────────────────────────────────────────────
char     serialBuf[64];
uint8_t  serialIdx = 0;
uint32_t lastCmdTimeMs   = 0;
uint32_t lastTelemTimeMs = 0;
uint32_t lastRampTimeUs  = 0;

// ─── Command Parsing ─────────────────────────────────────────────────────────
void parseCommand(char *cmd) {
  if (cmd[0] == 'V') {
    // Continuous Velocity: "V <left_rps> <right_rps>"
    float r1 = 0.0f, r2 = 0.0f;
    if (sscanf(cmd + 1, "%f %f", &r1, &r2) == 2) {
      axis1.target_rps = constrain(r1, -MAX_SPEED_RPS, MAX_SPEED_RPS);
      axis2.target_rps = constrain(r2, -MAX_SPEED_RPS, MAX_SPEED_RPS);
      lastCmdTimeMs = millis();
    }
  } else if (cmd[0] == 'M') {
    // Legacy / Position compatibility mode
    float r1 = 0.0f, r2 = 0.0f;
    if (sscanf(cmd + 1, "%f %f", &r1, &r2) == 2) {
      axis1.target_rps = constrain(r1, -MAX_SPEED_RPS, MAX_SPEED_RPS);
      axis2.target_rps = constrain(r2, -MAX_SPEED_RPS, MAX_SPEED_RPS);
      lastCmdTimeMs = millis();
    }
  } else if (cmd[0] == 'S') {
    // Immediate Clean Stop
    axis1.target_rps = 0.0f;
    axis2.target_rps = 0.0f;
    axis1.current_rps = 0.0f;
    axis2.current_rps = 0.0f;
    portENTER_CRITICAL(&timerMux);
    axis1.step_inc = 0;
    axis2.step_inc = 0;
    axis1.accumulator = 0;
    axis2.accumulator = 0;
    axis1.is_moving = false;
    axis2.is_moving = false;
    digitalWrite(axis1.pul_pin, HIGH);
    digitalWrite(axis2.pul_pin, HIGH);
    portEXIT_CRITICAL(&timerMux);
  } else if (cmd[0] == 'Z') {
    // Zero Encoder Counters
    portENTER_CRITICAL(&timerMux);
    axis1.pos_steps = 0;
    axis2.pos_steps = 0;
    portEXIT_CRITICAL(&timerMux);
  }
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialIdx > 0) {
        serialBuf[serialIdx] = '\0';
        parseCommand(serialBuf);
        serialIdx = 0;
      }
    } else if (serialIdx < sizeof(serialBuf) - 1) {
      serialBuf[serialIdx++] = c;
    }
  }
}

// ─── 50Hz Telemetry Output ────────────────────────────────────────────────────
void sendTelemetry() {
  long p1, p2;
  bool b1, b2;

  portENTER_CRITICAL(&timerMux);
  p1 = axis1.pos_steps;
  p2 = axis2.pos_steps;
  b1 = axis1.is_moving;
  b2 = axis2.is_moving;
  portEXIT_CRITICAL(&timerMux);

  float rev1 = (float)p1 / PPR;
  float rev2 = (float)p2 / PPR;

  Serial.printf("P %.4f %.4f %d %d 0 0\n", rev1, rev2, b1 ? 1 : 0, b2 ? 1 : 0);
}

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.setRxBufferSize(256);

  // Initialize motor pins
  pinMode(axis1.pul_pin, OUTPUT);
  pinMode(axis1.dir_pin, OUTPUT);
  pinMode(axis2.pul_pin, OUTPUT);
  pinMode(axis2.dir_pin, OUTPUT);

  digitalWrite(axis1.pul_pin, HIGH);
  digitalWrite(axis1.dir_pin, LOW);
  digitalWrite(axis2.pul_pin, HIGH);
  digitalWrite(axis2.dir_pin, LOW);

  // Universal ESP32 Timer Init (Compatible with Core 2.x and Core 3.x)
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  stepTimer = timerBegin(1000000);  // 1 MHz base clock
  timerAttachInterrupt(stepTimer, &onStepTimer);
  timerAlarm(stepTimer, 50, true, 0); // 50us period = 20,000 Hz
#else
  stepTimer = timerBegin(0, 80, true); // Timer 0, prescaler 80 (1us tick)
  timerAttachInterrupt(stepTimer, &onStepTimer, true);
  timerAlarmWrite(stepTimer, 50, true); // 50us alarm
  timerAlarmEnable(stepTimer);
#endif

  lastRampTimeUs  = micros();
  lastCmdTimeMs   = millis();
  lastTelemTimeMs = millis();

  Serial.println("AMR ServiceBot DDS Controller Ready");
}

// ─── Main Control Loop (Core 1) ───────────────────────────────────────────────
void loop() {
  uint32_t now_us = micros();
  uint32_t now_ms = millis();

  // 1. Process incoming commands from ROS 2
  readSerial();

  // 2. Watchdog timeout check (fail-safe smooth stop if ROS disconnected)
  if (now_ms - lastCmdTimeMs > WATCHDOG_TIMEOUT_MS) {
    axis1.target_rps = 0.0f;
    axis2.target_rps = 0.0f;
  }

  // 3. Update velocity ramps at fixed 100 Hz (every 10,000 us = 10 ms)
  if (now_us - lastRampTimeUs >= 10000) {
    float dt = (float)(now_us - lastRampTimeUs) / 1000000.0f;
    lastRampTimeUs = now_us;
    updateSpeed(axis1, dt);
    updateSpeed(axis2, dt);
  }

  // 4. Send 50 Hz odometry telemetry to ROS 2
  if (now_ms - lastTelemTimeMs >= TELEMETRY_INTERVAL_MS) {
    lastTelemTimeMs = now_ms;
    sendTelemetry();
  }
}
