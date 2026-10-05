#include <Arduino.h>

// -------- Pins --------
const uint8_t STEP_PINS[3] = {5, 6, 9};  // A,B,C
const uint8_t DIR_PINS [3] = {4, 7, 8};  // A,B,C
const uint8_t DELTA_MOTOR_COUNT = 3;

// MNTL is a brushed DC actuator. Select the H-bridge board you are using.
#define LINEAR_DRIVER_L298N  0
#define LINEAR_DRIVER_BTS7960 1
const uint8_t LINEAR_DRIVER = LINEAR_DRIVER_L298N;

// Shared Arduino allocation:
//   L298N:    D10=ENA,  D11=IN1,  D13=IN2
//   BTS7960:  D10=RPWM, D11=LPWM, D13=R_EN and L_EN (both tied together)
const uint8_t LINEAR_PWM_A_PIN = 10;
const uint8_t LINEAR_PWM_B_PIN = 11;
const uint8_t LINEAR_CONTROL_PIN = 13;
const uint8_t LINEAR_LIMIT_PIN = 12;  // optional external limit, active LOW
const uint8_t LINEAR_PWM = 255;     // full actuator power
const bool USE_EXTERNAL_LINEAR_LIMIT = false;

// Set these signs to match the actuator wiring. With the current wiring,
// positive is close and negative is open.
const float LINEAR_CLOSE_CMD = 1.0f;
const float LINEAR_OPEN_CMD = -1.0f;

// Do not connect the actuator directly to Arduino pins.  Use the selected
// H-bridge and a separate 12 V motor supply.  The MNTL's internal end switches are not
// normally available as a separate Arduino input.
const uint32_t COMMAND_TIMEOUT_US = 250000UL;
const uint32_t LINEAR_MAX_RUN_US = 5000000UL;

// -------- State from serial --------

// cmd[0..2] are *signed off_us* values from the Pi:
//   0   -> motor stopped
//   >0  -> forward, off_us =  cmd[i]
//   <0  -> reverse, off_us = -cmd[i]
// cmd[3] is a signed one-shot direction request for the linear actuator.
float cmd[4] = {0.0f, 0.0f, 0.0f, 0.0f};

const uint16_t on_us = 5;   // HIGH pulse width for all motors

// -------- Pulse schedulers (per motor) --------
bool          step_high[DELTA_MOTOR_COUNT]      = {false, false, false};
unsigned long next_toggle_us[DELTA_MOTOR_COUNT] = {0, 0, 0};
unsigned long last_command_us = 0;
bool          have_command = false;
bool          linear_running = false;
bool          linear_button_armed = true;
float         linear_run_cmd = 0.0f;
unsigned long linear_started_us = 0;

enum LinearState {
  LINEAR_OPENING,
  LINEAR_OPEN,
  LINEAR_CLOSING,
  LINEAR_CLOSED
};
LinearState linear_state = LINEAR_OPENING;

// -------- Serial buffer --------
#define LINE_BUF 96
char linebuf[LINE_BUF];
uint8_t line_len = 0;
bool line_overflow = false;

// ---------------- Utils ----------------
static inline void trimTail(char* s) {
  int n = strlen(s);
  while (n && (s[n-1] == '\r' || s[n-1] == ' ' || s[n-1] == '\t')) s[--n] = 0;
}

// ---------------- Serial parser ----------------
// Format from Pi: "cmdA,cmdB,cmdC,linear\n"
// The first three are signed integer off_us values; linear is a signed
// direction request (-1, 0, or +1).
bool read_serial() {
  bool parsed = false;

  // Consume only bytes already available.  This keeps the motor scheduler
  // running even if a serial line arrives in fragments.
  while (Serial.available()) {
    char ch = (char)Serial.read();

    if (ch != '\n') {
      if (line_len < LINE_BUF - 1) {
        linebuf[line_len++] = ch;
      } else {
        line_overflow = true;
      }
      continue;
    }

    linebuf[line_len] = '\0';
    trimTail(linebuf);

    if (!line_overflow && line_len > 0) {
      long a = 0, b = 0, c = 0, linear = 0;
      int fields = sscanf(linebuf, "%ld,%ld,%ld,%ld", &a, &b, &c, &linear);
      if (fields == 3 || fields == 4) {
        cmd[0] = (float)a;
        cmd[1] = (float)b;
        cmd[2] = (float)c;
        cmd[3] = (fields == 4) ? (float)linear : 0.0f;
        last_command_us = micros();
        have_command = true;
        parsed = true;
      }
    }

    line_len = 0;
    line_overflow = false;
  }

  return parsed;
}

// ---------------- Motor driving ----------------
void stopLinearMotor() {
  analogWrite(LINEAR_PWM_A_PIN, 0);
  analogWrite(LINEAR_PWM_B_PIN, 0);
  digitalWrite(LINEAR_CONTROL_PIN, LOW);
}

void startLinearMove(float direction_cmd, unsigned long now) {
  linear_running = true;
  linear_run_cmd = direction_cmd;
  linear_started_us = now;
  linear_state = (direction_cmd > 0.0f) ? LINEAR_CLOSING : LINEAR_OPENING;

  bool forward = direction_cmd > 0.0f;
  if (LINEAR_DRIVER == LINEAR_DRIVER_L298N) {
    digitalWrite(LINEAR_PWM_B_PIN, forward ? HIGH : LOW);  // IN1
    digitalWrite(LINEAR_CONTROL_PIN, forward ? LOW : HIGH);  // IN2
    analogWrite(LINEAR_PWM_A_PIN, LINEAR_PWM);  // ENA
  } else {
    digitalWrite(LINEAR_CONTROL_PIN, HIGH);  // R_EN and L_EN
    analogWrite(LINEAR_PWM_A_PIN, forward ? LINEAR_PWM : 0);  // RPWM
    analogWrite(LINEAR_PWM_B_PIN, forward ? 0 : LINEAR_PWM);  // LPWM
  }
}

void finishLinearMove() {
  bool was_closing = linear_state == LINEAR_CLOSING;
  linear_running = false;
  linear_run_cmd = 0.0f;
  linear_state = was_closing ? LINEAR_CLOSED : LINEAR_OPEN;
  stopLinearMotor();
}

void updateLinearMove(unsigned long now, bool command_fresh) {
  bool button_pressed = command_fresh && fabs(cmd[3]) >= 1.0f;
  bool limit_hit = USE_EXTERNAL_LINEAR_LIMIT &&
      (digitalRead(LINEAR_LIMIT_PIN) == LOW);

  // A lost serial link must stop an in-progress linear move immediately.
  if (have_command && !command_fresh && linear_running) {
    linear_running = false;
    linear_run_cmd = 0.0f;
    stopLinearMotor();
  }

  // A button release rearms the toggle. Holding the button after a move
  // completes cannot immediately start the next move.
  if (!button_pressed) {
    linear_button_armed = true;
  }

  if (!linear_running && button_pressed && linear_button_armed && !limit_hit) {
    linear_button_armed = false;
    if (linear_state == LINEAR_OPEN) {
      startLinearMove(LINEAR_CLOSE_CMD, now);
    } else if (linear_state == LINEAR_CLOSED) {
      startLinearMove(LINEAR_OPEN_CMD, now);
    }
  }

  if (linear_running &&
      (limit_hit || (unsigned long)(now - linear_started_us) >= LINEAR_MAX_RUN_US)) {
    finishLinearMove();
  }
}

void driveMotors() {
  unsigned long now = micros();

  // If the Pi/USB link stops sending commands, stop every motor.  The Pi
  // normally refreshes the command at 50 Hz, so 250 ms leaves ample margin.
  bool command_fresh = have_command &&
      (unsigned long)(now - last_command_us) <= COMMAND_TIMEOUT_US;

  updateLinearMove(now, command_fresh);

  for (int i = 0; i < DELTA_MOTOR_COUNT; ++i) {
    float c = command_fresh ? cmd[i] : 0.0f;

    // If nearly zero -> motor stopped
    if (fabs(c) < 1.0f) {  // threshold so tiny noise doesn't move the motor
      // ensure step is LOW and scheduler is idle-ish
      if (step_high[i]) {
        digitalWrite(STEP_PINS[i], LOW);
        step_high[i] = false;
      }
      next_toggle_us[i] = now;
      continue;
    }

    // Set direction based on sign of cmd
    if (c > 0.0f) {
      digitalWrite(DIR_PINS[i], HIGH);  // one direction
    } else {
      digitalWrite(DIR_PINS[i], LOW);   // opposite direction
    }

    unsigned long off_us = (unsigned long)fabs(c);  // off-time from Pi

    // Toggle this motor according to its own schedule
    if ((long)(now - next_toggle_us[i]) >= 0) {
      if (step_high[i]) {
        // End pulse: go LOW and wait off_us
        digitalWrite(STEP_PINS[i], LOW);
        step_high[i]      = false;
        next_toggle_us[i] = now + off_us;
      } else {
        // Start pulse: go HIGH and wait on_us
        digitalWrite(STEP_PINS[i], HIGH);
        step_high[i]      = true;
        next_toggle_us[i] = now + on_us;
      }
    }
  }
}

// ---------------- Arduino boilerplate ----------------
void setup() {
  Serial.begin(115200);
  Serial.println(F("Ready. Format: cmdA,cmdB,cmdC,linear  (linear to limit or 5s)"));

  if (USE_EXTERNAL_LINEAR_LIMIT) {
    // Optional external switch: wire between D12 and GND.  It is active LOW.
    pinMode(LINEAR_LIMIT_PIN, INPUT_PULLUP);
  }

  pinMode(LINEAR_PWM_A_PIN, OUTPUT);
  pinMode(LINEAR_PWM_B_PIN, OUTPUT);
  pinMode(LINEAR_CONTROL_PIN, OUTPUT);
  stopLinearMotor();

  // Establish a known starting state by opening the gripper on startup.
  startLinearMove(LINEAR_OPEN_CMD, micros());

  for (int i = 0; i < DELTA_MOTOR_COUNT; i++) {
    pinMode(STEP_PINS[i], OUTPUT);
    pinMode(DIR_PINS[i],  OUTPUT);
    digitalWrite(STEP_PINS[i], LOW);
    step_high[i]      = false;
    next_toggle_us[i] = micros();
  }
}

void loop() {
  read_serial();   // update commands when new line arrives
  driveMotors();   // run delta steppers and update the linear actuator
}
