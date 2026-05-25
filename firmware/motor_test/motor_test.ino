// Simple motor tester for the autonomous car drive system.
//
// Targets an Arduino Mega with up to 4 BTS7960 H-bridges (one per wheel).
// Steering is NOT included here — use the steering calibration sketch
// instead, since it needs the angle sensor and careful tuning.
//
// Each bridge has 4 pins: R_EN, L_EN, R_PWM, L_PWM. R_EN+L_EN must both
// be HIGH for the bridge outputs to be active; driving them LOW puts the
// outputs in high-Z (true hardware disable / e-stop).
//
// --- Serial commands (115200 baud, newline-terminated) ---
//   fl | fr | rl | rr      select a motor
//   all                    select all motors
//   e                      enable (EN pins HIGH) for selected
//   d                      disable (EN pins LOW) for selected
//   f                      spin selected forward at current speed (auto-enables)
//   b                      spin selected backward at current speed (auto-enables)
//   s                      stop selected (PWM=0, EN stays HIGH so motor brakes)
//   x                      EMERGENCY STOP: PWM=0 + EN=LOW on ALL motors
//   + / -                  change speed by 15 PWM units (0-255)
//   <number>               set speed directly, e.g. "120"
//   ?                      print status + help
//
// Unconnected motors just sit silent — safe to test only the ones
// you have wired up.

#include <Arduino.h>

struct Motor {
  const char* name;
  uint8_t r_en;
  uint8_t l_en;
  uint8_t rpwm;
  uint8_t lpwm;
};

constexpr uint8_t MOTOR_COUNT = 4;
const Motor motors[MOTOR_COUNT] = {
  // name, R_EN, L_EN, R_PWM, L_PWM    bridge
  {"FL",    24,   25,    5,     6},  // bridge 2 (top-left)
  {"FR",    26,   27,    7,     8},  // bridge 3
  {"RL",    22,   23,    3,     4},  // bridge 1 (bottom-left)
  {"RR",    28,   29,    9,    10},  // bridge 4 (bottom-right)
};

// -1 = all motors selected; otherwise an index into motors[].
int8_t selected = 0;
uint8_t speed = 80;  // safe default — enough to overcome stiction on most gearboxes

String buffer;

void set_enable(uint8_t idx, bool on) {
  digitalWrite(motors[idx].r_en, on ? HIGH : LOW);
  digitalWrite(motors[idx].l_en, on ? HIGH : LOW);
}

void drive_motor(uint8_t idx, int direction) {
  // direction: +1 forward, -1 backward, 0 stop
  if (direction == 0) {
    analogWrite(motors[idx].rpwm, 0);
    analogWrite(motors[idx].lpwm, 0);
    return;
  }
  set_enable(idx, true);  // auto-enable on motion
  if (direction > 0) {
    analogWrite(motors[idx].rpwm, speed);
    analogWrite(motors[idx].lpwm, 0);
  } else {
    analogWrite(motors[idx].rpwm, 0);
    analogWrite(motors[idx].lpwm, speed);
  }
}

void for_selected(void (*fn)(uint8_t)) {
  if (selected < 0) {
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i) fn(i);
  } else {
    fn((uint8_t)selected);
  }
}

void drive_fwd(uint8_t i)  { drive_motor(i, +1); }
void drive_back(uint8_t i) { drive_motor(i, -1); }
void drive_stop(uint8_t i) { drive_motor(i, 0); }
void enable_on(uint8_t i)  { set_enable(i, true); }
void enable_off(uint8_t i) { set_enable(i, false); }

void hard_stop_all() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    analogWrite(motors[i].rpwm, 0);
    analogWrite(motors[i].lpwm, 0);
    set_enable(i, false);
  }
}

void print_status() {
  Serial.print(F("[selected="));
  if (selected < 0) Serial.print(F("ALL"));
  else Serial.print(motors[selected].name);
  Serial.print(F(" speed="));
  Serial.print(speed);
  Serial.println(F("]"));
}

void print_help() {
  Serial.println(F("--- Motor tester ---"));
  Serial.println(F("Select : fl | fr | rl | rr | all"));
  Serial.println(F("Enable : e (on)  d (off)"));
  Serial.println(F("Drive  : f (fwd)  b (back)  s (stop)  x (estop all)"));
  Serial.println(F("Speed  : + / -  or type a number 0-255"));
  Serial.println(F("Status : ?"));
}

void handle_command(String cmd) {
  cmd.trim();
  cmd.toLowerCase();
  if (cmd.length() == 0) return;

  if (cmd == "fl" || cmd == "fr" || cmd == "rl" || cmd == "rr") {
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
      String n = motors[i].name;
      n.toLowerCase();
      if (n == cmd) {
        hard_stop_all();
        selected = i;
        print_status();
        return;
      }
    }
  } else if (cmd == "all") {
    hard_stop_all();
    selected = -1;
    print_status();
  } else if (cmd == "e") {
    for_selected(enable_on);
    Serial.println(F("-> enable HIGH"));
  } else if (cmd == "d") {
    for_selected(enable_off);
    Serial.println(F("-> enable LOW"));
  } else if (cmd == "f") {
    for_selected(drive_fwd);
    Serial.println(F("-> forward"));
  } else if (cmd == "b") {
    for_selected(drive_back);
    Serial.println(F("-> backward"));
  } else if (cmd == "s") {
    for_selected(drive_stop);
    Serial.println(F("-> stop (PWM=0, EN unchanged)"));
  } else if (cmd == "x") {
    hard_stop_all();
    Serial.println(F("-> EMERGENCY STOP (all PWM=0, all EN=LOW)"));
  } else if (cmd == "+") {
    speed = (speed > 240) ? 255 : speed + 15;
    print_status();
  } else if (cmd == "-") {
    speed = (speed < 15) ? 0 : speed - 15;
    print_status();
  } else if (cmd == "?") {
    print_status();
    print_help();
  } else {
    bool numeric = true;
    for (uint16_t i = 0; i < cmd.length(); ++i) {
      if (!isDigit(cmd[i])) { numeric = false; break; }
    }
    if (numeric) {
      int v = cmd.toInt();
      if (v < 0) v = 0;
      if (v > 255) v = 255;
      speed = (uint8_t)v;
      print_status();
    } else {
      Serial.print(F("Unknown command: "));
      Serial.println(cmd);
    }
  }
}

void setup() {
  Serial.begin(115200);
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    pinMode(motors[i].r_en, OUTPUT);
    pinMode(motors[i].l_en, OUTPUT);
    pinMode(motors[i].rpwm, OUTPUT);
    pinMode(motors[i].lpwm, OUTPUT);
  }
  hard_stop_all();  // start fully disabled

  delay(200);
  Serial.println();
  Serial.println(F("Motor tester ready (all motors disabled)."));
  print_help();
  print_status();
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      handle_command(buffer);
      buffer = "";
    } else {
      buffer += c;
      if (buffer.length() > 32) buffer = "";  // overflow guard
    }
  }
}
