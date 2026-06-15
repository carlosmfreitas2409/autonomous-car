// Steering calibration using a 10k linear potentiometer (the red/yellow one)
// instead of an AS5600 magnetic encoder.
//
// Wiring:
//   pot outer pin 1  -> 5V
//   pot outer pin 2  -> GND
//   pot wiper (center) -> PIN_STEER_POT (analog input)
// Mechanically couple the pot shaft to the steering linkage so it sweeps a
// good chunk of its travel between full-left and full-right. Avoid running it
// into its end stops.
//
// Commands (Serial Monitor, 115200, newline ending):
//   r <ms> [pwm]   pulse right
//   l <ms> [pwm]   pulse left
//   s              stop now
//   c              capture current position as center
//   ml             capture current position as max-left
//   mr             capture current position as max-right
//   p              print captured values as config.h snippet

#include <Arduino.h>

constexpr uint8_t PIN_STEER_R_EN = 30;
constexpr uint8_t PIN_STEER_L_EN = 31;
constexpr uint8_t PIN_STEER_RPWM = 11;
constexpr uint8_t PIN_STEER_LPWM = 12;

// Wiper of the 10k pot. A0 on the Mega; change if you wire it elsewhere.
constexpr uint8_t PIN_STEER_POT = A11;

constexpr uint16_t MAX_PULSE_MS = 500;
constexpr uint8_t MAX_PWM = 140;
constexpr uint8_t DEFAULT_PWM = 90;
constexpr uint16_t ANGLE_PRINT_PERIOD_MS = 500;

// Median-of-N sampling to knock down ADC jitter on the analog read.
constexpr uint8_t POT_SAMPLES = 9;

bool pulse_active = false;
uint32_t pulse_until_micros = 0;

int32_t captured_center = -1;
int32_t captured_left = -1;
int32_t captured_right = -1;

uint32_t last_angle_print_ms = 0;

uint16_t read_angle() {
  uint16_t samples[POT_SAMPLES];
  for (uint8_t i = 0; i < POT_SAMPLES; ++i) {
    samples[i] = (uint16_t)analogRead(PIN_STEER_POT);
  }
  // Insertion sort, then take the middle element.
  for (uint8_t i = 1; i < POT_SAMPLES; ++i) {
    uint16_t key = samples[i];
    int8_t j = (int8_t)i - 1;
    while (j >= 0 && samples[j] > key) {
      samples[j + 1] = samples[j];
      --j;
    }
    samples[j + 1] = key;
  }
  return samples[POT_SAMPLES / 2];
}

void drive_pwm(int16_t signed_pwm) {
  if (signed_pwm > MAX_PWM) signed_pwm = MAX_PWM;
  if (signed_pwm < -(int16_t)MAX_PWM) signed_pwm = -(int16_t)MAX_PWM;

  if (signed_pwm > 0) {
    analogWrite(PIN_STEER_RPWM, (uint8_t)signed_pwm);
    analogWrite(PIN_STEER_LPWM, 0);
  } else if (signed_pwm < 0) {
    analogWrite(PIN_STEER_RPWM, 0);
    analogWrite(PIN_STEER_LPWM, (uint8_t)(-signed_pwm));
  } else {
    analogWrite(PIN_STEER_RPWM, 0);
    analogWrite(PIN_STEER_LPWM, 0);
  }
}

void stop_pulse() {
  drive_pwm(0);
  pulse_active = false;
}

void start_pulse(int8_t dir, uint32_t ms, uint8_t pwm) {
  if (ms == 0 || ms > MAX_PULSE_MS) {
    Serial.print(F("rejected: ms must be 1.."));
    Serial.println(MAX_PULSE_MS);
    return;
  }
  if (pwm == 0) pwm = DEFAULT_PWM;
  if (pwm > MAX_PWM) pwm = MAX_PWM;

  int16_t signed_pwm = (dir > 0) ? (int16_t)pwm : -(int16_t)pwm;
  drive_pwm(signed_pwm);
  pulse_until_micros = micros() + ms * 1000UL;
  pulse_active = true;

  Serial.print(F("pulse "));
  Serial.print(dir > 0 ? F("right") : F("left"));
  Serial.print(F(" "));
  Serial.print(ms);
  Serial.print(F("ms @ pwm "));
  Serial.println(pwm);
}

void capture(int32_t& slot, const __FlashStringHelper* label) {
  uint16_t a = read_angle();
  slot = a;
  Serial.print(F("captured "));
  Serial.print(label);
  Serial.print(F(" = "));
  Serial.println(a);
}

void print_captured() {
  Serial.println(F("--- captured values ---"));
  Serial.print(F("center = "));
  if (captured_center < 0) Serial.println(F("(unset)"));
  else Serial.println(captured_center);
  Serial.print(F("left   = "));
  if (captured_left < 0) Serial.println(F("(unset)"));
  else Serial.println(captured_left);
  Serial.print(F("right  = "));
  if (captured_right < 0) Serial.println(F("(unset)"));
  else Serial.println(captured_right);

  if (captured_center >= 0 && captured_left >= 0 && captured_right >= 0) {
    Serial.println(F("--- config.h snippet ---"));
    Serial.print(F("constexpr uint16_t STEER_CENTER_RAW    = "));
    Serial.print(captured_center);
    Serial.println(F(";"));
    Serial.print(F("constexpr uint16_t STEER_MAX_LEFT_RAW  = "));
    Serial.print(captured_left);
    Serial.println(F(";"));
    Serial.print(F("constexpr uint16_t STEER_MAX_RIGHT_RAW = "));
    Serial.print(captured_right);
    Serial.println(F(";"));
  }
}

// Splits "r 100 80" into argv parts. Returns count.
uint8_t tokenize(char* line, char* argv[], uint8_t max_args) {
  uint8_t n = 0;
  char* p = line;
  while (*p && n < max_args) {
    while (*p == ' ' || *p == '\t') ++p;
    if (!*p) break;
    argv[n++] = p;
    while (*p && *p != ' ' && *p != '\t') ++p;
    if (*p) *p++ = '\0';
  }
  return n;
}

void handle_line(char* line) {
  // Trim trailing CR/whitespace.
  for (char* p = line; *p; ++p) {
    if (*p == '\r' || *p == '\n') { *p = '\0'; break; }
  }

  char* argv[4];
  uint8_t argc = tokenize(line, argv, 4);
  if (argc == 0) {
    stop_pulse();
    return;
  }

  const char* cmd = argv[0];

  if (!strcmp(cmd, "s")) {
    stop_pulse();
    Serial.println(F("stopped"));
  } else if (!strcmp(cmd, "c")) {
    capture(captured_center, F("center"));
  } else if (!strcmp(cmd, "ml")) {
    capture(captured_left, F("max-left"));
  } else if (!strcmp(cmd, "mr")) {
    capture(captured_right, F("max-right"));
  } else if (!strcmp(cmd, "p")) {
    print_captured();
  } else if (!strcmp(cmd, "r") || !strcmp(cmd, "l")) {
    if (argc < 2) {
      Serial.println(F("usage: r <ms> [pwm]"));
      return;
    }
    uint32_t ms = (uint32_t)atol(argv[1]);
    uint8_t pwm = (argc >= 3) ? (uint8_t)atoi(argv[2]) : DEFAULT_PWM;
    start_pulse(cmd[0] == 'r' ? +1 : -1, ms, pwm);
  } else {
    Serial.print(F("unknown: "));
    Serial.println(cmd);
    stop_pulse();
  }
}

void poll_serial() {
  static char buf[32];
  static uint8_t len = 0;

  while (Serial.available()) {
    // Mid-pulse: any incoming byte aborts immediately.
    if (pulse_active) stop_pulse();

    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (len > 0) {
        buf[len] = '\0';
        handle_line(buf);
        len = 0;
      }
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    } else {
      // Overflow: drop and reset.
      len = 0;
    }
  }
}

void setup() {
  pinMode(PIN_STEER_R_EN, OUTPUT);
  pinMode(PIN_STEER_L_EN, OUTPUT);
  pinMode(PIN_STEER_RPWM, OUTPUT);
  pinMode(PIN_STEER_LPWM, OUTPUT);
  digitalWrite(PIN_STEER_R_EN, HIGH);
  digitalWrite(PIN_STEER_L_EN, HIGH);
  analogWrite(PIN_STEER_RPWM, 0);
  analogWrite(PIN_STEER_LPWM, 0);

  pinMode(PIN_STEER_POT, INPUT);

  Serial.begin(115200);
  while (!Serial) {}

  Serial.println(F("steering calibration (10k pot)"));
  uint16_t a = read_angle();
  if (a == 0 || a >= 1023) {
    Serial.print(F("WARNING: pot reads "));
    Serial.print(a);
    Serial.println(F(" (rail) - check wiring/wiper"));
  } else {
    Serial.println(F("pot ok"));
  }
}

void loop() {
  if (pulse_active && (int32_t)(micros() - pulse_until_micros) >= 0) {
    stop_pulse();
  }

  poll_serial();

  uint32_t now = millis();
  if (now - last_angle_print_ms >= ANGLE_PRINT_PERIOD_MS) {
    last_angle_print_ms = now;
    Serial.print(F("angle "));
    Serial.println(read_angle());
  }
}
