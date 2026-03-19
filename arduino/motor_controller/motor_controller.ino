// Motor controller for Arduino Mega — 4x BTS7960 H-bridges (one per wheel)
//
// Wiring per BTS7960 module:
//   RPWM → Arduino PWM pin (forward)
//   LPWM → Arduino PWM pin (reverse)
//   R_EN, L_EN, VCC → 5V
//   GND → GND
//   R_IS, L_IS → not connected
//
// Serial protocol (from Python):
//   M <throttle> <steering>\n  — motor command (PWM -255..255)
//   E\n                        — emergency stop
//   P\n                        — heartbeat

// --- Pin assignments (all PWM-capable, avoids Timer0) ---

// Front-Left motor
const int FL_RPWM = 2;
const int FL_LPWM = 3;

// Front-Right motor
const int FR_RPWM = 5;
const int FR_LPWM = 6;

// Rear-Left motor
const int RL_RPWM = 7;
const int RL_LPWM = 8;

// Rear-Right motor
const int RR_RPWM = 9;
const int RR_LPWM = 10;

const unsigned long WATCHDOG_TIMEOUT_MS = 500;
unsigned long lastCommandTime = 0;

String inputBuffer = "";

void setMotor(int rpwmPin, int lpwmPin, int speed) {
  if (speed > 0) {
    analogWrite(rpwmPin, constrain(speed, 0, 255));
    analogWrite(lpwmPin, 0);
  } else if (speed < 0) {
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, constrain(-speed, 0, 255));
  } else {
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, 0);
  }
}

void setAllMotors(int speed) {
  setMotor(FL_RPWM, FL_LPWM, speed);
  setMotor(FR_RPWM, FR_LPWM, speed);
  setMotor(RL_RPWM, RL_LPWM, speed);
  setMotor(RR_RPWM, RR_LPWM, speed);
}

void stopMotors() {
  setAllMotors(0);
}

void processCommand(const String& cmd) {
  if (cmd.length() == 0) return;

  char type = cmd.charAt(0);

  if (type == 'M') {
    int firstSpace = cmd.indexOf(' ');
    if (firstSpace < 0) return;
    int secondSpace = cmd.indexOf(' ', firstSpace + 1);
    if (secondSpace < 0) return;

    int throttleSpeed = cmd.substring(firstSpace + 1, secondSpace).toInt();
    int steeringValue = cmd.substring(secondSpace + 1).toInt();

    // TODO: Implement steering mixing. Options:
    //   1. Trapezoidal spindle + servo: receive steeringValue, command a servo
    //      for mechanical front-wheel steering with limit switches at endpoints.
    //   2. Ackermann via differential speed: adjust per-wheel speeds based on
    //      steeringValue. Inner wheels slow down, outer wheels maintain speed.
    //      Formula: leftSpeed = throttle * (1 - steering * factor)
    //              rightSpeed = throttle * (1 + steering * factor)
    // For now: all 4 motors get the same throttle, steering is ignored.
    (void)steeringValue;

    setAllMotors(throttleSpeed);
    lastCommandTime = millis();

  } else if (type == 'E') {
    stopMotors();
    lastCommandTime = millis();

  } else if (type == 'P') {
    lastCommandTime = millis();
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(FL_RPWM, OUTPUT);
  pinMode(FL_LPWM, OUTPUT);
  pinMode(FR_RPWM, OUTPUT);
  pinMode(FR_LPWM, OUTPUT);
  pinMode(RL_RPWM, OUTPUT);
  pinMode(RL_LPWM, OUTPUT);
  pinMode(RR_RPWM, OUTPUT);
  pinMode(RR_LPWM, OUTPUT);

  stopMotors();
  lastCommandTime = millis();

  inputBuffer.reserve(64);
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      inputBuffer.trim();
      processCommand(inputBuffer);
      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }

  if (millis() - lastCommandTime > WATCHDOG_TIMEOUT_MS) {
    stopMotors();
  }
}
