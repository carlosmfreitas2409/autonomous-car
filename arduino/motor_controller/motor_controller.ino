const int LEFT_PWM_PIN  = 5;
const int LEFT_DIR_PIN  = 4;
const int RIGHT_PWM_PIN = 6;
const int RIGHT_DIR_PIN = 7;

const unsigned long WATCHDOG_TIMEOUT_MS = 500;

unsigned long lastCommandTime = 0;
String inputBuffer = "";

void setMotor(int pwmPin, int dirPin, int speed) {
  if (speed >= 0) {
    digitalWrite(dirPin, HIGH);
    analogWrite(pwmPin, constrain(speed, 0, 255));
  } else {
    digitalWrite(dirPin, LOW);
    analogWrite(pwmPin, constrain(-speed, 0, 255));
  }
}

void stopMotors() {
  analogWrite(LEFT_PWM_PIN, 0);
  analogWrite(RIGHT_PWM_PIN, 0);
}

void processCommand(const String& cmd) {
  if (cmd.length() == 0) return;

  char type = cmd.charAt(0);

  if (type == 'M') {
    // M <left> <right>
    int firstSpace = cmd.indexOf(' ');
    if (firstSpace < 0) return;
    int secondSpace = cmd.indexOf(' ', firstSpace + 1);
    if (secondSpace < 0) return;

    int leftSpeed  = cmd.substring(firstSpace + 1, secondSpace).toInt();
    int rightSpeed = cmd.substring(secondSpace + 1).toInt();

    setMotor(LEFT_PWM_PIN,  LEFT_DIR_PIN,  leftSpeed);
    setMotor(RIGHT_PWM_PIN, RIGHT_DIR_PIN, rightSpeed);
    lastCommandTime = millis();

  } else if (type == 'E') {
    // Emergency stop
    stopMotors();
    lastCommandTime = millis();

  } else if (type == 'P') {
    // Heartbeat — just reset watchdog
    lastCommandTime = millis();
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(LEFT_PWM_PIN,  OUTPUT);
  pinMode(LEFT_DIR_PIN,  OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);

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
