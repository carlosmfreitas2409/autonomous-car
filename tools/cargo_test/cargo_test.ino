#include <Arduino.h>
#include <HX711.h>

constexpr uint8_t PIN_LINK_TX = 32;
constexpr uint8_t PIN_LINK_RX = 33;
constexpr uint8_t PIN_SCALE_DT = 26;
constexpr uint8_t PIN_SCALE_SCK = 25;
constexpr uint8_t PIN_LID = 13;
constexpr uint8_t PIN_LOCK = 2;

constexpr uint32_t LOCK_MAX_ON_MS = 60000;
constexpr uint32_t PRINT_PERIOD_MS = 500;

HX711 scale;
float counts_per_gram = 423.4243f;

bool lock_on = false;
uint32_t lock_on_since = 0;
uint32_t pulse_until = 0;

bool link_send = false;
uint32_t last_link_send = 0;
uint32_t tx_count = 0;
uint32_t rx_count = 0;
String last_rx = "-";
String link_line;

float weight_sum = 0;
int weight_samples = 0;
float last_weights[10];
uint8_t weight_index = 0;
uint32_t last_print = 0;

String line;

void write_lock(bool unlocked) {
  digitalWrite(PIN_LOCK, unlocked ? LOW : HIGH);
}

void lock_off(const char* why) {
  if (lock_on || pulse_until) {
    Serial.printf("lock OFF (%s) after %lu ms\n", why, (unsigned long)(millis() - lock_on_since));
  }
  write_lock(false);
  lock_on = false;
  pulse_until = 0;
}

bool lid_closed() {
  return digitalRead(PIN_LID) == HIGH;
}

void update_lock() {
  const uint32_t now = millis();
  if (pulse_until && (int32_t)(now - pulse_until) >= 0) lock_off("pulse done");
  if (!lock_on) return;
  if (!lid_closed()) lock_off("lid opened");
  else if (now - lock_on_since >= LOCK_MAX_ON_MS) lock_off("60 s cap");
}

void unlock() {
  if (!lid_closed()) {
    Serial.println("lid is open: the lock only powers with the lid closed");
    return;
  }
  write_lock(true);
  lock_on = true;
  lock_on_since = millis();
  Serial.println("lock ON (pin LOW): unlocked");
}

void handle_link_line(const String& received) {
  rx_count++;
  last_rx = received;
  if (received == "K 1" && !lock_on) {
    Serial.println("K 1 from the master, unlocking");
    unlock();
  } else if (received == "K 0" && lock_on) {
    Serial.println("K 0 from the master, locking");
    lock_off("K 0");
  } else if (received != "K 0" && received != "K 1") {
    Serial.printf("received \"%s\" (garbled? check the wiring and the common ground)\n", received.c_str());
  }
}

bool weight_stable() {
  float low = last_weights[0], high = last_weights[0];
  for (float w : last_weights) {
    low = min(low, w);
    high = max(high, w);
  }
  return high - low <= 5.0f;
}

void update_link() {
  while (Serial2.available()) {
    const char c = Serial2.read();
    if (c == '\n') {
      handle_link_line(link_line);
      link_line = "";
    } else if (c != '\r' && link_line.length() < 32) {
      link_line += c;
    }
  }

  const uint32_t now = millis();
  if (link_send && now - last_link_send >= 100) {
    last_link_send = now;
    const int grams = (int)lroundf(last_weights[(weight_index + 9) % 10]);
    Serial2.printf("C %d %d %d %d 0\n", grams, weight_stable(), lid_closed(), lock_on);
    tx_count++;
  }
}

void update_scale() {
  if (!scale.is_ready()) return;
  const float grams = scale.get_units(1);
  weight_sum += grams;
  weight_samples++;
  last_weights[weight_index] = grams;
  weight_index = (weight_index + 1) % 10;
}

void print_status() {
  const uint32_t now = millis();
  if (now - last_print < PRINT_PERIOD_MS) return;
  last_print = now;

  if (weight_samples > 0) {
    Serial.printf("weight %.1f g (%s)", weight_sum / weight_samples, weight_stable() ? "stable" : "settling");
  } else {
    Serial.print("weight -- (HX711 not answering)");
  }
  Serial.printf(" | lid %s | lock %s | link tx %lu rx %lu (last %s)%s\n",
                lid_closed() ? "closed" : "OPEN",
                lock_on ? "ON (unlocked)" : "off",
                (unsigned long)tx_count, (unsigned long)rx_count, last_rx.c_str(),
                link_send ? " (sending C)" : "");
  weight_sum = 0;
  weight_samples = 0;
}

void print_help() {
  Serial.println("t tare | <grams> calibrate | u unlock | l lock | p <ms> pulse | c send C lines | ? help");
}

void handle(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd == "?") {
    print_help();
  } else if (cmd == "t") {
    scale.tare(20);
    Serial.println("tared");
  } else if (cmd == "u") {
    unlock();
  } else if (cmd == "l") {
    lock_off("l");
  } else if (cmd == "c") {
    link_send = !link_send;
    Serial.printf("C lines to the sonar node %s\n", link_send ? "on" : "off");
  } else if (cmd.startsWith("p ")) {
    const long ms = constrain(cmd.substring(2).toInt(), 1, 2000);
    write_lock(true);
    lock_on_since = millis();
    pulse_until = lock_on_since + ms;
    Serial.printf("pulse %ld ms\n", ms);
  } else if (cmd.toFloat() > 0) {
    const float grams = cmd.toFloat();
    const long counts = scale.get_value(20);
    counts_per_gram = counts / grams;
    scale.set_scale(counts_per_gram);
    Serial.printf("SCALE_COUNTS_PER_GRAM = %.4ff;  (firmware/cargo/src/config.h)\n", counts_per_gram);
  } else {
    Serial.println("unknown command");
    print_help();
  }
}

void setup() {
  digitalWrite(PIN_LOCK, HIGH);
  pinMode(PIN_LOCK, OUTPUT);

  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, PIN_LINK_RX, PIN_LINK_TX);
  pinMode(PIN_LID, INPUT);

  scale.begin(PIN_SCALE_DT, PIN_SCALE_SCK);
  scale.set_scale(counts_per_gram);
  if (scale.wait_ready_timeout(1000)) {
    scale.tare(20);
  } else {
    Serial.println("HX711 not answering: check DT/SCK and its power");
  }

  Serial.println("cargo bench tester");
  print_help();
}

void loop() {
  while (Serial.available()) {
    const char c = Serial.read();
    if (c == '\n') {
      handle(line);
      line = "";
    } else if (c != '\r') {
      line += c;
    }
  }
  update_scale();
  update_lock();
  update_link();
  print_status();
}
