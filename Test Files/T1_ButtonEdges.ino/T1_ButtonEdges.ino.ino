/*
 * T1_ButtonEdges.ino
 * Purpose: Confirm per-press edge detection and long-press timing (no motor activity).
 */

const uint8_t lift_button  = 4;
const uint8_t lower_button = 5;
const uint8_t mode_switch  = 6;

/* --- Edge + long-press helpers (standalone copy) --- */
bool readPressEdge(uint8_t pin) {
  static uint8_t last[20];
  static bool seeded[20] = {false};

  uint8_t r = digitalRead(pin);
  if (!seeded[pin]) { last[pin] = r; seeded[pin] = true; return false; }
  bool edge = (last[pin] == HIGH && r == LOW);
  last[pin] = r;
  return edge;
}

bool readLongPress(uint8_t pin, unsigned long hold_ms) {
  static unsigned long tStart[20] = {0};
  if (digitalRead(pin) == LOW) {
    if (tStart[pin] == 0) tStart[pin] = millis();
    if ((long)(millis() - tStart[pin]) >= (long)hold_ms) { tStart[pin] = 0; return true; }
  } else {
    tStart[pin] = 0;
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  pinMode(lift_button,  INPUT_PULLUP);
  pinMode(lower_button, INPUT_PULLUP);
  pinMode(mode_switch,  INPUT_PULLUP);
  Serial.println(F("[T1] Edges/Long-press — tap buttons for EDGE events; hold MODE ~1.5s for LONG."));
}

void loop() {
  if (readPressEdge(lift_button))  Serial.println(F("EDGE: LIFT"));
  if (readPressEdge(lower_button)) Serial.println(F("EDGE: LOWER"));
  if (readPressEdge(mode_switch))  Serial.println(F("EDGE: MODE"));

  if (readLongPress(mode_switch, 1500)) Serial.println(F("LONG: MODE (>=1500 ms)"));
}
