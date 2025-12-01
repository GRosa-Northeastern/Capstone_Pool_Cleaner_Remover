/*
 * T0_Wiring_Smoke.ino
 * Purpose: Verify all inputs are wired and read correctly (no motor activity).
 */

const uint8_t lower_contactSwitch = 8;
const uint8_t upper_contactSwitch = 9;

const uint8_t lift_button  = 4;
const uint8_t lower_button = 5;
const uint8_t mode_switch  = 6;

void setup() {
  Serial.begin(115200);

  pinMode(lower_contactSwitch, INPUT_PULLUP);
  pinMode(upper_contactSwitch, INPUT_PULLUP);
  pinMode(lift_button,  INPUT_PULLUP);
  pinMode(lower_button, INPUT_PULLUP);
  pinMode(mode_switch,  INPUT_PULLUP);

  Serial.println(F("[T0] Wiring smoke — idle should read HIGH (INPUT_PULLUP). Press switches to see LOW."));
}

void loop() {
  Serial.print(F("LOWER_SW=")); Serial.print(digitalRead(lower_contactSwitch));
  Serial.print(F("  UPPER_SW=")); Serial.print(digitalRead(upper_contactSwitch));
  Serial.print(F("  LIFT_BTN=")); Serial.print(digitalRead(lift_button));
  Serial.print(F("  LOWER_BTN=")); Serial.print(digitalRead(lower_button));
  Serial.print(F("  MODE_BTN=")); Serial.println(digitalRead(mode_switch));
  delay(200);
}
