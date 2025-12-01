/*
 * T3_Ready_HoldToRun.ino
 * Purpose: Validate manual “hold-to-run” behavior with interlocks (no FSM/Tension logic).
 */

const uint8_t lower_contactSwitch = 8;
const uint8_t upper_contactSwitch = 9;

const uint8_t lift_button  = 4;
const uint8_t lower_button = 5;

const uint8_t pinPWM = 3;
const uint8_t pinDIR = 10;
const uint8_t pinEN  = 11;
const uint8_t pinBRK = 12;

const uint8_t LIFT_DUTY = 180;
const uint8_t DROP_DUTY = 160;

void motorStop() {
  analogWrite(pinPWM, 0);
  digitalWrite(pinEN, LOW);
  digitalWrite(LED_BUILTIN, LOW);
}

void motorRun(uint8_t duty) {
  if (duty > 255) duty = 255;
  digitalWrite(pinEN, HIGH);
  analogWrite(pinPWM, duty);
  digitalWrite(LED_BUILTIN, duty > 0 ? HIGH : LOW);
}

void drivePull(uint8_t duty) { digitalWrite(pinDIR, HIGH); motorRun(duty); } // HIGH=PULL
void driveDrop(uint8_t duty) { digitalWrite(pinDIR, LOW);  motorRun(duty); } // LOW=DROP

bool readContactSwitch(uint8_t pin) { return digitalRead(pin) == LOW; } // active-low

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(lower_contactSwitch, INPUT_PULLUP);
  pinMode(upper_contactSwitch, INPUT_PULLUP);
  pinMode(lift_button,  INPUT_PULLUP);
  pinMode(lower_button, INPUT_PULLUP);

  pinMode(pinDIR, OUTPUT);  digitalWrite(pinDIR, LOW);
  pinMode(pinEN,  OUTPUT);  digitalWrite(pinEN,  LOW);
  pinMode(pinBRK, OUTPUT);  digitalWrite(pinBRK, LOW);
  pinMode(pinPWM, OUTPUT);  analogWrite(pinPWM, 0);

  Serial.println(F("[T3] Hold-to-run: hold LIFT/DROP; release to stop. Interlocks stop if limit is active."));
}

void loop() {
  bool liftHeld  = (digitalRead(lift_button)  == LOW);
  bool lowerHeld = (digitalRead(lower_button) == LOW);

  // Manual interlocks: stop if trying to move into an active limit
  if (liftHeld  && readContactSwitch(upper_contactSwitch)) { motorStop(); return; }
  if (lowerHeld && readContactSwitch(lower_contactSwitch)) { motorStop(); return; }

  if (liftHeld && !lowerHeld) {
    drivePull(LIFT_DUTY);
  } else if (lowerHeld && !liftHeld) {
    driveDrop(DROP_DUTY);
  } else {
    motorStop();
  }
}
