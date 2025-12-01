/*
 * T2_DirPwm_Motor.ino
 * Purpose: Verify BLDC driver wiring (DIR/PWM/EN/BRK) and direction. Run on bench with no load.
 * D3 PWM ~490 Hz on Uno (compatible with most drivers).
 */

const uint8_t pinPWM = 3;   // speed (PWM)
const uint8_t pinDIR = 10;  // direction
const uint8_t pinEN  = 11;  // enable (active-HIGH)
const uint8_t pinBRK = 12;  // brake (active-HIGH) — held LOW here

void motorStop() {
  analogWrite(pinPWM, 0);
  digitalWrite(pinEN, LOW);
  digitalWrite(LED_BUILTIN, LOW);
}

void motorRun(uint8_t duty) {
  if (duty > 255) duty = 255;
  digitalWrite(pinEN, HIGH);
  analogWrite(pinPWM, duty);
  digitalWrite(LED_BUILTIN, duty > 0 ? HIGH : LOW); // LED surrogate
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(pinDIR, OUTPUT);  digitalWrite(pinDIR, LOW); // LOW=DROP (flip if reversed)
  pinMode(pinEN,  OUTPUT);  digitalWrite(pinEN,  LOW);
  pinMode(pinBRK, OUTPUT);  digitalWrite(pinBRK, LOW); // brake off
  pinMode(pinPWM, OUTPUT);  analogWrite(pinPWM, 0);

  delay(500);
}

void loop() {
  // PULL test
  digitalWrite(pinDIR, HIGH);
  motorRun(120);
  delay(500);
  motorStop();
  delay(800);

  // DROP test
  digitalWrite(pinDIR, LOW);
  motorRun(120);
  delay(500);
  motorStop();
  delay(1200);
}
