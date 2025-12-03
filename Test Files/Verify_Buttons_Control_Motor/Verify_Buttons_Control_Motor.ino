// CHANGE PINS

/* Button IO */
const int LIFT_BUTTON_PIN = 2;
const int LOWER_BUTTON_PIN = 3;

/* Motor IO */
const int DIR_PIN = 4;
const int PWM_PIN = 9; 
const int PWM_SPEED = 200; // Adjust motor speed
const int DIR_LIFT = LOW; // flip or note down if this is correct
const int DIR_LOWER = HIGH;

void setup() {
  // put your setup code here, to run once:
  pinMode(LIFT_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LOWER_BUTTON_PIN, INPUT_PULLUP);

  /* Motor Pins */
  pinMode(DIR_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  analogWrite(PWM_PIN, 0); // ensures motor is off at onset

  Serial.begin(115200); // Make sure serial monitor is correct

}

void loop() {
  // put your main code here, to run repeatedly:
  bool liftPressed = (digitalRead(LIFT_BUTTON_PIN) == LOW);
  bool lowerPressed = (digitalRead(LOWER_BUTTON_PIN) == LOW);

  /* If Chain where if a button is pushed it runs a function otherwise motor off */
  if (liftPressed && !lowerPressed) {
    // LIFT
    digitalWrite(DIR_PIN, DIR_LIFT_LEVEL);
    analogWrite(PWM_PIN, PWM_SPEED);
    Serial.println("Lift Button Pressed")
  }
  else if (lowerPressed && !liftPressed) {
    // LOWER
    digitalWrite(DIR_PIN, DIR_LOWER_LEVEL);
    analogWrite(PWM_PIN, PWM_SPEED);
    Serial.println("Lower Button Pressed")
  }
  else {
    // No button or both buttons -> STOP motor
    analogWrite(PWM_PIN, 0);
  }
  // delay(); can add if you want to slow things down
}
