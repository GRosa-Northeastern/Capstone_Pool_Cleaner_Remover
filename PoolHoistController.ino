/* --- Contact Switches Pins --- */
const uint8_t lower_contactSwitch = 8;
const uint8_t upper_contactSwitch = 9;

/* --- Motor Pin --- */
const uint8_t motor_pin = 3; // Should be controlled via PWM on Arduino

/* --- Button Pins --- */
const uint8_t lift_button = 4;
const uint8_t lower_button = 5;
const uint8_t mode_switch = 6;
const uint8_t e_stop = 7;

/* --- Other Variables (no title yet) --- */
const unsigned long DEBOUNCE_MS = 35;

/**
 * @brief Configure pins, force safe output states, and perform minimal wiring checks.
 *
 * Details:
 * - All buttons/switches are configured with INPUT_PULLUP, so ACTIVE == LOW.
 * - Motor is disabled at boot (analogWrite 0).
 * - Warn if both contact switches read active or if E-STOP is tripped.
 *
 * @return void
 */
void initializeHardware();

/**
 * @brief Immediately stop motor output (PWM = 0).
 * @return void
 */
void motorStop();

/**
 * @brief Drive the motor with a PWM duty (0–255). Direction is handled elsewhere.
 *
 * @param duty PWM value (0–255). Values above 255 are clamped.
 * @return void
 */
void motorRun(uint8_t duty);

/**
 * @brief Read a contact switch wired with INPUT_PULLUP (active-low).
 *
 * @param pin Digital pin of the switch.
 * @return true if the switch is CLOSED/ACTIVE, false otherwise.
 */
bool readContactSwitch(uint8_t pin);

void setup(){
   // Setup Serial Monitor for testing
   Serial.begin(115200);
   
   /* --- initilizeHarware() --- */
   initializeHardware();

   /* --- Boot up banner --- */ 
   Serial.println("[BOOT] Pins initialized.");
   Serial.print("  E-STOP level: "); Serial.println(digitalRead(e_stop));
   Serial.print("  Lower SW: ");     Serial.println(digitalRead(lower_contactSwitch));
   Serial.print("  Upper SW: ");     Serial.println(digitalRead(upper_contactSwitch));
}

void loop() {
   // simulate being in water/tension mode:
   TensionStatus status = tensionHoldTask();

   static TensionStatus last = (TensionStatus)255;
   if (status != last) {
      last = status;
      if (status == TENSION_CONFLICT) {
         Serial.println(F("[TENSION] Conflict: both switches active."));
      }
      else if (status == TENSION_OK) {
         Serial.println(F("[TENSION] Correction jog."));
      }
      else {
         Serial.println(F("[TENSION] In-band / idle."));
      }                       
  }
}

// -----------------------------------------------------------------------------
// Implementations
// -----------------------------------------------------------------------------

void initializeHardware() {
   /* --- Contact Switches Inputs --- */
   pinMode(lower_contactSwitch, INPUT_PULLUP);
   pinMode(upper_contactSwitch, INPUT_PULLUP);

   /* --- Buttons Inputs --- */
   pinMode(lift_button,  INPUT_PULLUP);
   pinMode(lower_button, INPUT_PULLUP);
   pinMode(mode_switch,  INPUT_PULLUP);

   /* E-STOP: normally-closed chain → HIGH when OK, LOW when tripped */
   pinMode(e_stop, INPUT_PULLUP);

   /* --- Motor Output --- */
   pinMode(motor_pin, OUTPUT);
   motorStop();  // motor should not be on when machine is turned on

   /* --- Failure Detection to be built later using Onboard LED ---*/
   pinMode(LED_BUILTIN, OUTPUT);
   digitalWrite(LED_BUILTIN, LOW);

   /* --- Wiring Checks --- */
   const bool looseActive = readContactSwitch(lower_contactSwitch);
   const bool tightActive = readContactSwitch(upper_contactSwitch);
   if (looseActive && tightActive) {
      Serial.println(F("[WARNING] Both contact switches read ACTIVE at boot (wiring or mechanical conflict)."));
   }
   if (digitalRead(e_stop) == LOW) {
      Serial.println(F("[WARNING] E-STOP is TRIPPED at boot. Clear before operation."));
   }
}

/* --- Motor Functions --- */
void motorStop() {
   analogWrite(motor_pin, 0);
}

void motorRun(uint8_t duty) {
   // Clamp and write PWM
   if (duty > 255) {
      duty = 255;
   }
   analogWrite(motor_pin, duty);
}

/* --- Contact Switch Readings --- */
bool readContactSwitch(uint8_t pin) {
   // Active-low: LOW means switch is closed/triggered
   return digitalRead(pin) == LOW;
}

/* ===========================
      Tensioner Controller 
   =========================== 
*/
/* --- Tuning knobs --- */
const uint8_t  DUTY_PULL         = 170;      // PWM for pull jogs (0–255)
const uint8_t  DUTY_DROP         = 150;      // PWM for drop jogs (0–255)
const unsigned long JOG_PULSE_MS = 220;      // how long to jog each correction
const unsigned long COOLDOWN_MS  = 500;      // quiet time after a jog to avoid noise

/* --- Diagnostics --- */
enum TensionStatus : uint8_t {
   TENSION_OK = 0,
   TENSION_CONFLICT,   // both switches active
   TENSION_IDLE,       // neither switch; in band
};
volatile TensionStatus tensionStatus = TENSION_IDLE;

/* --- Internal jog state --- */
static bool           jogActive      = false;
static unsigned long  jogEndAt_ms    = 0;
static unsigned long  cooldownUntil  = 0;

/* -------------------------------------------------------
 * driveMotorPull / driveMotorDrop
 * For now these just call motorRun() so we can compile and test timing.
 * We can replace the bodies when we add a DIR pin / H-bridge / relay mapping.
 * ------------------------------------------------------- 
 */
void driveMotorPull(uint8_t duty) {
   // TODO: set DIR=REVERSE; then PWM
   motorRun(duty);
}

void driveMotorDrop(uint8_t duty) {
   // TODO: set DIR=FORWARD; then PWM
   motorRun(duty);
}

/* -------------------------------------------------------
 * startJog
 * Begin a time-limited jog in a given direction.
 * ------------------------------------------------------- 
 */
void startJog(bool pullDirection, uint8_t duty, unsigned long pulse_ms) {
   if (pullDirection) {
      driveMotorPull(duty);
   }
   else {
      driveMotorDrop(duty);
   }

  jogActive   = true;
  jogEndAt_ms = millis() + pulse_ms;
}

/* -------------------------------------------------------
 * serviceJog
 * Non-blocking: stops the motor when the jog time expires
 * and starts the cooldown window.
 * ------------------------------------------------------- */
void serviceJog() {
  if (jogActive && (long)(millis() - jogEndAt_ms) >= 0) {
    motorStop();
    jogActive     = false;
    cooldownUntil = millis() + COOLDOWN_MS;
  }
}

/* -------------------------------------------------------
 * tensionHoldTask
 * Call this every loop while you're in "water/tension mode".
 * Uses the contact switches to keep the cord in a tension band by jogging.
 *
 * Behavior:
 *  - If both switches are active -> conflict status (don’t move).
 *  - If cooldown active -> don’t start a new jog yet.
 *  - If LOOSE -> jog PULL for JOG_PULSE_MS.
 *  - If TIGHT -> jog DROP for JOG_PULSE_MS.
 *  - Else -> do nothing (in-band).
 *
 * Returns: current TensionStatus (OK / CONFLICT / IDLE)
 * ------------------------------------------------------- 
 */
TensionStatus tensionHoldTask() {
   // Always keep the timed jog serviced
   serviceJog();

   // If we're currently jogging or cooling down, we don't start a new jog
   if (jogActive || (long)(millis() - cooldownUntil) < 0) {
      return tensionStatus; // unchanged status during jog/cooldown
   }

   const bool looseActive = readContactSwitch(lower_contactSwitch); // active-low
   const bool tightActive = readContactSwitch(upper_contactSwitch); // active-low

   // Sensor conflict: both switches should never be active together
   if (looseActive && tightActive) {
      motorStop();
      tensionStatus = TENSION_CONFLICT;
      return tensionStatus;
   }

   /* --- Decide on corrective action --- */
   if (looseActive) {
      startJog(/*pullDirection=*/true, DUTY_PULL, JOG_PULSE_MS);
      tensionStatus = TENSION_OK;
   } 
   else if (tightActive) {
      startJog(/*pullDirection=*/false, DUTY_DROP, JOG_PULSE_MS);
      tensionStatus = TENSION_OK;
   } 
   else {
      // In the sweet spot — no action
      motorStop();
      tensionStatus = TENSION_IDLE;
   }
   return tensionStatus;
}