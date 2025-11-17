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

/* --- Other Variables --- */
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
 * @param duty PWM value (0–255). Values above 255 are clamped.
 * @return void
 */
void motorRun(uint8_t duty);

/**
 * @brief Read a contact switch wired with INPUT_PULLUP
 * @param pin Digital pin of the switch.
 * @return true if the switch is CLOSED/ACTIVE, false otherwise.
 */
bool readContactSwitch(uint8_t pin);

/** 
 *  @brief Read a button press and return true exactly once per press.
 *  @param pin Digital pin.
 *  @return true on the press edge, else false.
 */
bool readPressEdge(uint8_t pin);

/** 
 *  @brief Read a long-press
 *  @param pin Digital pin
 *  @param hold_ms Hold duration in ms.
 *  @return true once when the long-press completes.
 */
bool readLongPress(uint8_t pin, unsigned long hold_ms);

/** 
 *  @brief True if E-STOP input is tripped 
 */
bool estopTripped();

/** 
 *  @brief Immediately cut motor output and latch the controller in EStopped. 
 */
void enterEStopped();

/** 
 *  @brief Attempt reset from E-STOP latch using current chosen gesture.
 *  Requires E-STOP healthy. Returns true if reset succeeded.
 */
bool tryResetFromEStop();

/** 
 *  @brief Drive motor in the PULL (retract) direction at a duty.
 *  set DIR=REVERSE; analogWrite(PWM, duty)
 *  @param duty PWM 0–255.
 */
void driveMotorPull(uint8_t duty);

/** 
 *  @brief Drive motor in the DROP (let-out) direction at a duty.
 *  TODO set DIR=FORWARD; analogWrite(PWM, duty)
 *  @param duty PWM 0–255.
 */
void driveMotorDrop(uint8_t duty);

/** 
 *  @brief Start a time-limited jog in the chosen direction
 *  @param pullDirection true=PULL, false=DROP
 *  @param duty PWM 0–255
 *  @param pulse_ms Jog duration in ms
 */
void startJog(bool pullDirection, uint8_t duty, unsigned long pulse_ms);

/** 
 *  @brief Service an in-progress jog; auto-stop and start cooldown when time elapses. 
 */
void serviceJog();

/* --- Diagnostics --- */
enum TensionStatus : uint8_t {
   TENSION_OK = 0,
   TENSION_CONFLICT,   // both switches active
   TENSION_IDLE,       // neither switch; in band
};

/** 
 *  @brief Uses contact switches to issue brief correction jogs with cooldown.
 *  @return TensionStatus::OK, ::Conflict, or ::InBand
 */
TensionStatus tensionHoldTask();

/** 
 *  @brief One FSM step. Handles Ready↔TensionHold, Fault, and EStopped. 
 */
void fsmStep();


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
   fsmStep();
}

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

/* --- Button helpers --- */
bool readPressEdge(uint8_t pin) {
   static uint8_t last[20];
   static bool seeded[20] = {false};

   uint8_t r = digitalRead(pin);
   if (!seeded[pin]) {          // first time this pin is read
      last[pin] = r;             // seed with actual current level
      seeded[pin] = true;
      return false;              // don’t report an edge on the first read
   }
   bool edge = (last[pin] == HIGH && r == LOW);
   last[pin] = r;
   return edge;
}

bool readLongPress(uint8_t pin, unsigned long hold_ms) {
   static unsigned long tStart[20] = {0};
   if (digitalRead(pin) == LOW) {
      if (tStart[pin] == 0) tStart[pin] = millis();
      if ((long)(millis() - tStart[pin]) >= (long)hold_ms) {
         tStart[pin] = 0;
         return true;
      }
   } 
   else {
      tStart[pin] = 0;
   }
   return false;
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

/* --- Tensioner Controller ---*/
/* --- Tuning knobs --- */
const uint8_t  DUTY_PULL         = 170;      // PWM for pull jogs (0–255)
const uint8_t  DUTY_DROP         = 150;      // PWM for drop jogs (0–255)
const unsigned long JOG_PULSE_MS = 220;      // how long to jog each correction
const unsigned long COOLDOWN_MS  = 500;      // quiet time after a jog to avoid noise
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
 * Call this every loop while machine is in "water/tension mode".
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

/* --- E-STOP guard --- */

bool estopTripped() { return digitalRead(e_stop) == LOW; }

enum class State : uint8_t { Ready, TensionHold, Fault, EStopped };
State state = State::Ready;

void enterEStopped() {
   motorStop();
   state = State::EStopped;
   Serial.println(F("[E-STOP] TRIPPED -> power cut, software latched."));
}

bool tryResetFromEStop() {
   if (estopTripped()) return false; // cannot reset while pressed
   // Current reset gesture is holding mode_switch, but can be changed or just removed (I don't like the idea of removing this as I would rather they intentionally need to do something to reset)
   if (readLongPress(mode_switch, 1500)) {
      Serial.println(F("[E-STOP] Reset gesture accepted. Rebooting logic..."));
      // optional: re-run checks
      state = State::Ready;
      return true;
   }
   return false;
}

/* --- Minimal FSM (READY ↔ TENSION_HOLD) --- */

const uint8_t LIFT_DUTY  = 180;  // tune later
const uint8_t DROP_DUTY  = 160;

void fsmStep() {
   // Global, highest priority: hard E-STOP
   if (estopTripped() && state != State::EStopped) {
      enterEStopped();
      return;
   }

   switch (state) {
      case State::EStopped:
         motorStop();
         Serial.println(F("[STATE] -> Estop Tripped"));
         tryResetFromEStop();
         return;

      case State::Fault:
         motorStop();
         if (readLongPress(mode_switch, 1500) && !estopTripped()) {
            state = State::Ready;
         }
         return;

      case State::Ready: {
         // Toggle into tension mode on MODE button press edge
         if (readPressEdge(mode_switch)) {
            motorStop();
            state = State::TensionHold;
            Serial.println(F("[STATE] -> TENSION_HOLD"));
            break;
         }

         // Hold-to-run manual control
         bool liftHeld  = (digitalRead(lift_button)  == LOW);
         bool lowerHeld = (digitalRead(lower_button) == LOW);

         if (liftHeld && !lowerHeld) {
            // retract while held
            driveMotorPull(LIFT_DUTY);
         } else if (lowerHeld && !liftHeld) {
            // let out while held
            driveMotorDrop(DROP_DUTY);
         } else {
            // neither (or both) -> stop
            motorStop();
         }
      } 
      break;

      case State::TensionHold: {
         // Allow manual override while in water:
         bool liftHeld  = (digitalRead(lift_button)  == LOW);
         bool lowerHeld = (digitalRead(lower_button) == LOW);
         if (liftHeld != lowerHeld) {
            // exactly one held -> manual overrides tension
            if (liftHeld)  driveMotorPull(LIFT_DUTY);
            if (lowerHeld) driveMotorDrop(DROP_DUTY);
         } else {
            // no manual override -> run the tension keeper
            TensionStatus s = tensionHoldTask();
            if (s == TENSION_CONFLICT) {
               Serial.println(F("[FAULT] Tension switch conflict."));
               state = State::Fault;
            }
         }

         // Toggle back to READY on MODE press
         if (readPressEdge(mode_switch)) {
            motorStop();
            state = State::Ready;
            Serial.println(F("[STATE] -> READY"));
         }
      } break;
   }
}

