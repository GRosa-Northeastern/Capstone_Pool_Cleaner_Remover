/* --- Contact Switches Pins --- */
const uint8_t lower_contactSwitch = 8;
const uint8_t upper_contactSwitch = 9;

/* --- Motor Pin --- */
const uint8_t motor_pin = 3;

/* --- Motor Driver Pins (DIR + PWM + EN + optional BRK) --- */
const uint8_t pinPWM = motor_pin;  
const uint8_t pinDIR = 4;         
// const uint8_t pinEN  = 11;         
const uint8_t pinBRK = 5;         // optional brake input (active-HIGH)

/* --- Manual speeds (PWM duty) --- */
const uint8_t LIFT_DUTY  = 200;   // tune later
const uint8_t DROP_DUTY  = 200;   // tune later

/* --- Button Pins --- */
const uint8_t lift_button = 12;
const uint8_t lower_button = 13;
const uint8_t mode_switch = 11;

/* --- Tension Controller Tuning knobs --- */
const uint8_t  DUTY_PULL         = 200;      // PWM for pull jogs (0–255)
const uint8_t  DUTY_DROP         = 200;      // PWM for drop jogs (0–255)
const unsigned long JOG_PULSE_MS = 220;      // how long to jog each correction
const unsigned long COOLDOWN_MS  = 500;      // quiet time after a jog to avoid noise

/* --- Other Variables --- */
const unsigned long DEBOUNCE_MS = 35;

/**
 * @brief Configure pins, force safe output states, and perform minimal wiring checks.
 *
 * Details:
 * - All buttons/switches are configured with INPUT_PULLUP, so ACTIVE == LOW.
 * - Motor is disabled at boot (analogWrite 0).
 * - Warn if both contact switches read active
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
 *  @brief One FSM step. Handles Ready↔TensionHold, Fault
 */
void fsmStep();

/** 
 *  @brief Manual hold-to-run behavior used by READY (and when exiting TENSION).
 *  Drives PULL/DROP while exactly one of Lift/Lower is held; otherwise stops.
 */
void handleManualRun();

void setup(){
   // Setup Serial Monitor for testing
   Serial.begin(115200);
   
   /* --- initilizeHarware() --- */
   initializeHardware();

   /* --- Boot up banner --- */ 
   Serial.println(F("==== Pool Hoist Controller ===="));
   Serial.print  (F("Build: ")); Serial.print(F(__DATE__)); Serial.print(F(" "));
   Serial.println(F(__TIME__));
   Serial.println(F("Pins: PWM=D3  DIR=D10  EN=D11  BRK=D12"));
   Serial.println(F("DIR map: HIGH=PULL, LOW=DROP")); 
   Serial.print  (F("Tune: LIFT=")); Serial.print(LIFT_DUTY);
   Serial.print  (F(" DROP="));       Serial.print(DROP_DUTY);
   Serial.print  (F(" JOG="));        Serial.print(JOG_PULSE_MS);
   Serial.print  (F("ms COOL="));     Serial.print(COOLDOWN_MS);
   Serial.println(F("ms"));

   Serial.print  (F("Init Levels  LowerSW=")); Serial.print(digitalRead(lower_contactSwitch));
   Serial.print  (F(" UpperSW="));             Serial.print(digitalRead(upper_contactSwitch));
   Serial.print  (F(" BRK="));                 Serial.print(digitalRead(pinBRK));
   Serial.print  (F(" DIR="));                 Serial.println(digitalRead(pinDIR));
   Serial.println(F("================================"));
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

   /* --- Motor Output --- */
   pinMode(motor_pin, OUTPUT);
   
   /* --- Motor Driver IO --- */
   pinMode(pinDIR, OUTPUT);  digitalWrite(pinDIR, LOW);   // LOW should be drop, CHANGE if untrue
   pinMode(pinBRK, OUTPUT);  digitalWrite(pinBRK, LOW);   // brake off

   motorStop();  // PWM = 0, EN = LOW

   /* --- Failure Detection to be built later using Onboard LED ---*/
   pinMode(LED_BUILTIN, OUTPUT);
   digitalWrite(LED_BUILTIN, LOW);

   /* --- Wiring Checks --- */
   const bool looseActive = readContactSwitch(lower_contactSwitch);
   const bool tightActive = readContactSwitch(upper_contactSwitch);
   if (looseActive && tightActive) {
      Serial.println(F("[WARNING] Both contact switches read ACTIVE at boot (wiring or mechanical conflict)."));
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
   analogWrite(pinPWM, 0);
}

void motorRun(uint8_t duty) {
   if (duty > 255){
      duty = 255;
   } 
   analogWrite(pinPWM, duty);   
}


/* --- Contact Switch Readings --- */
bool readContactSwitch(uint8_t pin) {
   // Active-low: LOW means switch is closed/triggered
   return digitalRead(pin) == LOW;
}

/* --- Tensioner Controller ---*/
volatile TensionStatus tensionStatus = TENSION_IDLE;

/* --- Internal jog state --- */
static bool           jogActive      = false;
static unsigned long  jogEndAt_ms    = 0;
static unsigned long  cooldownUntil  = 0;

/*
 * @brief Define PULL and DROP by DIR level. Flip HIGH/LOW meanings if reversed. 
 */
void driveMotorPull(uint8_t duty) {
   digitalWrite(pinDIR, HIGH);  // define HIGH = PULL (retract)
   motorRun(duty);
}

void driveMotorDrop(uint8_t duty) {
   digitalWrite(pinDIR, LOW);   // define LOW  = DROP (let out)
   motorRun(duty);
}

void handleManualRun() {
   bool liftHeld  = (digitalRead(lift_button)  == LOW);
   bool lowerHeld = (digitalRead(lower_button) == LOW);

   if (liftHeld && !lowerHeld) {
      driveMotorPull(LIFT_DUTY);
   } else if (lowerHeld && !liftHeld) {
      driveMotorDrop(DROP_DUTY);
   } else {
      motorStop();
   }
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

enum class State : uint8_t { Ready, TensionHold, Fault };
State state = State::Ready;

/* --- Minimal FSM (READY ↔ TENSION_HOLD) --- */
void fsmStep() {
   switch (state) {
      case State::Fault:
         motorStop();
         if (readLongPress(mode_switch, 1500)) {
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
         // Hold to run manual
         handleManualRun();

         // Manual interlocks: stop if trying to move into an active limit
         if (digitalRead(lift_button) == LOW && readContactSwitch(upper_contactSwitch)) motorStop();
         if (digitalRead(lower_button) == LOW && readContactSwitch(lower_contactSwitch)) motorStop();
      } 
      break;

      case State::TensionHold: {
         // TensionHold
         bool liftHeld  = (digitalRead(lift_button)  == LOW);
         bool lowerHeld = (digitalRead(lower_button) == LOW);

         // Manual override -> exit to READY immediately and act this cycle
         if (liftHeld || lowerHeld) {
            state = State::Ready;
            Serial.println(F("[STATE] TensionHold -> READY (manual override)"));
            handleManualRun();
            break;
         }

         // No manual -> run tension keeper
         TensionStatus s = tensionHoldTask();
         if (s == TENSION_CONFLICT) {
            Serial.println(F("[FAULT] Tension switch conflict."));
            state = State::Fault;
         }

         // Toggle back to READY on MODE press
         if (readPressEdge(mode_switch)) {
            motorStop();
            state = State::Ready;
            Serial.println(F("[STATE] -> READY"));
         } 
      }
      break;
   }
}
