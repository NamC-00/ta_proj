/*
  multi_motor_loop_smooth.ino
  ----------------------------------------
  - 2 Servo motors (servo1, servo2)
  - 1 DC motor (constant low-speed clockwise)
  - Serial signal "CAPTURE" to trigger Python screenshot or camera

  Wiring:
  - Servo1 signal -> Pin 9
  - Servo2 signal -> Pin 10
  - DC motor DIR  -> Pin 4
  - DC motor PWM  -> Pin 3 (PWM)
  - Common ground between Arduino, servo power, and DC motor driver
*/

#include <Servo.h>

// ---------- CONFIG ----------
const uint8_t SERVO1_PIN = 9;
const uint8_t SERVO2_PIN = 10;

const uint8_t DC_DIR_PIN  = 4;   // H-bridge direction (HIGH = CW)
const uint8_t DC_PWM_PIN  = 3;   // PWM control pin

const int DC_SPEED = 100;            // (0–255) low speed
const int SERVO_STEP_DELAY = 10;     // ms per degree for smooth motion (used by servo2)
const int LOOP_DELAY_MS = 1000;      // delay between cycles (ms)

// NEW: desired time (ms) to move servo1 across its travel (used to compute uniform step delay)
const int SERVO1_MOVE_TIME_MS = 900; // total ms for full ~180° travel (adjust to taste)
// ----------------------------

Servo servo1;
Servo servo2;

// Function for smooth servo movement (unchanged — still used for servo2)
void smoothMove(Servo &servo, int fromPos, int toPos, int stepDelay) {
  if (fromPos < toPos) {
    for (int pos = fromPos; pos <= toPos; pos++) {
      servo.write(pos);
      delay(stepDelay);
    }
  } else {
    for (int pos = fromPos; pos >= toPos; pos--) {
      servo.write(pos);
      delay(stepDelay);
    }
  }
}

// NEW: move servo1 with uniform speed regardless of angle distance
void smoothMoveTimed(Servo &servo, int fromPos, int toPos, int totalMoveMs) {
  int delta = abs(toPos - fromPos);
  if (delta == 0) return;
  // compute delay per degree (at least 1 ms)
  int stepDelay = totalMoveMs / delta;
  if (stepDelay < 1) stepDelay = 1;

  if (fromPos < toPos) {
    for (int pos = fromPos; pos <= toPos; pos++) {
      servo.write(pos);
      delay(stepDelay);
    }
  } else {
    for (int pos = fromPos; pos >= toPos; pos--) {
      servo.write(pos);
      delay(stepDelay);
    }
  }
}

void setup() {
  Serial.begin(9600);

  // Attach servos
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);

  // Set initial positions
  servo1.write(55);
  servo2.write(0);
  delay(1000);

  // DC motor setup
  pinMode(DC_DIR_PIN, OUTPUT);
  pinMode(DC_PWM_PIN, OUTPUT);

  // Start DC motor clockwise at low speed
  digitalWrite(DC_DIR_PIN, HIGH);
  analogWrite(DC_PWM_PIN, DC_SPEED);

  Serial.println("System Initialized");
}

void loop() {
  smoothMoveTimed(servo1, 50, 100, SERVO1_MOVE_TIME_MS);
  delay(1000);

  smoothMove(servo2, 0, 50, SERVO_STEP_DELAY);
  delay(1000);

  smoothMoveTimed(servo1, 100, 170, SERVO1_MOVE_TIME_MS);
  delay(1000);

  smoothMove(servo2, 50, 180, SERVO_STEP_DELAY);
  delay(1000);

  Serial.println("CAPTURE");  // Python listens for this message
  delay(3000);

  smoothMove(servo2, 180,0, SERVO_STEP_DELAY);
  delay(1000);

  smoothMoveTimed(servo1, 170, 50, SERVO1_MOVE_TIME_MS);
  delay(1000);


  delay(LOOP_DELAY_MS);
  }
