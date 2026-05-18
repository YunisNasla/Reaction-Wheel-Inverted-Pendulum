#include <Wire.h>

// ===== Pins =====
#define R_EN  8
#define L_EN  11
#define RPWM  9
#define LPWM  10

#define ENC_A 2
#define ENC_B 3
#define ENC_Z 4

// ===== Encoder =====
const long COUNTS_PER_REV = 10000;
volatile long encoderCount = 0;
volatile bool zCalibrated = false;

// ===== PID Controller — TUNING KNOBS =====
double targetAngle = 180.0;
double Kp = 55.0;
double Ki = 5.0;
double Kd = 10.0;
double Kff = 0.6;
double integral = 0;
double lastInput = 0;
unsigned long lastPidTime = 0;
const unsigned long PID_SAMPLE_TIME = 15;

const float PREDICTION_HORIZON_MS = 30;

// ===== Motor stiction & smoothing =====
const int BALANCE_MIN_PWM = 75;
const int BALANCE_DEADBAND = 12;
const int DISTURBANCE_DEADBAND = 0;
const float DISTURBANCE_VEL_THRESHOLD = 25;
const float HARD_DISTURBANCE_VEL = 80;
const float OUTPUT_SMOOTHING = 0.35;

// ===== Direction reversal limiter =====
const unsigned long MIN_DIR_HOLD_MS = 60;
const unsigned long DISTURBANCE_DIR_HOLD_MS = 10;
bool lastBalanceDir = true;
unsigned long lastBalanceDirChange = 0;
double smoothedOutput = 0;

const float KD_BOOST_NEAR_VERTICAL = 1.6;
const float KD_BOOST_ANGLE_THRESHOLD = 5.0;

// ===== Swing-up tuning =====
const int SWINGUP_PWM_MAX = 255;
const int SWINGUP_PWM_MIN = 200;
const float COAST_THRESHOLD_DEG = 45.0;
const float ROTATION_VEL_THRESHOLD = 400.0;
const float ROTATION_ANGLE_THRESHOLD = 90.0;

// ===== Near-miss handling — TUNING KNOBS =====
const float NEAR_MISS_ANGLE = 50.0;          // [TUNE] within this many deg of vert = near-miss
const int SKIP_PUMPS_AFTER_NEAR_MISS = 1;    // [TUNE] skip this many pumps after a near-miss
const float ENERGY_REDUCE_FACTOR = 0.75;     // [TUNE] when overshooting, reduce PWM by this much

// ===== Approach decelerator — TUNING KNOBS =====
const float DECEL_ANGLE_START = 60.0;        // [TUNE] start braking this far from vert
const float DECEL_ANGLE_END = 35.0;          // [TUNE] stop braking when this close to vert
const float DECEL_VEL_THRESHOLD = 200.0;     // [TUNE] only brake if approaching faster than this
const int DECEL_PWM = 180;                   // [TUNE] PWM for braking pulse

// ===== Catch zone tuning =====
const float BALANCE_ANGLE_THRESHOLD = 35.0;
const float SLOW_CATCH_ANGLE = 60.0;
const float SLOW_CATCH_VEL = 300.0;

// ===== State =====
float currentAngle = 0.0;
float previousAngle = 0.0;
float angularVelocity = 0.0;
float lastAngularVelocity = 0.0;
float angularAccelEst = 0.0;
float prevRawAngle = 0.0;
float lastRawAngleForJumpCheck = -1;
bool currentMotorDir = true;
bool motorActive = false;
const unsigned long samplingTime = 10;
unsigned long previousMillis = 0;
unsigned long lastTick = 0;
unsigned long stabilizedTime = 0;
bool wasInBalanceMode = false;

float peakVelMagnitude = 0;
float lastPeakReported = 0;

// Track minimum angle from vertical reached on each swing
float minAngleFromVertThisSwing = 360.0;
int pumpsToSkip = 0;
float energyMultiplier = 1.0;  // ramps down on repeated near-misses

float wheelSpeedEst = 0.0;
const float vert = 180.0;

// ===== Wheel desaturation — TUNING KNOBS =====
const float WHEEL_SPEED_DECAY_RATE = 0.3;   // [TUNE] est decays toward zero at this rate (per sec)
const float WHEEL_ACCEL_GAIN = 0.015;        // [TUNE] how fast sustained output builds the estimate
const float DESAT_MAX_BIAS_DEG = 4.5;        // [TUNE] max degrees targetAngle shifts for desaturation

const int numSamples = 8;
int sampleIndex = 0;
float samples[numSamples];

const int8_t QUAD_TABLE[16] = {
   0, -1, +1,  0,
  +1,  0,  0, -1,
  -1,  0,  0, +1,
   0, +1, -1,  0
};

volatile uint8_t lastAB = 0;

void encoderISR_AB() {
  uint8_t a = digitalRead(ENC_A);
  uint8_t b = digitalRead(ENC_B);
  uint8_t newAB = (a << 1) | b;
  uint8_t idx = (lastAB << 2) | newAB;
  encoderCount += QUAD_TABLE[idx];
  lastAB = newAB;
}

void encoderISR_Z() {
  if (!zCalibrated) {
    encoderCount = 0;
    zCalibrated = true;
    detachInterrupt(digitalPinToInterrupt(ENC_Z));
  }
}

void runMotorPWM(bool dir, int pwm) {
  pwm = constrain(pwm, 0, 255);
  if (dir) {
    analogWrite(RPWM, pwm);
    analogWrite(LPWM, 0);
  } else {
    analogWrite(RPWM, 0);
    analogWrite(LPWM, pwm);
  }
}

void motorStop() {
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
}

void setMotorPID(int output) {
  float velMag = abs(angularVelocity);
  bool disturbance = velMag > DISTURBANCE_VEL_THRESHOLD;
  bool hardDisturbance = velMag > HARD_DISTURBANCE_VEL;

  int activeDeadband = disturbance ? DISTURBANCE_DEADBAND : BALANCE_DEADBAND;

  if (abs(output) < activeDeadband) {
    motorStop();
    return;
  }

  bool desiredDir = (output > 0);
  unsigned long now = millis();

  unsigned long activeHold;
  if (hardDisturbance) {
    activeHold = 0;
  } else if (disturbance) {
    activeHold = DISTURBANCE_DIR_HOLD_MS;
  } else {
    activeHold = MIN_DIR_HOLD_MS;
  }

  if (desiredDir != lastBalanceDir && (now - lastBalanceDirChange) < activeHold) {
    motorStop();
    return;
  }

  if (desiredDir != lastBalanceDir) {
    lastBalanceDir = desiredDir;
    lastBalanceDirChange = now;
  }

  if (output > 0) {
    int pwm = max(output, BALANCE_MIN_PWM);
    pwm = min(pwm, 255);
    analogWrite(RPWM, pwm);
    analogWrite(LPWM, 0);
  } else {
    int pwm = max(-output, BALANCE_MIN_PWM);
    pwm = min(pwm, 255);
    analogWrite(RPWM, 0);
    analogWrite(LPWM, pwm);
  }
}

double computePID(double input, double velocity) {
  unsigned long now = millis();
  unsigned long timeChange = now - lastPidTime;
  if (timeChange < PID_SAMPLE_TIME) return smoothedOutput;

  double dt = PREDICTION_HORIZON_MS / 1000.0;
  double predictedAngle = input + velocity * dt + 0.5 * angularAccelEst * dt * dt;
  double predictedError = targetAngle - predictedAngle;

  double error = predictedError;

  double actualError = targetAngle - input;
  integral += (Ki * actualError * (timeChange / 1000.0));
  if (integral > 255)  integral = 255;
  if (integral < -255) integral = -255;

  float angleFromVert = abs(input - targetAngle);
  double effectiveKd = Kd;
  if (angleFromVert < KD_BOOST_ANGLE_THRESHOLD) {
    effectiveKd = Kd * KD_BOOST_NEAR_VERTICAL;
  }

  double feedforward = -Kff * velocity;

  double rawOutput = Kp * error + integral - effectiveKd * velocity + feedforward;
  if (rawOutput > 255)  rawOutput = 255;
  if (rawOutput < -255) rawOutput = -255;

  float velMag = abs(velocity);
  float dynamicSmoothing;
  if (velMag > HARD_DISTURBANCE_VEL) {
    dynamicSmoothing = 1.0;
  } else if (velMag > DISTURBANCE_VEL_THRESHOLD) {
    dynamicSmoothing = 0.9;
  } else {
    dynamicSmoothing = OUTPUT_SMOOTHING;
  }
  smoothedOutput = dynamicSmoothing * rawOutput + (1.0 - dynamicSmoothing) * smoothedOutput;

  lastInput = input;
  lastPidTime = now;
  return smoothedOutput;
}

void resetPID(double currentInput) {
  integral = 0;
  lastInput = currentInput;
  lastPidTime = millis() - PID_SAMPLE_TIME;  // allow immediate first computation
  lastBalanceDir = true;
  lastBalanceDirChange = millis();
  smoothedOutput = 0;
  angularAccelEst = 0;
  wheelSpeedEst = 0;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("=== RWIP — Adaptive Swing ===");

  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);
  motorStop();

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ENC_Z, INPUT_PULLUP);

  lastAB = (digitalRead(ENC_A) << 1) | digitalRead(ENC_B);

  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR_AB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), encoderISR_AB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_Z), encoderISR_Z, RISING);

  for (int i = 0; i < numSamples; i++) samples[i] = 0;

  Serial.println("\n*** Pendulum at bottom = 0 deg ***");
  Serial.println("*** Swing through bottom once to zero encoder ***");
  Serial.println("*** Starting in 3 sec... ***\n");
  delay(3000);

  Serial.println("*** Pre-spinning wheel... ***");
  runMotorPWM(true, 255);
  delay(800);
  motorStop();
  delay(300);
  Serial.println("*** Pre-spin done. Begin swing-up. ***\n");

  lastPidTime = millis();
  lastTick = millis();
}

void loop() {
  unsigned long now = millis();
  unsigned long deltaTime = now - lastTick;
  lastTick = now;

  if (now - previousMillis >= samplingTime) {
    previousMillis = now;

    noInterrupts();
    long count = encoderCount;
    interrupts();

    float rawAngle = ((float)count * 360.0) / COUNTS_PER_REV;
    while (rawAngle < 0)    rawAngle += 360.0;
    while (rawAngle >= 360) rawAngle -= 360.0;

    if (lastRawAngleForJumpCheck >= 0) {
      float jump = abs(rawAngle - lastRawAngleForJumpCheck);
      if (jump > 180) jump = 360 - jump;
      if (jump > 30) {
        Serial.print("!!! JUMP: ");
        Serial.print(lastRawAngleForJumpCheck, 1);
        Serial.print(" -> ");
        Serial.println(rawAngle, 1);
      }
    }
    lastRawAngleForJumpCheck = rawAngle;

    samples[sampleIndex] = rawAngle;
    sampleIndex = (sampleIndex + 1) % numSamples;
    float sum = 0;
    for (int i = 0; i < numSamples; i++) sum += samples[i];
    currentAngle = sum / numSamples;

    lastAngularVelocity = angularVelocity;
    float rawDelta = rawAngle - prevRawAngle;
    if (rawDelta > 180)  rawDelta -= 360;
    if (rawDelta < -180) rawDelta += 360;
    angularVelocity = rawDelta / (samplingTime / 1000.0);
    prevRawAngle = rawAngle;

    angularAccelEst = (angularVelocity - lastAngularVelocity) / (samplingTime / 1000.0);

    previousAngle = currentAngle;
  }

  float angleFromVert = abs(currentAngle - vert);
  float velMag = abs(angularVelocity);

  bool inCatchZone = (angleFromVert < BALANCE_ANGLE_THRESHOLD);
  bool slowAndClose = (angleFromVert < SLOW_CATCH_ANGLE) && (velMag < SLOW_CATCH_VEL);
  bool shouldBalance = inCatchZone || slowAndClose;

  // ===== SWING-UP =====
  if (!shouldBalance) {
    if (wasInBalanceMode) {
      wasInBalanceMode = false;
      stabilizedTime = 0;
      motorActive = false;
      motorStop();
    }

    // Track how close pendulum got to vertical this swing
    if (angleFromVert < minAngleFromVertThisSwing) {
      minAngleFromVertThisSwing = angleFromVert;
    }

    if (velMag > peakVelMagnitude) peakVelMagnitude = velMag;

    bool signChange = (lastAngularVelocity * angularVelocity < 0) && (abs(lastAngularVelocity) > 5);
    if (signChange) {
      Serial.print(">>> PEAK: "); Serial.print(peakVelMagnitude, 0);
      Serial.print(" | minFromVert: "); Serial.print(minAngleFromVertThisSwing, 1);

      // If this swing was a near-miss, schedule pump skips and reduce energy
      if (minAngleFromVertThisSwing < NEAR_MISS_ANGLE) {
        pumpsToSkip = SKIP_PUMPS_AFTER_NEAR_MISS;
        energyMultiplier *= ENERGY_REDUCE_FACTOR;
        if (energyMultiplier < 0.4) energyMultiplier = 0.4;  // floor
        Serial.print(" | NEAR-MISS! skip="); Serial.print(pumpsToSkip);
        Serial.print(" energy="); Serial.print(energyMultiplier, 2);
      } else {
        // Not a near-miss — gradually restore energy multiplier
        energyMultiplier = min(1.0f, energyMultiplier + 0.05f);
      }
      Serial.println();

      lastPeakReported = peakVelMagnitude;
      peakVelMagnitude = 0;
      minAngleFromVertThisSwing = 360.0;  // reset for next swing
    }

    bool isRotating = (velMag > ROTATION_VEL_THRESHOLD) && (angleFromVert > ROTATION_ANGLE_THRESHOLD);

    // Approach decelerator: if approaching vertical fast, brake to bleed energy
    bool approachingFast = (angleFromVert > DECEL_ANGLE_END) &&
                           (angleFromVert < DECEL_ANGLE_START) &&
                           (velMag > DECEL_VEL_THRESHOLD);
    // Only brake if pendulum is moving TOWARD vertical
    // angle moving toward vert means: if currentAngle < vert, vel > 0; if currentAngle > vert, vel < 0
    bool movingTowardVert = ((currentAngle < vert && angularVelocity > 0) ||
                             (currentAngle > vert && angularVelocity < 0));

    if (isRotating) {
      if (motorActive) {
        motorActive = false;
        motorStop();
      }
      peakVelMagnitude = 0;
    } else if (approachingFast && movingTowardVert) {
      // Brake: drive motor opposite to current motion to bleed energy
      bool brakeDir = (angularVelocity < 0);  // opposite of motion direction
      currentMotorDir = brakeDir;
      motorActive = true;
      runMotorPWM(brakeDir, DECEL_PWM);

      if (now % 200 < 10) {
        Serial.print("BRAKE | ang: "); Serial.print(currentAngle, 1);
        Serial.print(" | vel: "); Serial.println(angularVelocity, 0);
      }
    } else if (angleFromVert > COAST_THRESHOLD_DEG && pumpsToSkip == 0) {
      bool desiredDir = (angularVelocity > 0);
      if (!motorActive || desiredDir != currentMotorDir) {
        currentMotorDir = desiredDir;
        motorActive = true;
        // Decrement skip counter when we actually pump
        if (pumpsToSkip > 0) pumpsToSkip--;
      }

      int pwmScale = map((int)angleFromVert, (int)COAST_THRESHOLD_DEG, 180,
                         SWINGUP_PWM_MIN, SWINGUP_PWM_MAX);
      pwmScale = constrain(pwmScale, SWINGUP_PWM_MIN, SWINGUP_PWM_MAX);
      pwmScale = (int)(pwmScale * energyMultiplier);
      runMotorPWM(currentMotorDir, pwmScale);
    } else {
      if (motorActive) {
        motorActive = false;
        motorStop();
      }
      // Decrement skip counter on the swing where we're skipping
      // (only at the moment we'd normally pump)
      if (pumpsToSkip > 0 && angleFromVert > COAST_THRESHOLD_DEG) {
        // Don't decrement here — only when we actually skip a pump opportunity
        // The pump opportunity is when we'd otherwise enter the pump branch
      }
    }

    // Decrement pumpsToSkip on each new swing cycle (sign change)
    if (signChange && pumpsToSkip > 0) {
      pumpsToSkip--;
    }

    if (now % 200 < 10) {
      Serial.print("SWING | ang: "); Serial.print(currentAngle, 1);
      Serial.print(" | vel: "); Serial.print(angularVelocity, 0);
      Serial.print(" | fromVert: "); Serial.print(angleFromVert, 1);
      Serial.print(" | skip: "); Serial.print(pumpsToSkip);
      Serial.print(" | E: "); Serial.println(energyMultiplier, 2);
    }
  }

  // ===== STABILIZATION =====
  if (shouldBalance) {
    if (!wasInBalanceMode) {
      wasInBalanceMode = true;
      resetPID(currentAngle);
      wheelSpeedEst = 0;
      energyMultiplier = 1.0;
      pumpsToSkip = 0;
      minAngleFromVertThisSwing = 360.0;
      Serial.print(">>> ENTERING BALANCE MODE | entry ang: ");
      Serial.print(currentAngle, 1);
      Serial.print(" | entry vel: ");
      Serial.println(angularVelocity, 0);
    }

    motorActive = false;
    double output = computePID(currentAngle, angularVelocity);
    setMotorPID((int)output);

    float dt = min((float)deltaTime / 1000.0f, 0.05f);
    wheelSpeedEst += (-WHEEL_SPEED_DECAY_RATE * wheelSpeedEst + WHEEL_ACCEL_GAIN * output) * dt;
    wheelSpeedEst = constrain(wheelSpeedEst, -1.0f, 1.0f);
    targetAngle = 180.0 + DESAT_MAX_BIAS_DEG * wheelSpeedEst;

    stabilizedTime += deltaTime;

    // After 1 second of stable balance, consider it a real catch and restore energy
    if (stabilizedTime > 1000 && energyMultiplier < 1.0) {
      energyMultiplier = 1.0;
    }

    if (now % 50 < 10) {
      Serial.print("BAL | ang: "); Serial.print(currentAngle, 1);
      Serial.print(" | vel: "); Serial.print(angularVelocity, 0);
      Serial.print(" | err: "); Serial.print(targetAngle - currentAngle, 1);
      Serial.print(" | out: "); Serial.print((int)output);
      Serial.print(" | whl: "); Serial.println(wheelSpeedEst, 2);
    }
  }
}