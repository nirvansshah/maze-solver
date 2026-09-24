// Board: Arduino Nano R4
// Driver: TB6612FNG | Encoders: D2 (Right), D3 (Left)
// VL53L0X XSHUT: A0, A1, A2, A3, 11 -> I2C addrs 0x30..0x34

#include <Wire.h>
#include <Arduino.h>
#include "Adafruit_VL53L0X.h"

// ---------------- Pins ----------------
const int R_IN1 = 7, R_IN2 = 8, R_PWM = 10;
const int L_IN1 = 6, L_IN2 = 5, L_PWM = 9;

const int R_ENC = 2;      // Right encoder
const int L_ENC = 3;      // Left encoder

// ---------------- Sensors ----------------
#define NUM_SENS 5
const int     XSHUT[NUM_SENS] = {A0, A1, A2, A3, 11};
const uint8_t ADDR[NUM_SENS]  = {0x30, 0x31, 0x32, 0x33, 0x34};
Adafruit_VL53L0X tof[NUM_SENS];

// Physical mapping
// Front <- 2, Left <- 4, Right <- 0, L45 <- 3, R45 <- 1
const uint8_t MAP_FRONT = 2;
const uint8_t MAP_LEFT  = 4;
const uint8_t MAP_RIGHT = 0;
const uint8_t MAP_L45   = 3;
const uint8_t MAP_R45   = 1;

// ---------------- Motion tuning ----------------
int  FWD_PWM        = 110;
int  TURN_PWM       = 80;
int  TRIM           = 20;
long TICKS_PER_CELL = 160;
long TICKS_TURN_90  = 65;
int  BRAKE_PWM      = 20;
int  BRAKE_MS       = 20;

// Stop a movement if the encoder is not responding.
const unsigned long MOVE_TIMEOUT_MS = 2500;
const unsigned long TURN_TIMEOUT_MS = 1800;

// ---------------- Thresholds (mm) ----------------
int TH_LEFT_NEAR   = 130;
int TH_RIGHT_NEAR  = 130;
int TH_FRONT_BLOCK = 180;

// ---------------- Encoders ----------------
volatile long rt = 0, lt = 0;

void isrR() { rt++; }
void isrL() { lt++; }

inline void resetTicks() {
  noInterrupts();
  rt = 0;
  lt = 0;
  interrupts();
}

inline long avgTicks() {
  noInterrupts();
  long a = rt;
  long b = lt;
  interrupts();
  return (a + b) / 2;
}

inline long avgTurnTicks() {
  noInterrupts();
  long a = rt;
  long b = lt;
  interrupts();
  return (labs(a) + labs(b)) / 2;
}

// ---------------- Motors ----------------
void drive(int l, int r) {
  l = constrain(l, -255, 255);
  r = constrain(r, -255, 255);

  if (l >= 0) {
    digitalWrite(L_IN1, HIGH);
    digitalWrite(L_IN2, LOW);
    analogWrite(L_PWM, l);
  } else {
    digitalWrite(L_IN1, LOW);
    digitalWrite(L_IN2, HIGH);
    analogWrite(L_PWM, -l);
  }

  if (r >= 0) {
    digitalWrite(R_IN1, HIGH);
    digitalWrite(R_IN2, LOW);
    analogWrite(R_PWM, r);
  } else {
    digitalWrite(R_IN1, LOW);
    digitalWrite(R_IN2, HIGH);
    analogWrite(R_PWM, -r);
  }
}

void stopAll() {
  analogWrite(L_PWM, 0);
  analogWrite(R_PWM, 0);
}

// ---------------- Movement primitives ----------------
void forwardOneCell() {
  resetTicks();
  unsigned long start = millis();

  while (avgTicks() < TICKS_PER_CELL) {
    if (millis() - start > MOVE_TIMEOUT_MS) {
      break;
    }

    noInterrupts();
    long rightTicks = rt;
    long leftTicks  = lt;
    interrupts();

    long d = rightTicks - leftTicks;
    int l = FWD_PWM;
    int r = FWD_PWM;

    // Give the slower side a small nudge.
    if (d > 2) l += TRIM;
    if (d < -2) r += TRIM;

    drive(l, r);
  }

  stopAll();
  delay(100);

  Serial.print("Forward Done - R Ticks: ");
  Serial.print(rt);
  Serial.print(" L Ticks: ");
  Serial.println(lt);
}

void turnLeft90() {
  resetTicks();
  unsigned long start = millis();

  while (avgTurnTicks() < TICKS_TURN_90) {
    if (millis() - start > TURN_TIMEOUT_MS) {
      break;
    }

    noInterrupts();
    long rightTicks = rt;
    long leftTicks  = lt;
    interrupts();

    long d = labs(rightTicks) - labs(leftTicks);
    int l = -TURN_PWM;
    int r = TURN_PWM;

    // Slow the side that is getting ahead.
    if (d > 2) r -= 10;
    if (d < -2) l += 10;

    drive(l, r);
  }

  drive(BRAKE_PWM, -BRAKE_PWM);
  delay(BRAKE_MS);
  stopAll();
  delay(200);

  Serial.print("Left Turn - R Ticks: ");
  Serial.print(rt);
  Serial.print(" L Ticks: ");
  Serial.println(lt);
}

void turnRight90() {
  resetTicks();
  unsigned long start = millis();

  while (avgTurnTicks() < TICKS_TURN_90) {
    if (millis() - start > TURN_TIMEOUT_MS) {
      break;
    }

    noInterrupts();
    long rightTicks = rt;
    long leftTicks  = lt;
    interrupts();

    long d = labs(rightTicks) - labs(leftTicks);
    int l = TURN_PWM;
    int r = -TURN_PWM;

    // Slow the side that is getting ahead.
    if (d > 2) r += 10;
    if (d < -2) l -= 10;

    drive(l, r);
  }

  drive(-BRAKE_PWM, BRAKE_PWM);
  delay(BRAKE_MS);
  stopAll();
  delay(200);

  Serial.print("Right Turn - R Ticks: ");
  Serial.print(rt);
  Serial.print(" L Ticks: ");
  Serial.println(lt);
}

void uTurn() {
  turnLeft90();
  turnLeft90();
  Serial.println("U-Turn Done");
}

// ---------------- Robust sensor bring-up ----------------
static bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

bool setupVL53L0X() {
  // Hold all sensors in reset first.
  for (int i = 0; i < NUM_SENS; i++) {
    pinMode(XSHUT[i], OUTPUT);
    digitalWrite(XSHUT[i], LOW);
  }
  delay(50);

  if (i2cPresent(0x29)) {
    Serial.println(F("ERROR: 0x29 present while all XSHUT LOW. Check XSHUT wiring."));
    return false;
  }

  for (int i = 0; i < NUM_SENS; i++) {
    digitalWrite(XSHUT[i], HIGH);
    delay(100);

    if (!tof[i].begin(0x29, &Wire)) {
      digitalWrite(XSHUT[i], LOW);
      delay(60);
      digitalWrite(XSHUT[i], HIGH);
      delay(120);

      if (!tof[i].begin(0x29, &Wire)) {
        Serial.print(F("Sensor "));
        Serial.print(i);
        Serial.println(F(" boot FAIL"));
        return false;
      }
    }

    tof[i].setAddress(ADDR[i]);
    tof[i].setMeasurementTimingBudgetMicroSeconds(20000);

    Serial.print(F("S"));
    Serial.print(i);
    Serial.print(F(" @0x"));
    Serial.println(ADDR[i], HEX);
  }

  return true;
}

// ---------------- Sensor reading ----------------
bool readAll(uint16_t d[NUM_SENS]) {
  static uint16_t last[NUM_SENS] = {2000, 2000, 2000, 2000, 2000};

  for (int i = 0; i < NUM_SENS; i++) {
    VL53L0X_RangingMeasurementData_t m;
    tof[i].rangingTest(&m, false);

    if (m.RangeStatus == 0) {
      uint16_t reading = (m.RangeMilliMeter > 2000) ? 2000 : m.RangeMilliMeter;
      int change = abs((int)reading - (int)last[i]);

      if (change < 500 || last[i] == 2000) {
        d[i] = reading;
        last[i] = reading;
      } else {
        d[i] = last[i];
      }
    } else {
      d[i] = 2000;
      last[i] = 2000;
    }
  }

  return true;
}

// ---------------- Setup / Loop ----------------
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);

  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);
  pinMode(R_PWM, OUTPUT);

  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  pinMode(L_PWM, OUTPUT);

  pinMode(R_ENC, INPUT_PULLUP);
  pinMode(L_ENC, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(R_ENC), isrR, RISING);
  attachInterrupt(digitalPinToInterrupt(L_ENC), isrL, RISING);

  stopAll();

  if (!setupVL53L0X()) {
    Serial.println(F("Fix sensor wiring and reboot."));
    while (1) {
      delay(1000);
    }
  }

  Wire.setClock(400000);

  Serial.println(F("== Left-Hand Maze Logic =="));
  Serial.println(F("Mapping: FRONT<-2  LEFT<-4  RIGHT<-0  L45<-3  R45<-1"));
  Serial.print(F("Thresh(mm) L/R="));
  Serial.print(TH_LEFT_NEAR);
  Serial.print("/");
  Serial.print(TH_RIGHT_NEAR);
  Serial.print(F(" FRONT="));
  Serial.println(TH_FRONT_BLOCK);
  Serial.println(F("Calibrate TICKS_TURN_90: Do 4x 90 degree turns, divide total ticks by 4"));
}

void loop() {
  uint16_t raw[NUM_SENS];
  if (!readAll(raw)) return;

  // Apply mapping.
  uint16_t FRONT = raw[MAP_FRONT];
  uint16_t LEFT  = raw[MAP_LEFT];
  uint16_t RIGHT = raw[MAP_RIGHT];
  uint16_t L45   = raw[MAP_L45];
  uint16_t R45   = raw[MAP_R45];

  // Use the closer of the two readings on each side.
  uint16_t LEFT_FUSED  = min(LEFT, L45);
  uint16_t RIGHT_FUSED = min(RIGHT, R45);

  bool leftWall  = (LEFT_FUSED < TH_LEFT_NEAR);
  bool rightWall = (RIGHT_FUSED < TH_RIGHT_NEAR);
  bool frontBlock = (FRONT < TH_FRONT_BLOCK);

  // Debug prints.
  Serial.print("Front:"); Serial.print(FRONT);
  Serial.print(" Left:"); Serial.print(LEFT);
  Serial.print(" Right:"); Serial.print(RIGHT);
  Serial.print(" L45:"); Serial.print(L45);
  Serial.print(" R45:"); Serial.print(R45);
  Serial.print(" | Fused L/R:"); Serial.print(LEFT_FUSED);
  Serial.print("/"); Serial.print(RIGHT_FUSED);
  Serial.print(" | Flags -> L:"); Serial.print(leftWall);
  Serial.print(" R:"); Serial.print(rightWall);
  Serial.print(" F:"); Serial.println(frontBlock);

  // Left-hand rule: left, forward, right, U-turn.
  if (!leftWall) {
    Serial.println("Action: Turn Left");
    turnLeft90();
    forwardOneCell();
  }
  else if (!frontBlock) {
    Serial.println("Action: Move Forward");
    forwardOneCell();
  }
  else if (!rightWall) {
    Serial.println("Action: Turn Right");
    turnRight90();
    forwardOneCell();
  }
  else {
    Serial.println("Action: U-Turn");
    uTurn();
    forwardOneCell();
  }

  delay(100);
}
