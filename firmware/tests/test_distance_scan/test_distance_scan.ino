#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include <Servo.h>

#define PIN_SERVO_X 44
#define PIN_SERVO_Y 45
#define PIN_LED1    8
#define PIN_LED2    7

Adafruit_VL53L0X lox;
Servo servoX;
Servo servoY;

float refDist = 0;
int state = 0;

float readCM() {
  VL53L0X_RangingMeasurementData_t m;
  lox.rangingTest(&m, false);
  if (m.RangeStatus != 4) return m.RangeMilliMeter / 10.0;
  return -1;
}

float readStable() {
  float sum = 0;
  int c = 0;
  for (int i = 0; i < 5; i++) {
    float d = readCM();
    if (d > 0) { sum += d; c++; }
    delay(10);
  }
  return (c > 0) ? sum / c : -1;
}

float getReference() {
  float sum = 0;
  int c = 0;
  for (int i = 0; i < 10; i++) {
    float d = readStable();
    if (d > 0) { sum += d; c++; }
    delay(50);
  }
  return (c > 0) ? sum / c : -1;
}

float scanDimension() {
  float maxDim = 0;
  for (int angle = 60; angle <= 120; angle++) {
    servoX.write(angle);
    delay(40);
    float d = readStable();
    if (d > 0) {
      float h = refDist - d;
      if (h > maxDim) maxDim = h;
    }
  }
  for (int angle = 120; angle >= 60; angle--) {
    servoX.write(angle);
    delay(40);
    float d = readStable();
    if (d > 0) {
      float h = refDist - d;
      if (h > maxDim) maxDim = h;
    }
  }
  servoX.write(90);
  return maxDim;
}

void rotateServoY(int targetDeg) {
  static int currentDeg = 0;
  int step = (targetDeg > currentDeg) ? 1 : -1;
  while (currentDeg != targetDeg) {
    currentDeg += step;
    servoY.writeMicroseconds(map(currentDeg, 0, 180, 1000, 2000));
    delay(15);
  }
  delay(500);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);
  if (!lox.begin()) {
    Serial.println("VL53L0X FAIL");
    while (1);
  }
  servoX.attach(PIN_SERVO_X);
  servoY.attach(PIN_SERVO_Y);
  servoX.write(90);
  servoY.writeMicroseconds(1000);
  delay(1000);
  Serial.println("=== DISTANCE SCAN TEST ===");
  Serial.println("Place box on platform after REF is shown.");
}

void loop() {
  if (state == 0) {
    digitalWrite(PIN_LED1, HIGH);
    digitalWrite(PIN_LED2, LOW);
    refDist = getReference();
    Serial.print("REF DIST = ");
    Serial.println(refDist, 2);
    state = 1;
  }
  else if (state == 1) {
    float d = readStable();
    Serial.print("dist=");
    Serial.print(d, 2);
    Serial.print(" delta=");
    Serial.println(d > 0 ? fabs(d - refDist) : -1, 2);
    if (d > 0 && fabs(d - refDist) > 2.0) {
      Serial.println("OBJECT DETECTED");
      digitalWrite(PIN_LED1, LOW);
      digitalWrite(PIN_LED2, HIGH);
      state = 2;
    }
    delay(200);
  }
  else if (state == 2) {
    Serial.println("Scanning DIM 1 (Y at 0 deg)...");
    float dim1 = scanDimension();
    Serial.print("DIM1 = ");
    Serial.println(dim1, 2);

    Serial.println("Rotating Y to 90 deg...");
    rotateServoY(90);
    refDist = getReference();
    Serial.println("Scanning DIM 2...");
    float dim2 = scanDimension();
    Serial.print("DIM2 = ");
    Serial.println(dim2, 2);

    Serial.println("Rotating Y to 180 deg...");
    rotateServoY(180);
    refDist = getReference();
    Serial.println("Scanning DIM 3...");
    float dim3 = scanDimension();
    Serial.print("DIM3 = ");
    Serial.println(dim3, 2);

    Serial.println("===== RESULT =====");
    Serial.print("W = "); Serial.print(dim1, 2); Serial.println(" cm");
    Serial.print("L = "); Serial.print(dim2, 2); Serial.println(" cm");
    Serial.print("H = "); Serial.print(dim3, 2); Serial.println(" cm");

    digitalWrite(PIN_LED2, LOW);
    delay(2000);
    Serial.println("Reset");
    rotateServoY(0);
    state = 0;
  }
}
