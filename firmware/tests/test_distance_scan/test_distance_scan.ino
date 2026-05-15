#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include <Servo.h>

#define PIN_SERVO_X 44
#define PIN_SERVO_Y 45
#define PIN_LED1    8
#define PIN_LED2    7

#define CLK1 22
#define DIO1 24
#define CLK2 26
#define DIO2 28
#define CLK3 30
#define DIO3 32

Adafruit_VL53L0X lox;
Servo servoX;
Servo servoY;

float refDist = 0;
int state = 0;

uint8_t seg[] = {
  0x3f, 0x06, 0x5b, 0x4f,
  0x66, 0x6d, 0x7d, 0x07,
  0x7f, 0x6f
};
uint8_t SEG_BLANK = 0x00;
uint8_t SEG_DASH  = 0x40;

void startTM(int clk, int dio) {
  pinMode(dio, OUTPUT);
  digitalWrite(dio, HIGH);
  digitalWrite(clk, HIGH);
  delayMicroseconds(5);
  digitalWrite(dio, LOW);
}

void stopTM(int clk, int dio) {
  digitalWrite(clk, LOW);
  digitalWrite(dio, LOW);
  delayMicroseconds(5);
  digitalWrite(clk, HIGH);
  delayMicroseconds(5);
  digitalWrite(dio, HIGH);
}

void writeByte(int clk, int dio, uint8_t b) {
  for (int i = 0; i < 8; i++) {
    digitalWrite(clk, LOW);
    delayMicroseconds(5);
    digitalWrite(dio, (b >> i) & 1);
    delayMicroseconds(5);
    digitalWrite(clk, HIGH);
    delayMicroseconds(5);
  }
  digitalWrite(clk, LOW);
  pinMode(dio, INPUT);
  delayMicroseconds(5);
  digitalWrite(clk, HIGH);
  delayMicroseconds(5);
  pinMode(dio, OUTPUT);
}

void writeRaw(int clk, int dio, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4) {
  startTM(clk, dio); writeByte(clk, dio, 0x40); stopTM(clk, dio);
  startTM(clk, dio); writeByte(clk, dio, 0xC0);
  writeByte(clk, dio, d1);
  writeByte(clk, dio, d2);
  writeByte(clk, dio, d3);
  writeByte(clk, dio, d4);
  stopTM(clk, dio);
  startTM(clk, dio); writeByte(clk, dio, 0x88 | 7); stopTM(clk, dio);
}

void clearDisplay(int clk, int dio) {
  writeRaw(clk, dio, SEG_BLANK, SEG_BLANK, SEG_BLANK, SEG_BLANK);
}

void showDash(int clk, int dio) {
  writeRaw(clk, dio, SEG_DASH, SEG_DASH, SEG_DASH, SEG_DASH);
}

void showCM(int clk, int dio, float cm) {
  int v = (int)(cm * 100 + 0.5);
  if (v > 9999) v = 9999;
  if (v < 0) v = 0;
  int d1 = (v / 1000) % 10;
  int d2 = (v / 100) % 10;
  int d3 = (v / 10) % 10;
  int d4 = v % 10;
  uint8_t b1 = d1 > 0 ? seg[d1] : SEG_BLANK;
  uint8_t b2 = seg[d2] | 0x80;
  uint8_t b3 = seg[d3];
  uint8_t b4 = seg[d4];
  writeRaw(clk, dio, b1, b2, b3, b4);
}

void showDist(int clk, int dio, float cm) {
  int v = (int)(cm * 10 + 0.5);
  if (v > 9999) v = 9999;
  if (v < 0) v = 0;
  int d1 = (v / 1000) % 10;
  int d2 = (v / 100) % 10;
  int d3 = (v / 10) % 10;
  int d4 = v % 10;
  uint8_t b1 = d1 > 0 ? seg[d1] : SEG_BLANK;
  uint8_t b2 = (d1 > 0 || d2 > 0) ? seg[d2] : SEG_BLANK;
  uint8_t b3 = seg[d3] | 0x80;
  uint8_t b4 = seg[d4];
  writeRaw(clk, dio, b1, b2, b3, b4);
}

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
  pinMode(CLK1, OUTPUT); pinMode(DIO1, OUTPUT);
  pinMode(CLK2, OUTPUT); pinMode(DIO2, OUTPUT);
  pinMode(CLK3, OUTPUT); pinMode(DIO3, OUTPUT);

  if (!lox.begin()) {
    Serial.println("VL53L0X FAIL");
    while (1);
  }
  servoX.attach(PIN_SERVO_X);
  servoY.attach(PIN_SERVO_Y);
  servoX.write(90);
  servoY.writeMicroseconds(1000);
  delay(1000);

  showDash(CLK1, DIO1);
  showDash(CLK2, DIO2);
  showDash(CLK3, DIO3);

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
    clearDisplay(CLK1, DIO1);
    showDist(CLK2, DIO2, refDist);
    clearDisplay(CLK3, DIO3);
    state = 1;
  }
  else if (state == 1) {
    float d = readStable();
    showDist(CLK2, DIO2, d > 0 ? d : 0);
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
    showDash(CLK1, DIO1);
    float dim1 = scanDimension();
    Serial.print("DIM1 = ");
    Serial.println(dim1, 2);
    showCM(CLK1, DIO1, dim1);

    Serial.println("Rotating Y to 90 deg...");
    rotateServoY(90);
    refDist = getReference();
    Serial.println("Scanning DIM 2...");
    showDash(CLK2, DIO2);
    float dim2 = scanDimension();
    Serial.print("DIM2 = ");
    Serial.println(dim2, 2);
    showCM(CLK2, DIO2, dim2);

    Serial.println("Rotating Y to 180 deg...");
    rotateServoY(180);
    refDist = getReference();
    Serial.println("Scanning DIM 3...");
    showDash(CLK3, DIO3);
    float dim3 = scanDimension();
    Serial.print("DIM3 = ");
    Serial.println(dim3, 2);
    showCM(CLK3, DIO3, dim3);

    Serial.println("===== RESULT =====");
    Serial.print("W = "); Serial.print(dim1, 2); Serial.println(" cm");
    Serial.print("L = "); Serial.print(dim2, 2); Serial.println(" cm");
    Serial.print("H = "); Serial.print(dim3, 2); Serial.println(" cm");

    digitalWrite(PIN_LED2, LOW);
    delay(5000);
    Serial.println("Reset");
    rotateServoY(0);
    clearDisplay(CLK1, DIO1);
    clearDisplay(CLK2, DIO2);
    clearDisplay(CLK3, DIO3);
    state = 0;
  }
}
