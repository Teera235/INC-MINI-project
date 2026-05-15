#include <Wire.h>
#include <Servo.h>
#include <OneWire.h>
#include <math.h>
#include <avr/io.h>
#include "arduinoFFT.h"
#include "Adafruit_VL53L0X.h"

#define PIN_FFT          A0
#define PIN_FAN_SENSOR   A2
#define PIN_LDR_CLOCK    A8
#define PIN_LDR_DATA     A9
#define PIN_LDR_COLOR    A10
#define PIN_LED_KETTLE   A3

#define PIN_PWM_FAN      11
#define PIN_DS18B20      10
#define PIN_RELAY        12
#define PIN_BUZZER       9
#define PIN_LED_CARD     8
#define PIN_LED_SCAN     7
#define PIN_LED_WEIGH    6

#define PIN_MOTOR_ENA    5
#define PIN_MOTOR_IN1    4
#define PIN_MOTOR_IN2    3

#define PIN_SERVO_X      44
#define PIN_SERVO_Y      45

#define CLK1 22
#define DIO1 24
#define CLK2 26
#define DIO2 28
#define CLK3 30
#define DIO3 32

#define RELAY_ON  LOW
#define RELAY_OFF HIGH

const int THRESHOLD_HOLE = 500;
const int THRESHOLD_DATA = 500;
const int THRESHOLD_REMOVE_COLOR = 55;
const int THRESHOLD_CARD_OUT = 900;
const int PATTERN_SIZE = 10;
int CardA[10] = { 0, 1, 0, 1, 0, 1, 0, 1, 1, 1 };
int CardB[10] = { 0, 1, 0, 1, 1, 0, 0, 1, 1, 1 };
int CardC[10] = { 0, 1, 0, 0, 1, 1, 0, 1, 1, 1 };

int pattern[PATTERN_SIZE];
int indexCounter = 0;
long colorSum = 0;
int colorCount = 0;
bool cardInserted = false;
bool lastClockState = HIGH;
unsigned long removalTimer = 0;
char programNum = 0;

const uint16_t SAMPLES = 128;
double vReal[SAMPLES];
double vImag[SAMPLES];
const float ALPHA = 0.15;
float filteredFreq = 0;
bool firstFreq = true;
const unsigned long SAMPLING_INTERVAL_US = 500;

float freqEmpty = 0;
float freqFull  = 0;
const float W_MIN = 200.0;
const float W_MAX = 800.0;

OneWire ds(PIN_DS18B20);
float targetTemp  = 0;
float currentTemp = 0;
const float MARGIN_ON  = 1.0;
const float MARGIN_OFF = 0.3;
unsigned long lastTempRead = 0;
const unsigned long TEMP_INTERVAL = 1000;

const int POLES = 2;
int sensorMax = 0;
int sensorMin = 1023;
unsigned long lastPulseTime = 0;
unsigned long lastPulseReceived = 0;
bool aboveThreshold = false;
float rps = 0;
float rpsBuf[5] = { 0, 0, 0, 0, 0 };
int rpsBufIdx = 0;

float targetRPS = 0;
float kP = 0.3;
float kI = 0.4;
float kD = 0.02;
float errorPrev = 0;
float integral = 0;
float pwmValue = 0;
unsigned long lastPIDTime = 0;

Adafruit_VL53L0X lox;
Servo servoX;
Servo servoY;
float refDist = 0;
float dimW = 0, dimL = 0, dimH = 0;
int currentY = 0;

float measuredWeight = 0;

unsigned long stateStartTime = 0;
unsigned long lastObjectCheck = 0;

uint8_t seg[] = {
  0x3f, 0x06, 0x5b, 0x4f,
  0x66, 0x6d, 0x7d, 0x07,
  0x7f, 0x6f
};
uint8_t SEG_BLANK = 0x00;
uint8_t SEG_DASH  = 0x40;
uint8_t SEG_E     = 0x79;

uint8_t TEXT_LO[4]   = { 0x38, 0x3f, 0x00, 0x00 };
uint8_t TEXT_HI[4]   = { 0x76, 0x06, 0x00, 0x00 };
uint8_t TEXT_GO[4]   = { 0x3d, 0x3f, 0x00, 0x00 };
uint8_t TEXT_DONE[4] = { 0x5e, 0x3f, 0x54, 0x79 };

#define LED_COUNT 3
#define IDX_CARD  0
#define IDX_SCAN  1
#define IDX_WEIGH 2
uint8_t  ledPin[LED_COUNT]      = { PIN_LED_CARD, PIN_LED_SCAN, PIN_LED_WEIGH };
bool     ledEnabled[LED_COUNT]  = { false, false, false };
bool     ledSolidF[LED_COUNT]   = { false, false, false };
unsigned long ledPeriod[LED_COUNT] = { 500, 500, 500 };
unsigned long ledToggle[LED_COUNT] = { 0, 0, 0 };
bool     ledState[LED_COUNT]    = { false, false, false };

enum SystemState {
  STATE_CAL_EMPTY,
  STATE_CAL_FULL,
  STATE_WAIT_CARD,
  STATE_CARD_DISPLAY,
  STATE_CARD_INVALID,
  STATE_WAIT_OBJECT,
  STATE_REF_DIST,
  STATE_SCAN_DIM1,
  STATE_ROTATE_TO_DIM2,
  STATE_SCAN_DIM2,
  STATE_ROTATE_TO_DIM3,
  STATE_SCAN_DIM3,
  STATE_SCAN_DONE,
  STATE_TRANSFER,
  STATE_WEIGH_DONE,
  STATE_HEATING,
  STATE_BOIL_COMPLETE,
  STATE_DONE
};
SystemState state = STATE_CAL_EMPTY;

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
  writeRaw(clk, dio, SEG_BLANK, SEG_BLANK, SEG_BLANK, SEG_DASH);
}

void showE(int clk, int dio) {
  writeRaw(clk, dio, SEG_BLANK, SEG_BLANK, SEG_BLANK, SEG_E);
}

void showDigit(int clk, int dio, int d) {
  if (d < 0 || d > 9) { clearDisplay(clk, dio); return; }
  writeRaw(clk, dio, SEG_BLANK, SEG_BLANK, SEG_BLANK, seg[d]);
}

void showText(int clk, int dio, uint8_t data[]) {
  writeRaw(clk, dio, data[0], data[1], data[2], data[3]);
}

void showInt(int clk, int dio, int val) {
  if (val > 9999) val = 9999;
  if (val < 0) val = 0;
  int d1 = (val / 1000) % 10;
  int d2 = (val / 100) % 10;
  int d3 = (val / 10) % 10;
  int d4 = val % 10;
  uint8_t b1 = d1 > 0 ? seg[d1] : SEG_BLANK;
  uint8_t b2 = (d1 > 0 || d2 > 0) ? seg[d2] : SEG_BLANK;
  uint8_t b3 = (d1 > 0 || d2 > 0 || d3 > 0) ? seg[d3] : SEG_BLANK;
  uint8_t b4 = seg[d4];
  writeRaw(clk, dio, b1, b2, b3, b4);
}

void showFloat1d(int clk, int dio, float val) {
  int v = (int)(val * 10);
  if (v > 9999) v = 9999;
  if (v < 0) v = 0;
  uint8_t b1 = (v / 1000) % 10 > 0 ? seg[(v / 1000) % 10] : SEG_BLANK;
  uint8_t b2 = seg[(v / 100) % 10];
  uint8_t b3 = seg[(v / 10) % 10] | 0x80;
  uint8_t b4 = seg[v % 10];
  writeRaw(clk, dio, b1, b2, b3, b4);
}

void showCM(int clk, int dio, float cm) {
  int v = (int)(cm * 100 + 0.5);
  if (v > 9999) v = 9999;
  if (v < 0) v = 0;
  uint8_t b1 = (v / 1000) % 10 > 0 ? seg[(v / 1000) % 10] : SEG_BLANK;
  uint8_t b2 = seg[(v / 100) % 10] | 0x80;
  uint8_t b3 = seg[(v / 10) % 10];
  uint8_t b4 = seg[v % 10];
  writeRaw(clk, dio, b1, b2, b3, b4);
}

void showTemp(int clk, int dio, float t) {
  int v = (int)(t + 0.5);
  if (v > 999) v = 999;
  if (v < 0) v = 0;
  uint8_t b1 = SEG_BLANK;
  uint8_t b2 = (v / 100) % 10 > 0 ? seg[(v / 100) % 10] : SEG_BLANK;
  uint8_t b3 = ((v / 100) % 10 > 0 || (v / 10) % 10 > 0) ? seg[(v / 10) % 10] : SEG_BLANK;
  uint8_t b4 = seg[v % 10];
  writeRaw(clk, dio, b1, b2, b3, b4);
}

void showCountdown(int clk, int dio, int sec) {
  if (sec < 0) sec = 0;
  if (sec > 99) sec = 99;
  writeRaw(clk, dio, SEG_BLANK, SEG_BLANK, seg[sec / 10], seg[sec % 10]);
}

void showPattern4(int clk, int dio, int *pat, int len) {
  int start = (len > 4) ? len - 4 : 0;
  uint8_t b[4] = { SEG_BLANK, SEG_BLANK, SEG_BLANK, SEG_BLANK };
  for (int i = 0; i < 4 && start + i < len; i++) {
    b[i] = pat[start + i] ? seg[1] : seg[0];
  }
  writeRaw(clk, dio, b[0], b[1], b[2], b[3]);
}

void ledOff(int idx) {
  ledEnabled[idx] = false;
  ledSolidF[idx] = false;
  digitalWrite(ledPin[idx], LOW);
  ledState[idx] = false;
}

void ledSolid(int idx) {
  ledEnabled[idx] = true;
  ledSolidF[idx] = true;
  digitalWrite(ledPin[idx], HIGH);
  ledState[idx] = true;
}

void ledBlink(int idx, float hz) {
  ledEnabled[idx] = true;
  ledSolidF[idx] = false;
  unsigned long p = (unsigned long)(500.0 / hz);
  ledPeriod[idx] = (p < 50) ? 50 : p;
}

void updateLeds(unsigned long now) {
  for (int i = 0; i < LED_COUNT; i++) {
    if (!ledEnabled[i] || ledSolidF[i]) continue;
    if (now - ledToggle[i] >= ledPeriod[i]) {
      ledToggle[i] = now;
      ledState[i] = !ledState[i];
      digitalWrite(ledPin[i], ledState[i] ? HIGH : LOW);
    }
  }
}

void shortBeep(int ms) {
  digitalWrite(PIN_BUZZER, HIGH);
  delay(ms);
  digitalWrite(PIN_BUZZER, LOW);
}

float measureFreq() {
  for (uint16_t i = 0; i < SAMPLES; i++) {
    unsigned long t = micros();
    vReal[i] = (double)analogRead(PIN_FFT);
    vImag[i] = 0.0;
    while ((micros() - t) < SAMPLING_INTERVAL_US);
  }
  float sf = 1000000.0 / (float)SAMPLING_INTERVAL_US;
  double sum = 0;
  for (uint16_t i = 0; i < SAMPLES; i++) sum += vReal[i];
  double dc = sum / SAMPLES;
  for (uint16_t i = 0; i < SAMPLES; i++) vReal[i] -= dc;

  ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, sf);
  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(FFT_FORWARD);
  FFT.complexToMagnitude();

  double pm = 0;
  uint16_t pb = 1;
  for (uint16_t i = 1; i < (SAMPLES / 2); i++) {
    if (vReal[i] > pm) { pm = vReal[i]; pb = i; }
  }
  float freq = (float)pb * sf / (float)SAMPLES;
  if (pb > 1 && pb < (SAMPLES / 2 - 1)) {
    double y0 = vReal[pb - 1], y1 = vReal[pb], y2 = vReal[pb + 1];
    double d = y0 - 2.0 * y1 + y2;
    if (d != 0) freq = ((float)pb + 0.5 * (y0 - y2) / d) * sf / (float)SAMPLES;
  }
  if (firstFreq) { filteredFreq = freq; firstFreq = false; }
  else { filteredFreq = ALPHA * freq + (1.0 - ALPHA) * filteredFreq; }
  return filteredFreq;
}

float freqToWeight(float freq) {
  if (freqEmpty == freqFull) return W_MIN;
  float ratio = (freqEmpty - freq) / (freqEmpty - freqFull);
  if (ratio < 0) ratio = 0;
  if (ratio > 1) ratio = 1;
  return W_MIN + ratio * (W_MAX - W_MIN);
}

bool isObjectOnScale(float freq) {
  return (freqEmpty - freq) > 1.0;
}

float readTemp() {
  byte d[2];
  ds.reset();
  ds.skip();
  ds.write(0x44);
  delay(750);
  ds.reset();
  ds.skip();
  ds.write(0xBE);
  d[0] = ds.read();
  d[1] = ds.read();
  int raw = (d[1] << 8) | d[0];
  return raw / 16.0;
}

void setupPWM10bit() {
  pinMode(PIN_PWM_FAN, OUTPUT);
  TCCR1A = _BV(COM1A1) | _BV(WGM11) | _BV(WGM10);
  TCCR1B = _BV(WGM12) | _BV(CS10);
  OCR1A = 0;
}

void writePWM10bit(int v) {
  OCR1A = constrain(v, 0, 1023);
}

float medianOf5(float a, float b, float c, float d, float e) {
  float arr[5] = { a, b, c, d, e };
  for (int i = 0; i < 4; i++) {
    for (int j = i + 1; j < 5; j++) {
      if (arr[j] < arr[i]) {
        float t = arr[i]; arr[i] = arr[j]; arr[j] = t;
      }
    }
  }
  return arr[2];
}

int feedForwardPWM(float tgt) {
  if (tgt <= 0) return 0;
  if (tgt <= 1.0) return 200;
  if (tgt >= 20.0) return 800;
  return (int)(200.0 + (tgt - 1.0) * (600.0 / 19.0));
}

void updateFanSensor() {
  int val = analogRead(PIN_FAN_SENSOR);
  unsigned long nowMicros = micros();
  if (val > sensorMax) sensorMax = val;
  if (val < sensorMin) sensorMin = val;
  int range = sensorMax - sensorMin;
  if (range < 50) return;
  int threshold = (sensorMax + sensorMin) / 2;
  int hysteresis = range / 6;
  if (!aboveThreshold && val > threshold + hysteresis) {
    aboveThreshold = true;
    if (lastPulseTime > 0) {
      unsigned long interval = nowMicros - lastPulseTime;
      if (interval > 1000) {
        float r = 1000000.0 / (interval * POLES);
        rpsBuf[rpsBufIdx] = r;
        rpsBufIdx = (rpsBufIdx + 1) % 5;
        rps = medianOf5(rpsBuf[0], rpsBuf[1], rpsBuf[2], rpsBuf[3], rpsBuf[4]);
      }
    }
    lastPulseTime = nowMicros;
    lastPulseReceived = nowMicros;
  }
  if (aboveThreshold && val < threshold - hysteresis) aboveThreshold = false;
  if (nowMicros - lastPulseReceived > 500000) rps = 0;
}

void updateFanPID(unsigned long now) {
  if (now - lastPIDTime < 20) return;
  float dt = (now - lastPIDTime) / 1000.0;
  lastPIDTime = now;
  float error = targetRPS - rps;
  integral += error * dt;
  integral = constrain(integral, -100, 100);
  float deriv = (error - errorPrev) / dt;
  errorPrev = error;
  float output = (kP * error) + (kI * integral) + (kD * deriv);
  pwmValue = constrain(pwmValue + output, 0, 900);
  writePWM10bit((int)pwmValue);
}

float weightToTargetRPS(float w) {
  if (w < 200) return 1.0;
  if (w > 800) return 20.0;
  return 1.0 + (w - 200.0) * (19.0 / 600.0);
}

float weightToTargetTemp(float w) {
  if (w < 200) return 35.0;
  if (w > 800) return 50.0;
  return 35.0 + (w - 200.0) * (15.0 / 600.0);
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

bool objectAtScanner() {
  float d = readStable();
  return (d > 0 && fabs(d - refDist) > 2.0);
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
  int step = (targetDeg > currentY) ? 1 : -1;
  while (currentY != targetDeg) {
    currentY += step;
    servoY.writeMicroseconds(map(currentY, 0, 180, 1000, 2000));
    delay(15);
  }
  delay(500);
}

void conveyorRunMs(int speed, int durationMs, bool forward) {
  if (forward) {
    digitalWrite(PIN_MOTOR_IN1, HIGH);
    digitalWrite(PIN_MOTOR_IN2, LOW);
  } else {
    digitalWrite(PIN_MOTOR_IN1, LOW);
    digitalWrite(PIN_MOTOR_IN2, HIGH);
  }
  analogWrite(PIN_MOTOR_ENA, speed);
  unsigned long s = millis();
  while (millis() - s < (unsigned long)durationMs) {
    updateFanSensor();
    updateFanPID(millis());
    delay(2);
  }
  analogWrite(PIN_MOTOR_ENA, 0);
}

bool comparePattern(int a[], int b[]) {
  for (int i = 0; i < PATTERN_SIZE; i++) {
    if (a[i] != b[i]) return false;
  }
  return true;
}

void resetCardState() {
  indexCounter = 0;
  colorSum = 0;
  colorCount = 0;
  cardInserted = false;
  removalTimer = 0;
}

int readCardOnce() {
  int clockRaw = analogRead(PIN_LDR_CLOCK);
  bool currentClock = (clockRaw > THRESHOLD_HOLE) ? HIGH : LOW;

  if (!cardInserted) {
    if (currentClock == LOW) {
      cardInserted = true;
      indexCounter = 0;
      colorSum = 0;
      colorCount = 0;
      ledSolid(IDX_CARD);
      clearDisplay(CLK2, DIO2);
      showInt(CLK3, DIO3, 0);
      Serial.println("CARD_IN");
    }
    lastClockState = currentClock;
    return 0;
  }

  if (cardInserted && indexCounter < PATTERN_SIZE) {
    if (analogRead(PIN_LDR_COLOR) > THRESHOLD_REMOVE_COLOR) {
      if (removalTimer == 0) removalTimer = millis();
      if (millis() - removalTimer > 500) {
        Serial.print("REMOVED_EARLY bits=");
        Serial.println(indexCounter);
        bool partial = (indexCounter > 0);
        int avgColor = (colorCount > 0) ? (int)(colorSum / colorCount) : 0;
        resetCardState();
        lastClockState = currentClock;
        if (partial) {
          if      (avgColor >= 500 && avgColor < 600) programNum = '2';
          else if (avgColor >= 600)                   programNum = '3';
          else if (avgColor < 200)                    programNum = '3';
          else                                        programNum = '1';
          Serial.print("Partial -> ");
          Serial.println(programNum);
          return 1;
        }
        return 0;
      }
    } else {
      removalTimer = 0;
    }
  }

  if (currentClock == HIGH && lastClockState == LOW) {
    int rawData = analogRead(PIN_LDR_DATA);
    pattern[indexCounter] = (rawData > THRESHOLD_DATA) ? 1 : 0;
    int colorValue = analogRead(PIN_LDR_COLOR);
    colorSum += colorValue;
    colorCount++;

    Serial.print("Bit ");
    Serial.print(indexCounter);
    Serial.print(" = ");
    Serial.print(pattern[indexCounter]);
    Serial.print(" color=");
    Serial.println(colorValue);

    indexCounter++;
    showPattern4(CLK2, DIO2, pattern, indexCounter);
    showInt(CLK3, DIO3, indexCounter);
  }
  lastClockState = currentClock;

  if (indexCounter >= PATTERN_SIZE) {
    int result = 1;
    int avgColor = (colorCount > 0) ? (int)(colorSum / colorCount) : 0;

    if      (comparePattern(pattern, CardA)) { programNum = '1'; }
    else if (comparePattern(pattern, CardB)) { programNum = '2'; }
    else if (comparePattern(pattern, CardC)) { programNum = '3'; }
    else {
      if      (avgColor >= 500 && avgColor < 600) programNum = '2';
      else if (avgColor >= 600)                   programNum = '3';
      else if (avgColor < 200)                    programNum = '3';
      else                                        programNum = '1';
    }

    Serial.print("PATTERN=");
    for (int i = 0; i < PATTERN_SIZE; i++) Serial.print(pattern[i]);
    Serial.print(" AvgColor=");
    Serial.print(avgColor);
    Serial.print(" RESULT=");
    Serial.println(programNum);

    while (analogRead(PIN_LDR_CLOCK) < THRESHOLD_CARD_OUT) delay(10);
    resetCardState();
    return result;
  }
  return 0;
}

void setup() {
  Serial.begin(115200);

  pinMode(CLK1, OUTPUT); pinMode(DIO1, OUTPUT);
  pinMode(CLK2, OUTPUT); pinMode(DIO2, OUTPUT);
  pinMode(CLK3, OUTPUT); pinMode(DIO3, OUTPUT);

  pinMode(PIN_RELAY, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_LED_CARD, OUTPUT);
  pinMode(PIN_LED_SCAN, OUTPUT);
  pinMode(PIN_LED_WEIGH, OUTPUT);
  pinMode(PIN_LED_KETTLE, OUTPUT);
  digitalWrite(PIN_RELAY, RELAY_OFF);
  digitalWrite(PIN_LED_KETTLE, LOW);

  pinMode(PIN_MOTOR_ENA, OUTPUT);
  pinMode(PIN_MOTOR_IN1, OUTPUT);
  pinMode(PIN_MOTOR_IN2, OUTPUT);
  analogWrite(PIN_MOTOR_ENA, 0);

  Wire.begin();
  lox.begin();
  servoX.attach(PIN_SERVO_X);
  servoY.attach(PIN_SERVO_Y);
  servoX.write(90);
  servoY.writeMicroseconds(1000);
  currentY = 0;

  setupPWM10bit();

  showText(CLK1, DIO1, TEXT_LO);
  clearDisplay(CLK2, DIO2);
  clearDisplay(CLK3, DIO3);

  Serial.println("=== INC AUTOMATION CELL ===");
  Serial.println("Calibration phase 1: empty 10s");
  shortBeep(100);

  state = STATE_CAL_EMPTY;
  stateStartTime = millis();
}

void loop() {
  unsigned long now = millis();
  updateFanSensor();
  updateLeds(now);

  switch (state) {

    case STATE_CAL_EMPTY: {
      static float sumF = 0;
      static int cntF = 0;
      unsigned long elapsed = now - stateStartTime;
      float f = measureFreq();

      int remaining = 10 - (int)(elapsed / 1000);
      if (remaining < 0) remaining = 0;
      showCountdown(CLK3, DIO3, remaining);
      showFloat1d(CLK2, DIO2, f);

      if (elapsed > 2000) {
        sumF += f;
        cntF++;
      }

      if (elapsed >= 10000) {
        freqEmpty = (cntF > 0) ? sumF / cntF : 451.0;
        Serial.print("FREQ_EMPTY=");
        Serial.println(freqEmpty, 3);
        sumF = 0;
        cntF = 0;
        shortBeep(200);
        delay(150);
        shortBeep(200);
        showText(CLK1, DIO1, TEXT_HI);
        Serial.println("Calibration phase 2: full load 10s");
        state = STATE_CAL_FULL;
        stateStartTime = now;
      }
      break;
    }

    case STATE_CAL_FULL: {
      static float sumF2 = 0;
      static int cntF2 = 0;
      unsigned long elapsed = now - stateStartTime;
      float f = measureFreq();

      int remaining = 10 - (int)(elapsed / 1000);
      if (remaining < 0) remaining = 0;
      showCountdown(CLK3, DIO3, remaining);
      showFloat1d(CLK2, DIO2, f);

      if (elapsed > 2000) {
        sumF2 += f;
        cntF2++;
      }

      if (elapsed >= 10000) {
        freqFull = (cntF2 > 0) ? sumF2 / cntF2 : freqEmpty - 6.0;
        Serial.print("FREQ_FULL=");
        Serial.println(freqFull, 3);
        Serial.print("RANGE=");
        Serial.println(freqEmpty - freqFull, 3);
        sumF2 = 0;
        cntF2 = 0;
        shortBeep(500);
        showText(CLK1, DIO1, TEXT_GO);
        clearDisplay(CLK2, DIO2);
        clearDisplay(CLK3, DIO3);
        delay(1500);
        showDash(CLK1, DIO1);
        ledBlink(IDX_CARD, 2.0);
        Serial.println("Ready for card");
        state = STATE_WAIT_CARD;
        stateStartTime = now;
      }
      break;
    }

    case STATE_WAIT_CARD: {
      int r = readCardOnce();
      if (!cardInserted && !ledSolidF[IDX_CARD]) ledBlink(IDX_CARD, 2.0);

      if (r == 1) {
        if (programNum == '1')      showDigit(CLK1, DIO1, 1);
        else if (programNum == '2') showDigit(CLK1, DIO1, 2);
        else if (programNum == '3') showDigit(CLK1, DIO1, 3);
        shortBeep(150);
        ledBlink(IDX_CARD, 2.0);
        state = STATE_CARD_DISPLAY;
        stateStartTime = now;
      } else if (r == 2) {
        showE(CLK1, DIO1);
        shortBeep(500);
        state = STATE_CARD_INVALID;
        stateStartTime = now;
      }
      break;
    }

    case STATE_CARD_DISPLAY: {
      if (now - stateStartTime >= 2000) {
        ledOff(IDX_CARD);
        ledBlink(IDX_SCAN, 0.25);
        showDash(CLK1, DIO1);
        clearDisplay(CLK2, DIO2);
        clearDisplay(CLK3, DIO3);
        state = STATE_WAIT_OBJECT;
        stateStartTime = now;
      }
      break;
    }

    case STATE_CARD_INVALID: {
      if (now - stateStartTime >= 2000) {
        showDash(CLK1, DIO1);
        clearDisplay(CLK2, DIO2);
        clearDisplay(CLK3, DIO3);
        ledBlink(IDX_CARD, 2.0);
        state = STATE_WAIT_CARD;
        stateStartTime = now;
      }
      break;
    }

    case STATE_WAIT_OBJECT: {
      if (now - lastObjectCheck > 200) {
        lastObjectCheck = now;
        if (refDist <= 0) {
          rotateServoY(0);
          refDist = getReference();
          Serial.print("REF=");
          Serial.println(refDist);
        }
        if (objectAtScanner()) {
          Serial.println("OBJECT_DETECTED");
          ledBlink(IDX_SCAN, 0.5);
          state = STATE_REF_DIST;
          stateStartTime = now;
        }
      }
      break;
    }

    case STATE_REF_DIST: {
      refDist = getReference();
      Serial.print("REF=");
      Serial.println(refDist);
      state = STATE_SCAN_DIM1;
      stateStartTime = now;
      break;
    }

    case STATE_SCAN_DIM1: {
      digitalWrite(PIN_BUZZER, HIGH);
      dimW = scanDimension();
      digitalWrite(PIN_BUZZER, LOW);
      Serial.print("DIM1=");
      Serial.println(dimW);
      showCM(CLK1, DIO1, dimW);
      state = STATE_ROTATE_TO_DIM2;
      stateStartTime = now;
      break;
    }

    case STATE_ROTATE_TO_DIM2: {
      rotateServoY(90);
      refDist = getReference();
      state = STATE_SCAN_DIM2;
      stateStartTime = now;
      break;
    }

    case STATE_SCAN_DIM2: {
      digitalWrite(PIN_BUZZER, HIGH);
      dimL = scanDimension();
      digitalWrite(PIN_BUZZER, LOW);
      Serial.print("DIM2=");
      Serial.println(dimL);
      showCM(CLK2, DIO2, dimL);
      state = STATE_ROTATE_TO_DIM3;
      stateStartTime = now;
      break;
    }

    case STATE_ROTATE_TO_DIM3: {
      rotateServoY(180);
      refDist = getReference();
      state = STATE_SCAN_DIM3;
      stateStartTime = now;
      break;
    }

    case STATE_SCAN_DIM3: {
      digitalWrite(PIN_BUZZER, HIGH);
      dimH = scanDimension();
      digitalWrite(PIN_BUZZER, LOW);
      Serial.print("DIM3=");
      Serial.println(dimH);
      showCM(CLK3, DIO3, dimH);
      state = STATE_SCAN_DONE;
      stateStartTime = now;
      break;
    }

    case STATE_SCAN_DONE: {
      ledSolid(IDX_SCAN);
      shortBeep(150);
      delay(100);
      shortBeep(150);
      Serial.print("FINAL DIMS W=");
      Serial.print(dimW);
      Serial.print(" L=");
      Serial.print(dimL);
      Serial.print(" H=");
      Serial.println(dimH);
      delay(2000);
      state = STATE_TRANSFER;
      stateStartTime = now;
      break;
    }

    case STATE_TRANSFER: {
      ledOff(IDX_SCAN);
      ledBlink(IDX_WEIGH, 1.0);
      conveyorRunMs(120, 4000, true);

      delay(500);
      float f = measureFreq();
      measuredWeight = freqToWeight(f);
      if (measuredWeight < W_MIN) measuredWeight = W_MIN;
      if (measuredWeight > W_MAX) measuredWeight = W_MAX;

      targetTemp = weightToTargetTemp(measuredWeight);
      targetRPS  = weightToTargetRPS(measuredWeight);

      pwmValue = (float)feedForwardPWM(targetRPS);
      writePWM10bit((int)pwmValue);
      integral = 0;
      errorPrev = 0;

      Serial.print("WEIGHED freq=");
      Serial.print(f, 3);
      Serial.print(" w=");
      Serial.print(measuredWeight, 1);
      Serial.print(" tgtT=");
      Serial.print(targetTemp, 1);
      Serial.print(" tgtRPS=");
      Serial.println(targetRPS, 1);

      state = STATE_WEIGH_DONE;
      stateStartTime = now;
      break;
    }

    case STATE_WEIGH_DONE: {
      ledBlink(IDX_WEIGH, 0.25);
      int wr = ((int)(measuredWeight + 5)) / 10 * 10;
      showInt(CLK1, DIO1, wr);
      clearDisplay(CLK2, DIO2);
      clearDisplay(CLK3, DIO3);

      shortBeep(150); delay(150);
      shortBeep(150); delay(150);
      shortBeep(150);

      delay(1500);

      digitalWrite(PIN_LED_KETTLE, HIGH);
      digitalWrite(PIN_RELAY, RELAY_ON);
      lastTempRead = 0;
      Serial.println("HEATING START");
      state = STATE_HEATING;
      stateStartTime = now;
      break;
    }

    case STATE_HEATING: {
      updateFanPID(now);
      if (now - lastTempRead >= TEMP_INTERVAL) {
        lastTempRead = now;
        currentTemp = readTemp();

        if (currentTemp < targetTemp - MARGIN_ON) {
          digitalWrite(PIN_RELAY, RELAY_ON);
          digitalWrite(PIN_LED_KETTLE, HIGH);
        } else if (currentTemp >= targetTemp - MARGIN_OFF) {
          digitalWrite(PIN_RELAY, RELAY_OFF);
          digitalWrite(PIN_LED_KETTLE, LOW);
        }

        int wr = ((int)(measuredWeight + 5)) / 10 * 10;
        int rpm = (int)(rps * 60.0 + 0.5);
        showInt(CLK1, DIO1, wr);
        showTemp(CLK2, DIO2, currentTemp);
        showInt(CLK3, DIO3, rpm);

        Serial.print("T=");
        Serial.print(currentTemp, 2);
        Serial.print(" TGT=");
        Serial.print(targetTemp, 2);
        Serial.print(" RPS=");
        Serial.println(rps, 2);

        if (currentTemp >= targetTemp) {
          digitalWrite(PIN_RELAY, RELAY_OFF);
          digitalWrite(PIN_LED_KETTLE, LOW);
          digitalWrite(PIN_BUZZER, HIGH);
          delay(3000);
          digitalWrite(PIN_BUZZER, LOW);
          state = STATE_BOIL_COMPLETE;
          stateStartTime = now;
        }
      }
      break;
    }

    case STATE_BOIL_COMPLETE: {
      updateFanPID(now);
      currentTemp = readTemp();
      int wr = ((int)(measuredWeight + 5)) / 10 * 10;
      int rpm = (int)(rps * 60.0 + 0.5);
      showInt(CLK1, DIO1, wr);
      showTemp(CLK2, DIO2, currentTemp);
      showInt(CLK3, DIO3, rpm);

      if (now - stateStartTime >= 2000) {
        shortBeep(150); delay(100);
        shortBeep(150); delay(100);
        shortBeep(500);
        state = STATE_DONE;
        stateStartTime = now;
      }
      break;
    }

    case STATE_DONE: {
      Serial.print("FINAL W=");
      Serial.print(measuredWeight);
      Serial.print(" T=");
      Serial.print(currentTemp);
      Serial.print(" PROG=");
      Serial.println(programNum);

      showText(CLK1, DIO1, TEXT_DONE);
      showText(CLK2, DIO2, TEXT_DONE);
      showText(CLK3, DIO3, TEXT_DONE);

      delay(5000);

      ledOff(IDX_CARD);
      ledOff(IDX_SCAN);
      ledOff(IDX_WEIGH);
      digitalWrite(PIN_LED_KETTLE, LOW);
      digitalWrite(PIN_BUZZER, LOW);

      programNum = 0;
      dimW = dimL = dimH = 0;
      measuredWeight = 0;
      currentTemp = 0;
      targetTemp = 0;
      targetRPS = 0;
      pwmValue = 0;
      writePWM10bit(0);
      refDist = 0;
      indexCounter = 0;
      cardInserted = false;
      rotateServoY(0);

      showDash(CLK1, DIO1);
      clearDisplay(CLK2, DIO2);
      clearDisplay(CLK3, DIO3);
      ledBlink(IDX_CARD, 2.0);
      state = STATE_WAIT_CARD;
      stateStartTime = millis();
      break;
    }

    default: break;
  }

  delay(2);
}
