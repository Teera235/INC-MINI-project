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

const uint16_t SAMPLES = 128;
double vReal[SAMPLES];
double vImag[SAMPLES];
const float ALPHA = 0.15;
float filteredFreq = 0;
bool firstFreq = true;
const unsigned long SAMPLING_INTERVAL_US = 500;

const float MODEL_A = 0.0003768;
const float MODEL_B = -0.002523;
const float MODEL_C = 451.32;

float zeroFreq = 0;
float zeroOffset = 0;
const float SCALE_PRESENCE_DELTA = 2.0;

const int POLES = 2;
float targetRPS = 1.0;
int sensorMax = 0;
int sensorMin = 1023;
bool fanCalibrated = false;
unsigned long fanCalibStart = 0;
const unsigned long FAN_CALIB_TIME = 3000;
unsigned long lastPulseTime = 0;
unsigned long lastPulseReceived = 0;
bool aboveThreshold = false;
float rps = 0;
float kP = 0.5;
float kI = 1.0;
float kD = 0.05;
float errorPrev = 0;
float integral = 0;
float pwmValue = 200.0;
unsigned long lastPIDTime = 0;

OneWire ds(PIN_DS18B20);
float targetTemp = 0;
float currentTemp = 0;
float marginOn = 1.0;
float marginOff = 0.3;
unsigned long lastTempRead = 0;
const unsigned long TEMP_INTERVAL = 1000;

Adafruit_VL53L0X lox;
Servo servoX;
Servo servoY;
float refDist = 0;
float dimW = 0, dimL = 0, dimH = 0;
bool sensorLock = false;

const int THRESHOLD_HOLE = 500;
const int THRESHOLD_DATA = 200;
const int THRESHOLD_REMOVE_COLOR = 55;
const int THRESHOLD_CARD_OUT = 900;
const int PATTERN_SIZE = 10;
int CardA[10] = { 0, 1, 0, 1, 0, 1, 0, 1, 1, 1 };
int CardB[10] = { 0, 1, 0, 1, 1, 0, 0, 1, 1, 1 };
int CardC[10] = { 0, 1, 0, 0, 1, 1, 0, 1, 1, 1 };
int pattern[PATTERN_SIZE];
int patternRev[PATTERN_SIZE];
int indexCounter = 0;
long colorSum = 0;
int colorCount = 0;
bool cardInserted = false;
bool lastClockState = HIGH;
unsigned long removalTimer = 0;
char programNum = 0;
bool programValid = false;

float measuredWeight = 0;
unsigned long weightStableStart = 0;
const unsigned long WEIGHT_STABLE_TIME = 3000;
float lastWeight = 0;
const float WEIGHT_THRESHOLD = 10.0;
const float MIN_WEIGHT = 20.0;

unsigned long stateStartTime = 0;
unsigned long lastObjectCheck = 0;
unsigned long lastBuzzerToggle = 0;
bool buzzerState = false;
unsigned long invalidShowStart = 0;
int beepCount = 0;
unsigned long lastBeepTime = 0;
bool inLongBeep = false;
unsigned long longBeepStart = 0;
int dimCycleIdx = 0;
unsigned long lastDimCycle = 0;

enum SystemState {
  STATE_FAN_CALIB,
  STATE_WEIGHT_ZERO_CALIB,
  STATE_WAIT_CARD,
  STATE_CARD_VALID_DISPLAY,
  STATE_CARD_INVALID_DISPLAY,
  STATE_WAIT_OBJECT,
  STATE_REF_DIST,
  STATE_SCAN_DIM1,
  STATE_ROTATE_TO_DIM2,
  STATE_SCAN_DIM2,
  STATE_ROTATE_TO_DIM3,
  STATE_SCAN_DIM3,
  STATE_SCAN_DONE,
  STATE_TRANSFER_TO_SCALE,
  STATE_WAIT_ON_SCALE,
  STATE_WEIGHING,
  STATE_WEIGH_DONE,
  STATE_HEATING,
  STATE_BOIL_COMPLETE,
  STATE_DONE
};
SystemState state = STATE_FAN_CALIB;

uint8_t seg[] = {
  0x3f, 0x06, 0x5b, 0x4f,
  0x66, 0x6d, 0x7d, 0x07,
  0x7f, 0x6f
};
uint8_t SEG_DASH = 0x40;
uint8_t SEG_E    = 0x79;
uint8_t SEG_BLANK = 0x00;

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

void showDigit(int clk, int dio, int d) {
  if (d < 0 || d > 9) {
    clearDisplay(clk, dio);
    return;
  }
  writeRaw(clk, dio, SEG_BLANK, SEG_BLANK, SEG_BLANK, seg[d]);
}

void showDash(int clk, int dio) {
  writeRaw(clk, dio, SEG_BLANK, SEG_BLANK, SEG_BLANK, SEG_DASH);
}

void showE(int clk, int dio) {
  writeRaw(clk, dio, SEG_BLANK, SEG_BLANK, SEG_BLANK, SEG_E);
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

void showCM_1mm(int clk, int dio, float cm) {
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

void showTemp(int clk, int dio, float t) {
  int v = (int)(t + 0.5);
  if (v > 999) v = 999;
  if (v < 0) v = 0;
  int d1 = (v / 100) % 10;
  int d2 = (v / 10) % 10;
  int d3 = v % 10;
  uint8_t b1 = SEG_BLANK;
  uint8_t b2 = d1 > 0 ? seg[d1] : SEG_BLANK;
  uint8_t b3 = (d1 > 0 || d2 > 0) ? seg[d2] : SEG_BLANK;
  uint8_t b4 = seg[d3];
  writeRaw(clk, dio, b1, b2, b3, b4);
}

struct LedBlinker {
  uint8_t pin;
  bool enabled;
  bool solid;
  unsigned long periodMs;
  unsigned long lastToggle;
  bool state;
};

LedBlinker ledCard  = { PIN_LED_CARD,  false, false, 500, 0, false };
LedBlinker ledScan  = { PIN_LED_SCAN,  false, false, 500, 0, false };
LedBlinker ledWeigh = { PIN_LED_WEIGH, false, false, 500, 0, false };

void ledOff(LedBlinker &b) {
  b.enabled = false;
  b.solid = false;
  digitalWrite(b.pin, LOW);
  b.state = false;
}

void ledSolid(LedBlinker &b) {
  b.enabled = true;
  b.solid = true;
  digitalWrite(b.pin, HIGH);
  b.state = true;
}

void ledBlink(LedBlinker &b, float hz) {
  b.enabled = true;
  b.solid = false;
  b.periodMs = (unsigned long)(500.0 / hz);
  if (b.periodMs < 50) b.periodMs = 50;
}

void updateLed(LedBlinker &b, unsigned long now) {
  if (!b.enabled) return;
  if (b.solid) return;
  if (now - b.lastToggle >= b.periodMs) {
    b.lastToggle = now;
    b.state = !b.state;
    digitalWrite(b.pin, b.state ? HIGH : LOW);
  }
}

void updateAllLeds(unsigned long now) {
  updateLed(ledCard, now);
  updateLed(ledScan, now);
  updateLed(ledWeigh, now);
}

void buzzerBlink(unsigned long now, unsigned long periodMs) {
  if (now - lastBuzzerToggle >= periodMs) {
    lastBuzzerToggle = now;
    buzzerState = !buzzerState;
    digitalWrite(PIN_BUZZER, buzzerState ? HIGH : LOW);
  }
}

void buzzerOff() {
  digitalWrite(PIN_BUZZER, LOW);
  buzzerState = false;
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
  float corrected = freq - zeroOffset;
  float discriminant = MODEL_B * MODEL_B - 4.0 * MODEL_A * (MODEL_C - corrected);
  if (discriminant < 0) return 0;
  float w = (-MODEL_B + sqrt(discriminant)) / (2.0 * MODEL_A);
  if (w < 0) return 0;
  return w;
}

bool isObjectOnScale(float freq) {
  return fabs(freq - zeroFreq) > SCALE_PRESENCE_DELTA;
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
  OCR1A = 200;
}

void writePWM10bit(int value) {
  OCR1A = constrain(value, 0, 1023);
}

void updateFanPID(unsigned long nowMillis) {
  if (nowMillis - lastPIDTime >= 20) {
    float dt = (nowMillis - lastPIDTime) / 1000.0;
    lastPIDTime = nowMillis;
    float error = targetRPS - rps;
    integral += error * dt;
    integral = constrain(integral, -200, 200);
    float derivative = (error - errorPrev) / dt;
    errorPrev = error;
    float output = (kP * error) + (kI * integral) + (kD * derivative);
    pwmValue = constrain(pwmValue + output * 4.0, 0, 1023);
    writePWM10bit((int)pwmValue);
  }
}

void updateFanSensor() {
  int val = analogRead(PIN_FAN_SENSOR);
  unsigned long nowMicros = micros();

  if (!fanCalibrated) {
    if (val > sensorMax) sensorMax = val;
    if (val < sensorMin) sensorMin = val;
    if (millis() - fanCalibStart >= FAN_CALIB_TIME) {
      fanCalibrated = true;
    }
    return;
  }

  int threshold = (sensorMax + sensorMin) / 2;
  int hysteresis = (sensorMax - sensorMin) / 6;

  if (!aboveThreshold && val > threshold + hysteresis) {
    aboveThreshold = true;
    if (lastPulseTime > 0) {
      unsigned long interval = nowMicros - lastPulseTime;
      rps = 1000000.0 / (interval * POLES);
    }
    lastPulseTime = nowMicros;
    lastPulseReceived = nowMicros;
  }
  if (aboveThreshold && val < threshold - hysteresis) {
    aboveThreshold = false;
  }
  if (nowMicros - lastPulseReceived > 500000) rps = 0;

  if (val > sensorMax) sensorMax = val;
  if (val < sensorMin) sensorMin = val;
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
  if (sensorLock) return -1;
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
  int count = 0;
  for (int i = 0; i < 10; i++) {
    float d = readStable();
    if (d > 0) { sum += d; count++; }
    delay(100);
  }
  return (count > 0) ? sum / count : -1;
}

bool objectAtScanner() {
  static int hit = 0;
  float d = readStable();
  if (d > 0 && fabs(d - refDist) > 1.0) hit++;
  else hit = 0;
  return (hit >= 2);
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

bool isReversedPattern(int a[]) {
  int rev[PATTERN_SIZE];
  for (int i = 0; i < PATTERN_SIZE; i++) rev[i] = a[PATTERN_SIZE - 1 - i];
  if (comparePattern(rev, CardA)) return true;
  if (comparePattern(rev, CardB)) return true;
  if (comparePattern(rev, CardC)) return true;
  return false;
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
    }
    lastClockState = currentClock;
    return 0;
  }

  if (cardInserted && indexCounter < PATTERN_SIZE) {
    if (analogRead(PIN_LDR_COLOR) > THRESHOLD_REMOVE_COLOR) {
      if (removalTimer == 0) removalTimer = millis();
      if (millis() - removalTimer > 500) {
        Serial.println("CARD_REMOVED_EARLY");
        resetCardState();
        return 0;
      }
    } else {
      removalTimer = 0;
    }
  }

  if (currentClock == HIGH && lastClockState == LOW) {
    pattern[indexCounter] = (analogRead(PIN_LDR_DATA) > THRESHOLD_DATA) ? 1 : 0;
    int colorValue = analogRead(PIN_LDR_COLOR);
    colorSum += colorValue;
    colorCount++;
    indexCounter++;
  }
  lastClockState = currentClock;

  if (indexCounter >= PATTERN_SIZE) {
    int result = 0;
    if      (comparePattern(pattern, CardA)) { programNum = '1'; result = 1; }
    else if (comparePattern(pattern, CardB)) { programNum = '2'; result = 1; }
    else if (comparePattern(pattern, CardC)) { programNum = '3'; result = 1; }
    else if (isReversedPattern(pattern))     { programNum = 'E'; result = 2; }
    else                                     { programNum = 'E'; result = 2; }

    Serial.print("CARD_READ=");
    Serial.println(programNum);

    while (analogRead(PIN_LDR_CLOCK) < THRESHOLD_CARD_OUT) {
      delay(10);
    }
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

  setupPWM10bit();
  fanCalibStart = millis();

  showDash(CLK1, DIO1);
  clearDisplay(CLK2, DIO2);
  clearDisplay(CLK3, DIO3);

  state = STATE_FAN_CALIB;
  stateStartTime = millis();
}

void loop() {
  unsigned long now = millis();
  updateFanSensor();
  updateAllLeds(now);

  switch (state) {

    case STATE_FAN_CALIB: {
      if (fanCalibrated) {
        state = STATE_WEIGHT_ZERO_CALIB;
        stateStartTime = now;
      }
      break;
    }

    case STATE_WEIGHT_ZERO_CALIB: {
      static float sumF = 0;
      static int cntF = 0;
      float f = measureFreq();
      if (now - stateStartTime > 1000) {
        sumF += f;
        cntF++;
      }
      if (now - stateStartTime >= 4000) {
        zeroFreq = (cntF > 0) ? sumF / cntF : MODEL_C;
        zeroOffset = zeroFreq - MODEL_C;
        sumF = 0;
        cntF = 0;
        Serial.print("ZERO_FREQ=");
        Serial.print(zeroFreq, 3);
        Serial.print(" OFFSET=");
        Serial.println(zeroOffset, 3);
        state = STATE_WAIT_CARD;
        stateStartTime = now;
        ledBlink(ledCard, 2.0);
        showDash(CLK1, DIO1);
        clearDisplay(CLK2, DIO2);
        clearDisplay(CLK3, DIO3);
      }
      break;
    }

    case STATE_WAIT_CARD: {
      int r = readCardOnce();
      if (cardInserted) {
        ledSolid(ledCard);
      } else if (!cardInserted && !ledCard.solid) {
        ledBlink(ledCard, 2.0);
      }
      if (r == 1) {
        programValid = true;
        digitalWrite(PIN_BUZZER, HIGH);
        delay(150);
        digitalWrite(PIN_BUZZER, LOW);
        if (programNum == '1')      writeRaw(CLK1, DIO1, SEG_BLANK, SEG_BLANK, SEG_BLANK, seg[1]);
        else if (programNum == '2') writeRaw(CLK1, DIO1, SEG_BLANK, SEG_BLANK, SEG_BLANK, seg[2]);
        else if (programNum == '3') writeRaw(CLK1, DIO1, SEG_BLANK, SEG_BLANK, SEG_BLANK, seg[3]);
        ledBlink(ledCard, 2.0);
        state = STATE_CARD_VALID_DISPLAY;
        stateStartTime = now;
      } else if (r == 2) {
        programValid = false;
        showE(CLK1, DIO1);
        digitalWrite(PIN_BUZZER, HIGH);
        delay(500);
        digitalWrite(PIN_BUZZER, LOW);
        state = STATE_CARD_INVALID_DISPLAY;
        stateStartTime = now;
      }
      break;
    }

    case STATE_CARD_VALID_DISPLAY: {
      if (now - stateStartTime >= 2000) {
        ledOff(ledCard);
        ledBlink(ledScan, 0.25);
        showDash(CLK1, DIO1);
        clearDisplay(CLK2, DIO2);
        clearDisplay(CLK3, DIO3);
        state = STATE_WAIT_OBJECT;
        stateStartTime = now;
      }
      break;
    }

    case STATE_CARD_INVALID_DISPLAY: {
      if (now - stateStartTime >= 2000) {
        showDash(CLK1, DIO1);
        ledBlink(ledCard, 2.0);
        state = STATE_WAIT_CARD;
        stateStartTime = now;
      }
      break;
    }

    case STATE_WAIT_OBJECT: {
      if (now - lastObjectCheck > 200) {
        lastObjectCheck = now;
        servoY.writeMicroseconds(1000);
        if (refDist <= 0) refDist = getReference();
        if (objectAtScanner()) {
          Serial.println("OBJECT_DETECTED");
          ledBlink(ledScan, 0.5);
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
      buzzerBlink(now, 200);
      dimW = scanDimension();
      Serial.print("DIM1=");
      Serial.println(dimW);
      state = STATE_ROTATE_TO_DIM2;
      stateStartTime = now;
      break;
    }

    case STATE_ROTATE_TO_DIM2: {
      buzzerBlink(now, 200);
      rotateServoY(90);
      refDist = getReference();
      state = STATE_SCAN_DIM2;
      stateStartTime = now;
      break;
    }

    case STATE_SCAN_DIM2: {
      buzzerBlink(now, 200);
      dimL = scanDimension();
      Serial.print("DIM2=");
      Serial.println(dimL);
      state = STATE_ROTATE_TO_DIM3;
      stateStartTime = now;
      break;
    }

    case STATE_ROTATE_TO_DIM3: {
      buzzerBlink(now, 200);
      rotateServoY(180);
      refDist = getReference();
      state = STATE_SCAN_DIM3;
      stateStartTime = now;
      break;
    }

    case STATE_SCAN_DIM3: {
      buzzerBlink(now, 200);
      dimH = scanDimension();
      Serial.print("DIM3=");
      Serial.println(dimH);
      buzzerOff();
      state = STATE_SCAN_DONE;
      stateStartTime = now;
      lastDimCycle = now;
      dimCycleIdx = 0;
      break;
    }

    case STATE_SCAN_DONE: {
      ledSolid(ledScan);
      digitalWrite(PIN_BUZZER, HIGH);
      delay(200);
      digitalWrite(PIN_BUZZER, LOW);
      delay(100);
      digitalWrite(PIN_BUZZER, HIGH);
      delay(200);
      digitalWrite(PIN_BUZZER, LOW);

      showCM_1mm(CLK1, DIO1, dimW);
      showCM_1mm(CLK2, DIO2, dimL);
      showCM_1mm(CLK3, DIO3, dimH);
      Serial.print("DIMS W=");
      Serial.print(dimW);
      Serial.print(" L=");
      Serial.print(dimL);
      Serial.print(" H=");
      Serial.println(dimH);

      delay(2000);
      state = STATE_TRANSFER_TO_SCALE;
      stateStartTime = now;
      break;
    }

    case STATE_TRANSFER_TO_SCALE: {
      ledOff(ledScan);
      ledBlink(ledScan, 0.25);
      ledBlink(ledWeigh, 0.25);
      conveyorRunMs(120, 4000, true);
      state = STATE_WAIT_ON_SCALE;
      stateStartTime = now;
      break;
    }

    case STATE_WAIT_ON_SCALE: {
      float f = measureFreq();
      if (isObjectOnScale(f)) {
        ledBlink(ledWeigh, 1.0);
        state = STATE_WEIGHING;
        stateStartTime = now;
        weightStableStart = 0;
        lastWeight = 0;
        Serial.println("BOX_ON_SCALE");
      } else {
        if (now - stateStartTime > 30000) {
          Serial.println("SCALE_TIMEOUT");
          state = STATE_DONE;
        }
      }
      break;
    }

    case STATE_WEIGHING: {
      float freq = measureFreq();
      float weight = freqToWeight(freq);
      if (weight < 0) weight = 0;
      if (weight > 800) weight = 800;
      int wr = ((int)(weight + 5)) / 10 * 10;

      showInt(CLK1, DIO1, wr);

      if (fabs(weight - lastWeight) < WEIGHT_THRESHOLD) {
        if (weightStableStart == 0) weightStableStart = now;
        if (now - weightStableStart >= WEIGHT_STABLE_TIME && weight > MIN_WEIGHT) {
          measuredWeight = weight;
          targetTemp = weightToTargetTemp(measuredWeight);
          targetRPS = weightToTargetRPS(measuredWeight);
          Serial.print("W=");
          Serial.print(measuredWeight);
          Serial.print(" TGT_TEMP=");
          Serial.print(targetTemp);
          Serial.print(" TGT_RPS=");
          Serial.println(targetRPS);
          state = STATE_WEIGH_DONE;
          stateStartTime = now;
        }
      } else {
        weightStableStart = 0;
      }
      lastWeight = weight;
      break;
    }

    case STATE_WEIGH_DONE: {
      ledBlink(ledWeigh, 0.25);
      int wr = ((int)(measuredWeight + 5)) / 10 * 10;
      showInt(CLK1, DIO1, wr);
      if (beepCount < 3) {
        if (now - lastBeepTime >= 250) {
          lastBeepTime = now;
          if (digitalRead(PIN_BUZZER) == LOW) {
            digitalWrite(PIN_BUZZER, HIGH);
          } else {
            digitalWrite(PIN_BUZZER, LOW);
            beepCount++;
          }
        }
      } else {
        digitalWrite(PIN_BUZZER, LOW);
        if (now - stateStartTime >= 2500) {
          beepCount = 0;
          showInt(CLK1, DIO1, wr);
          showTemp(CLK2, DIO2, targetTemp);
          state = STATE_HEATING;
          stateStartTime = now;
          lastTempRead = 0;
          digitalWrite(PIN_LED_KETTLE, HIGH);
          digitalWrite(PIN_RELAY, RELAY_ON);
        }
      }
      break;
    }

    case STATE_HEATING: {
      updateFanPID(now);
      buzzerBlink(now, 300);

      if (now - lastTempRead >= TEMP_INTERVAL) {
        lastTempRead = now;
        currentTemp = readTemp();

        if (currentTemp < targetTemp - marginOn) {
          digitalWrite(PIN_RELAY, RELAY_ON);
          digitalWrite(PIN_LED_KETTLE, HIGH);
        } else if (currentTemp >= targetTemp - marginOff) {
          digitalWrite(PIN_RELAY, RELAY_OFF);
          digitalWrite(PIN_LED_KETTLE, LOW);
        }

        int wr = ((int)(measuredWeight + 5)) / 10 * 10;
        int rpm = (int)(rps * 60.0 + 0.5);
        showInt(CLK1, DIO1, wr);
        showTemp(CLK2, DIO2, targetTemp);
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
          buzzerOff();
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
        digitalWrite(PIN_BUZZER, HIGH);
        delay(150); digitalWrite(PIN_BUZZER, LOW); delay(100);
        digitalWrite(PIN_BUZZER, HIGH);
        delay(150); digitalWrite(PIN_BUZZER, LOW); delay(100);
        digitalWrite(PIN_BUZZER, HIGH);
        delay(500); digitalWrite(PIN_BUZZER, LOW);
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

      delay(5000);

      ledOff(ledCard);
      ledOff(ledScan);
      ledOff(ledWeigh);
      digitalWrite(PIN_LED_KETTLE, LOW);
      buzzerOff();
      programNum = 0;
      programValid = false;
      dimW = dimL = dimH = 0;
      measuredWeight = 0;
      currentTemp = 0;
      targetTemp = 0;
      targetRPS = 1.0;
      indexCounter = 0;
      cardInserted = false;
      weightStableStart = 0;
      lastWeight = 0;
      beepCount = 0;
      refDist = 0;
      servoY.writeMicroseconds(1000);

      ledBlink(ledCard, 2.0);
      showDash(CLK1, DIO1);
      clearDisplay(CLK2, DIO2);
      clearDisplay(CLK3, DIO3);
      state = STATE_WAIT_CARD;
      stateStartTime = millis();
      break;
    }

    default: break;
  }

  delay(2);
}
