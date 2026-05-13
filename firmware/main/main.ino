#include <Wire.h>
#include <Servo.h>
#include <OneWire.h>
#include <math.h>
#include <avr/io.h>
#include "arduinoFFT.h"
#include "Adafruit_VL53L0X.h"

#define PIN_FFT          A0
#define PIN_FAN_SENSOR   A2
#define PIN_LDR_CLOCK    A4
#define PIN_LDR_DATA     A5
#define PIN_LDR_COLOR    A6

#define PIN_PWM_FAN      11
#define PIN_DS18B20      10
#define PIN_RELAY        12
#define PIN_BUZZER       9
#define PIN_LED1         8
#define PIN_LED2         7
#define PIN_LED3         6

#define PIN_MOTOR_ENA    5
#define PIN_MOTOR_IN1    4
#define PIN_MOTOR_IN2    3

#define PIN_SERVO_X      44
#define PIN_SERVO_Y      45
#define PIN_LED_CARD     13

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

const int POLES = 2;
float targetRPS = 20.0;
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
float pwmValue = 400.0;
unsigned long lastPIDTime = 0;

OneWire ds(PIN_DS18B20);
float targetTemp = 0;
float currentTemp = 0;
float marginOn = 1.0;
float marginOff = 0.3;
unsigned long lastBeep = 0;
bool beepState = false;
unsigned long lastTempRead = 0;
const unsigned long TEMP_INTERVAL = 1000;

Adafruit_VL53L0X lox;
Servo servoX;
Servo servoY;
float refDist = 0;
float measuredH = 0;
float measuredL = 0;
bool sensorLock = false;

const int THRESHOLD_HOLE = 500;
const int PATTERN_SIZE = 10;
int CardA[10] = { 0, 1, 0, 1, 0, 1, 0, 1, 1, 1 };
int CardB[10] = { 0, 1, 0, 1, 1, 0, 0, 1, 1, 1 };
int CardC[10] = { 0, 1, 0, 0, 1, 1, 0, 1, 1, 1 };
int pattern[PATTERN_SIZE];
int indexCounter = 0;
bool cardInserted = false;
bool stableClock = HIGH;
bool lastClockRaw = HIGH;
unsigned long debounceTimer = 0;
const int DEBOUNCE_DELAY = 30;
char cardType = 0;

float measuredWeight = 0;
unsigned long weightStableStart = 0;
const unsigned long WEIGHT_STABLE_TIME = 3000;
float lastWeight = 0;
const float WEIGHT_THRESHOLD = 10.0;
const float MIN_WEIGHT = 20.0;

unsigned long stateStartTime = 0;
unsigned long lastObjectCheck = 0;

enum SystemState {
  STATE_INIT,
  STATE_FAN_CALIB,
  STATE_WEIGHT_ZERO_CALIB,
  STATE_WAIT_CARD,
  STATE_REF_DIST_1,
  STATE_WAIT_OBJECT,
  STATE_SCAN_HEIGHT,
  STATE_ROTATE_Y,
  STATE_REF_DIST_2,
  STATE_SCAN_LENGTH,
  STATE_WAIT_WEIGHT,
  STATE_HEATING,
  STATE_CONVEYOR,
  STATE_DONE
};
SystemState state = STATE_INIT;

uint8_t seg[] = {
  0x3f, 0x06, 0x5b, 0x4f,
  0x66, 0x6d, 0x7d, 0x07,
  0x7f, 0x6f
};

uint8_t TEXT_CAL[4]  = { 0x39, 0x77, 0x38, 0x00 };
uint8_t TEXT_GO[4]   = { 0x3d, 0x3f, 0x00, 0x00 };
uint8_t TEXT_DASH[4] = { 0x40, 0x40, 0x40, 0x40 };
uint8_t TEXT_DONE[4] = { 0x5e, 0x3f, 0x54, 0x79 };
uint8_t TEXT_HEAT[4] = { 0x76, 0x79, 0x77, 0x78 };
uint8_t TEXT_CARD[4] = { 0x39, 0x77, 0x50, 0x5e };
uint8_t TEXT_SCAN[4] = { 0x6d, 0x39, 0x77, 0x54 };
uint8_t TEXT_RUN[4]  = { 0x50, 0x1c, 0x54, 0x00 };

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

void showFloat1d(int clk, int dio, float val) {
  int v = (int)(val * 10);
  if (v > 9999) v = 9999;
  if (v < 0) v = 0;
  startTM(clk, dio);
  writeByte(clk, dio, 0x40);
  stopTM(clk, dio);
  startTM(clk, dio);
  writeByte(clk, dio, 0xC0);
  writeByte(clk, dio, (v / 1000) % 10 > 0 ? seg[(v / 1000) % 10] : 0x00);
  writeByte(clk, dio, seg[(v / 100) % 10]);
  writeByte(clk, dio, seg[(v / 10) % 10] | 0x80);
  writeByte(clk, dio, seg[v % 10]);
  stopTM(clk, dio);
  startTM(clk, dio);
  writeByte(clk, dio, 0x88 | 7);
  stopTM(clk, dio);
}

void showFloat2d(int clk, int dio, float val) {
  int v = (int)(val * 100);
  if (v > 9999) v = 9999;
  if (v < 0) v = 0;
  startTM(clk, dio);
  writeByte(clk, dio, 0x40);
  stopTM(clk, dio);
  startTM(clk, dio);
  writeByte(clk, dio, 0xC0);
  writeByte(clk, dio, seg[(v / 1000) % 10]);
  writeByte(clk, dio, seg[(v / 100) % 10] | 0x80);
  writeByte(clk, dio, seg[(v / 10) % 10]);
  writeByte(clk, dio, seg[v % 10]);
  stopTM(clk, dio);
  startTM(clk, dio);
  writeByte(clk, dio, 0x88 | 7);
  stopTM(clk, dio);
}

void showInt(int clk, int dio, int val) {
  if (val > 9999) val = 9999;
  if (val < 0) val = 0;
  int d1 = (val / 1000) % 10;
  int d2 = (val / 100) % 10;
  int d3 = (val / 10) % 10;
  int d4 = val % 10;
  startTM(clk, dio);
  writeByte(clk, dio, 0x40);
  stopTM(clk, dio);
  startTM(clk, dio);
  writeByte(clk, dio, 0xC0);
  writeByte(clk, dio, d1 > 0 ? seg[d1] : 0x00);
  writeByte(clk, dio, (d1 > 0 || d2 > 0) ? seg[d2] : 0x00);
  writeByte(clk, dio, (d1 > 0 || d2 > 0 || d3 > 0) ? seg[d3] : 0x00);
  writeByte(clk, dio, seg[d4]);
  stopTM(clk, dio);
  startTM(clk, dio);
  writeByte(clk, dio, 0x88 | 7);
  stopTM(clk, dio);
}

void showText(int clk, int dio, uint8_t data[]) {
  startTM(clk, dio);
  writeByte(clk, dio, 0x40);
  stopTM(clk, dio);
  startTM(clk, dio);
  writeByte(clk, dio, 0xC0);
  for (int i = 0; i < 4; i++) writeByte(clk, dio, data[i]);
  stopTM(clk, dio);
  startTM(clk, dio);
  writeByte(clk, dio, 0x88 | 7);
  stopTM(clk, dio);
}

void clearDisplay(int clk, int dio) {
  startTM(clk, dio);
  writeByte(clk, dio, 0x40);
  stopTM(clk, dio);
  startTM(clk, dio);
  writeByte(clk, dio, 0xC0);
  for (int i = 0; i < 4; i++) writeByte(clk, dio, 0x00);
  stopTM(clk, dio);
  startTM(clk, dio);
  writeByte(clk, dio, 0x88 | 7);
  stopTM(clk, dio);
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
  OCR1A = 400;
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
  for (int i = 0; i < 3; i++) {
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

bool objectDetected() {
  static int hit = 0;
  float d = readStable();
  if (d > 0 && fabs(d - refDist) > 5) hit++;
  else hit = 0;
  return (hit >= 3);
}

void rotateY_180() {
  for (int p = 0; p <= 180; p++) {
    servoY.writeMicroseconds(map(p, 0, 180, 1000, 2000));
    delay(15);
  }
  delay(500);
}

float scanAxis() {
  sensorLock = true;
  float sum = 0;
  int count = 0;
  int center = 1500;
  int range = 120;
  for (int i = 0; i < 80; i++) {
    int pos = map(i, 0, 80, center + range, center - range);
    servoX.writeMicroseconds(pos);
    delay(15);
    sensorLock = false;
    float d = readStable();
    sensorLock = true;
    if (d > 0) {
      float v = refDist - d;
      if (v > 0) { sum += v; count++; }
    }
  }
  servoX.writeMicroseconds(1500);
  sensorLock = false;
  return (count > 0) ? sum / count : -1;
}

void runConveyor(unsigned long durationMs, int speed) {
  digitalWrite(PIN_MOTOR_IN1, HIGH);
  digitalWrite(PIN_MOTOR_IN2, LOW);
  analogWrite(PIN_MOTOR_ENA, speed);
  unsigned long start = millis();
  while (millis() - start < durationMs) {
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

bool readCard() {
  int clockRaw = analogRead(PIN_LDR_CLOCK);
  bool rawClock = (clockRaw > THRESHOLD_HOLE);
  if (rawClock != lastClockRaw) debounceTimer = millis();
  if (millis() - debounceTimer > DEBOUNCE_DELAY) stableClock = rawClock;
  bool prevRaw = lastClockRaw;
  lastClockRaw = rawClock;
  bool currentClock = stableClock;

  if (!cardInserted) {
    if (currentClock == LOW) {
      cardInserted = true;
      indexCounter = 0;
      digitalWrite(PIN_LED_CARD, HIGH);
    }
    return false;
  }

  if (currentClock == HIGH && prevRaw == LOW && indexCounter < PATTERN_SIZE) {
    pattern[indexCounter] = (analogRead(PIN_LDR_DATA) > 500);
    indexCounter++;
  }

  if (indexCounter >= PATTERN_SIZE) {
    cardInserted = false;
    digitalWrite(PIN_LED_CARD, LOW);
    if (comparePattern(pattern, CardA)) { cardType = 'A'; return true; }
    if (comparePattern(pattern, CardB)) { cardType = 'B'; return true; }
    if (comparePattern(pattern, CardC)) { cardType = 'C'; return true; }
    indexCounter = 0;
  }
  return false;
}

void setLEDsForCard(char c) {
  digitalWrite(PIN_LED1, c == 'A' ? HIGH : LOW);
  digitalWrite(PIN_LED2, c == 'B' ? HIGH : LOW);
  digitalWrite(PIN_LED3, c == 'C' ? HIGH : LOW);
}

void setup() {
  Serial.begin(115200);

  pinMode(CLK1, OUTPUT); pinMode(DIO1, OUTPUT);
  pinMode(CLK2, OUTPUT); pinMode(DIO2, OUTPUT);
  pinMode(CLK3, OUTPUT); pinMode(DIO3, OUTPUT);

  pinMode(PIN_RELAY, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);
  pinMode(PIN_LED3, OUTPUT);
  pinMode(PIN_LED_CARD, OUTPUT);
  digitalWrite(PIN_RELAY, RELAY_OFF);

  pinMode(PIN_MOTOR_ENA, OUTPUT);
  pinMode(PIN_MOTOR_IN1, OUTPUT);
  pinMode(PIN_MOTOR_IN2, OUTPUT);
  analogWrite(PIN_MOTOR_ENA, 0);

  Wire.begin();
  lox.begin();
  servoX.attach(PIN_SERVO_X);
  servoY.attach(PIN_SERVO_Y);
  servoX.writeMicroseconds(1500);
  servoY.writeMicroseconds(1500);

  setupPWM10bit();
  fanCalibStart = millis();

  showText(CLK1, DIO1, TEXT_CAL);
  showText(CLK2, DIO2, TEXT_DASH);
  showText(CLK3, DIO3, TEXT_DASH);

  state = STATE_FAN_CALIB;
  stateStartTime = millis();
}

void loop() {
  unsigned long nowMillis = millis();
  updateFanSensor();

  switch (state) {

    case STATE_FAN_CALIB: {
      if (fanCalibrated) {
        state = STATE_WEIGHT_ZERO_CALIB;
        stateStartTime = nowMillis;
        showText(CLK1, DIO1, TEXT_CAL);
        clearDisplay(CLK2, DIO2);
        clearDisplay(CLK3, DIO3);
      }
      break;
    }

    case STATE_WEIGHT_ZERO_CALIB: {
      static float sumF = 0;
      static int cntF = 0;
      float f = measureFreq();
      showFloat1d(CLK2, DIO2, f);
      if (nowMillis - stateStartTime > 1000) {
        sumF += f;
        cntF++;
      }
      if (nowMillis - stateStartTime >= 4000) {
        zeroFreq = sumF / cntF;
        zeroOffset = zeroFreq - MODEL_C;
        sumF = 0;
        cntF = 0;
        state = STATE_WAIT_CARD;
        stateStartTime = nowMillis;
        showText(CLK1, DIO1, TEXT_CARD);
        clearDisplay(CLK2, DIO2);
        clearDisplay(CLK3, DIO3);
        Serial.print("ZERO_FREQ=");
        Serial.print(zeroFreq, 3);
        Serial.print(" OFFSET=");
        Serial.println(zeroOffset, 3);
      }
      break;
    }

    case STATE_WAIT_CARD: {
      if (readCard()) {
        setLEDsForCard(cardType);
        Serial.print("CARD=");
        Serial.println(cardType);
        state = STATE_REF_DIST_1;
        stateStartTime = nowMillis;
        showText(CLK1, DIO1, TEXT_SCAN);
      }
      break;
    }

    case STATE_REF_DIST_1: {
      refDist = getReference();
      Serial.print("REF1=");
      Serial.println(refDist);
      state = STATE_WAIT_OBJECT;
      stateStartTime = nowMillis;
      lastObjectCheck = nowMillis;
      break;
    }

    case STATE_WAIT_OBJECT: {
      if (nowMillis - lastObjectCheck > 200) {
        lastObjectCheck = nowMillis;
        if (objectDetected()) {
          state = STATE_SCAN_HEIGHT;
          stateStartTime = nowMillis;
        }
      }
      break;
    }

    case STATE_SCAN_HEIGHT: {
      measuredH = scanAxis();
      Serial.print("H=");
      Serial.println(measuredH);
      showFloat2d(CLK2, DIO2, measuredH);
      state = STATE_ROTATE_Y;
      stateStartTime = nowMillis;
      break;
    }

    case STATE_ROTATE_Y: {
      rotateY_180();
      state = STATE_REF_DIST_2;
      stateStartTime = nowMillis;
      break;
    }

    case STATE_REF_DIST_2: {
      refDist = getReference();
      Serial.print("REF2=");
      Serial.println(refDist);
      state = STATE_SCAN_LENGTH;
      stateStartTime = nowMillis;
      break;
    }

    case STATE_SCAN_LENGTH: {
      measuredL = scanAxis();
      Serial.print("L=");
      Serial.println(measuredL);
      showFloat2d(CLK3, DIO3, measuredL);
      state = STATE_WAIT_WEIGHT;
      stateStartTime = nowMillis;
      weightStableStart = 0;
      lastWeight = 0;
      break;
    }

    case STATE_WAIT_WEIGHT: {
      float freq = measureFreq();
      float weight = freqToWeight(freq);
      if (weight < 0) weight = 0;
      if (weight > 800) weight = 800;
      int wr = ((int)(weight + 5)) / 10 * 10;

      showFloat1d(CLK1, DIO1, freq);
      showInt(CLK2, DIO2, wr);

      if (fabs(weight - lastWeight) < WEIGHT_THRESHOLD) {
        if (weightStableStart == 0) weightStableStart = nowMillis;
        if (nowMillis - weightStableStart >= WEIGHT_STABLE_TIME && weight > MIN_WEIGHT) {
          measuredWeight = weight;
          targetTemp = 0.025 * measuredWeight + 30.0;
          state = STATE_HEATING;
          stateStartTime = nowMillis;
          lastTempRead = 0;
          showText(CLK1, DIO1, TEXT_HEAT);
          Serial.print("W=");
          Serial.print(measuredWeight);
          Serial.print(" TARGET=");
          Serial.println(targetTemp, 2);
        }
      } else {
        weightStableStart = 0;
      }
      lastWeight = weight;

      updateFanPID(nowMillis);
      break;
    }

    case STATE_HEATING: {
      updateFanPID(nowMillis);

      if (nowMillis - lastTempRead >= TEMP_INTERVAL) {
        lastTempRead = nowMillis;
        currentTemp = readTemp();

        if (currentTemp < targetTemp - marginOn) {
          digitalWrite(PIN_RELAY, RELAY_ON);
        } else if (currentTemp >= targetTemp - marginOff) {
          digitalWrite(PIN_RELAY, RELAY_OFF);
        }

        showFloat2d(CLK1, DIO1, targetTemp);
        showInt(CLK2, DIO2, (int)measuredWeight);
        showFloat2d(CLK3, DIO3, currentTemp);

        Serial.print("T=");
        Serial.print(currentTemp, 2);
        Serial.print(" TGT=");
        Serial.println(targetTemp, 2);

        if (currentTemp >= targetTemp) {
          digitalWrite(PIN_RELAY, RELAY_OFF);
          digitalWrite(PIN_BUZZER, HIGH);
          delay(3000);
          digitalWrite(PIN_BUZZER, LOW);
          showText(CLK2, DIO2, TEXT_DONE);
          state = STATE_CONVEYOR;
          stateStartTime = nowMillis;
        }
      }

      if (state == STATE_HEATING && nowMillis - lastBeep > 300) {
        lastBeep = nowMillis;
        beepState = !beepState;
        digitalWrite(PIN_BUZZER, beepState);
      }
      break;
    }

    case STATE_CONVEYOR: {
      showText(CLK1, DIO1, TEXT_RUN);
      runConveyor(5000, 120);
      state = STATE_DONE;
      stateStartTime = nowMillis;
      break;
    }

    case STATE_DONE: {
      Serial.print("FINAL H=");
      Serial.print(measuredH);
      Serial.print(" L=");
      Serial.print(measuredL);
      Serial.print(" W=");
      Serial.print(measuredWeight);
      Serial.print(" CARD=");
      Serial.println(cardType);

      showText(CLK1, DIO1, TEXT_DONE);
      showText(CLK2, DIO2, TEXT_DONE);
      showText(CLK3, DIO3, TEXT_DONE);
      delay(5000);

      digitalWrite(PIN_LED1, LOW);
      digitalWrite(PIN_LED2, LOW);
      digitalWrite(PIN_LED3, LOW);
      cardType = 0;
      measuredH = 0;
      measuredL = 0;
      measuredWeight = 0;
      indexCounter = 0;
      cardInserted = false;
      weightStableStart = 0;
      lastWeight = 0;

      state = STATE_WAIT_CARD;
      stateStartTime = millis();
      showText(CLK1, DIO1, TEXT_CARD);
      clearDisplay(CLK2, DIO2);
      clearDisplay(CLK3, DIO3);
      break;
    }

    default: break;
  }

  delay(2);
}
