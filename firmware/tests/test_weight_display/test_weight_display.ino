#include "arduinoFFT.h"
#include <math.h>

#define PIN_FFT A0
#define PIN_BUZZER 9

#define CLK1 22
#define DIO1 24
#define CLK2 26
#define DIO2 28
#define CLK3 30
#define DIO3 32

const uint16_t SAMPLES = 128;
double vReal[SAMPLES];
double vImag[SAMPLES];
const float ALPHA = 0.15;
float filteredFreq = 0;
bool firstFreq = true;
const unsigned long SAMPLING_INTERVAL_US = 500;

float freqEmpty = 0;
float freqFull = 0;
const float W_MIN = 600.0;
const float W_MAX = 800.0;

int calState = 0;
unsigned long calStart = 0;
float calSum = 0;
int calCount = 0;

uint8_t seg[] = {
  0x3f, 0x06, 0x5b, 0x4f,
  0x66, 0x6d, 0x7d, 0x07,
  0x7f, 0x6f
};
uint8_t SEG_BLANK = 0x00;
uint8_t SEG_DASH  = 0x40;

uint8_t TEXT_LO[4] = { 0x38, 0x3f, 0x00, 0x00 };
uint8_t TEXT_HI[4] = { 0x76, 0x06, 0x00, 0x00 };
uint8_t TEXT_GO[4] = { 0x3d, 0x3f, 0x00, 0x00 };

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

void showCountdown(int clk, int dio, int sec) {
  writeRaw(clk, dio, SEG_BLANK, SEG_BLANK, seg[sec / 10], seg[sec % 10]);
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
  float w = W_MIN + ratio * (W_MAX - W_MIN);
  return w;
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(CLK1, OUTPUT); pinMode(DIO1, OUTPUT);
  pinMode(CLK2, OUTPUT); pinMode(DIO2, OUTPUT);
  pinMode(CLK3, OUTPUT); pinMode(DIO3, OUTPUT);

  Serial.println("=== WEIGHT CALIBRATION (600-800g) ===");
  Serial.println("Phase 1: Leave empty for 10 seconds");
  Serial.println("Phase 2: Press full load for 10 seconds");

  showText(CLK1, DIO1, TEXT_LO);
  clearDisplay(CLK2, DIO2);
  clearDisplay(CLK3, DIO3);

  calState = 0;
  calStart = millis();
  calSum = 0;
  calCount = 0;

  digitalWrite(PIN_BUZZER, HIGH);
  delay(100);
  digitalWrite(PIN_BUZZER, LOW);
}

void loop() {
  unsigned long now = millis();
  unsigned long elapsed = now - calStart;
  float freq = measureFreq();

  if (calState == 0) {
    int remaining = 10 - (int)(elapsed / 1000);
    if (remaining < 0) remaining = 0;
    showCountdown(CLK3, DIO3, remaining);
    showFloat1d(CLK2, DIO2, freq);

    if (elapsed > 2000) {
      calSum += freq;
      calCount++;
    }

    if (elapsed >= 10000) {
      freqEmpty = calSum / calCount;
      Serial.print("FREQ_EMPTY = ");
      Serial.println(freqEmpty, 3);

      digitalWrite(PIN_BUZZER, HIGH);
      delay(300);
      digitalWrite(PIN_BUZZER, LOW);
      delay(200);
      digitalWrite(PIN_BUZZER, HIGH);
      delay(300);
      digitalWrite(PIN_BUZZER, LOW);

      calState = 1;
      calStart = now;
      calSum = 0;
      calCount = 0;

      showText(CLK1, DIO1, TEXT_HI);
      Serial.println("Phase 2: PRESS FULL LOAD NOW (10s)");
    }
  }
  else if (calState == 1) {
    int remaining = 10 - (int)(elapsed / 1000);
    if (remaining < 0) remaining = 0;
    showCountdown(CLK3, DIO3, remaining);
    showFloat1d(CLK2, DIO2, freq);

    if (elapsed > 2000) {
      calSum += freq;
      calCount++;
    }

    if (elapsed >= 10000) {
      freqFull = calSum / calCount;
      Serial.print("FREQ_FULL = ");
      Serial.println(freqFull, 3);
      Serial.print("RANGE = ");
      Serial.println(freqEmpty - freqFull, 3);

      digitalWrite(PIN_BUZZER, HIGH);
      delay(500);
      digitalWrite(PIN_BUZZER, LOW);

      calState = 2;
      showText(CLK1, DIO1, TEXT_GO);
      Serial.println("CALIBRATION DONE. Measuring...");
      Serial.print("Empty=");
      Serial.print(freqEmpty, 3);
      Serial.print(" Full=");
      Serial.println(freqFull, 3);
    }
  }
  else {
    float weight = freqToWeight(freq);
    int wr = ((int)(weight + 5)) / 10 * 10;

    showFloat1d(CLK1, DIO1, freq);
    showInt(CLK2, DIO2, wr);

    int remaining = (int)((freqEmpty - freq) * 100 / (freqEmpty - freqFull));
    if (remaining < 0) remaining = 0;
    if (remaining > 100) remaining = 100;
    showInt(CLK3, DIO3, remaining);

    Serial.print("freq=");
    Serial.print(freq, 3);
    Serial.print(" w=");
    Serial.print(weight, 1);
    Serial.print(" disp=");
    Serial.print(wr);
    Serial.print(" %=");
    Serial.println(remaining);

    delay(50);
  }
}
