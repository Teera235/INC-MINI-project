#include "arduinoFFT.h"
#include <math.h>

#define PIN_FFT A0

#define CLK1 22
#define DIO1 24
#define CLK2 26
#define DIO2 28

const uint16_t SAMPLES = 128;
double vReal[SAMPLES];
double vImag[SAMPLES];
const float ALPHA = 0.15;
float filteredFreq = 0;
bool firstFreq = true;
const unsigned long SAMPLING_INTERVAL_US = 500;

float baselineFreq = 0;
bool baselineInit = false;
const float BASELINE_ALPHA = 0.002;
const float BASELINE_LOCK = 0.5;

const float W_MIN = 600.0;
const float W_MAX = 800.0;
const float FREQ_RANGE = 6.0;

uint8_t seg[] = {
  0x3f, 0x06, 0x5b, 0x4f,
  0x66, 0x6d, 0x7d, 0x07,
  0x7f, 0x6f
};
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
  float delta = baselineFreq - freq;
  if (delta <= 0) return W_MIN;
  float w = W_MIN + (delta / FREQ_RANGE) * (W_MAX - W_MIN);
  if (w < W_MIN) w = W_MIN;
  if (w > W_MAX) w = W_MAX;
  return w;
}

void setup() {
  Serial.begin(115200);
  pinMode(CLK1, OUTPUT); pinMode(DIO1, OUTPUT);
  pinMode(CLK2, OUTPUT); pinMode(DIO2, OUTPUT);

  Serial.println("=== WEIGHT DISPLAY TEST (600-800g) ===");
  Serial.println("Calibrating baseline 3s...");

  float sum = 0;
  int cnt = 0;
  unsigned long start = millis();
  while (millis() - start < 3000) {
    float f = measureFreq();
    if (millis() - start > 500) {
      sum += f;
      cnt++;
    }
  }
  baselineFreq = (cnt > 0) ? sum / cnt : filteredFreq;
  baselineInit = true;
  Serial.print("BASELINE = ");
  Serial.println(baselineFreq, 3);
  Serial.println("Place weight. Display shows 600-800g range.");
  Serial.println("FREQ_RANGE = ");
  Serial.println(FREQ_RANGE, 2);
  Serial.println("Adjust FREQ_RANGE if full load does not reach 800g.");
}

void loop() {
  float freq = measureFreq();

  if (fabs(freq - baselineFreq) < BASELINE_LOCK) {
    baselineFreq = BASELINE_ALPHA * freq + (1.0 - BASELINE_ALPHA) * baselineFreq;
  }

  float weight = freqToWeight(freq);
  int wr = ((int)(weight + 5)) / 10 * 10;

  showFloat1d(CLK1, DIO1, freq);
  showInt(CLK2, DIO2, wr);

  Serial.print("freq=");
  Serial.print(freq, 3);
  Serial.print(" base=");
  Serial.print(baselineFreq, 3);
  Serial.print(" delta=");
  Serial.print(baselineFreq - freq, 3);
  Serial.print(" w=");
  Serial.print(weight, 1);
  Serial.print(" disp=");
  Serial.println(wr);

  delay(100);
}
