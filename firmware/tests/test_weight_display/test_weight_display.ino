// FFT + Quadratic model (จาก data จริง) + zero cal + drift comp + TM1637
// สูตร: freq = 0.0003768*w² - 0.002523*w + 451.32
// กลับด้าน: w = (-b + sqrt(b² - 4a*(c - freq))) / (2a)
//
// Calibrate:
//   0-10s:  ว่าง → วัด zero offset
//   10-20s: ว่าง → วัด drift rate

#include "arduinoFFT.h"
#include <math.h>

// --- FFT ---
const int PIN_INPUT = A0;
const uint16_t SAMPLES = 128;
double vReal[SAMPLES];
double vImag[SAMPLES];
const float ALPHA = 0.15;
float filteredFreq = 0;
bool first = true;
const unsigned long SAMPLING_INTERVAL_US = 500;

// --- Quadratic model (จาก fit_model.py) ---
// freq = A*w² + B*w + C
const float MODEL_A = 0.0003768;
const float MODEL_B = -0.002523;
const float MODEL_C = 451.32;
// ค่า freq ที่ 0g ตามโมเดล = MODEL_C = 451.32

// --- Calibration ---
int calPhase = 0;  // 0=zero, 1=drift, 2=done
float calSum = 0;
int calCount = 0;
float zeroFreq = 0;       // ค่าจริงตอน 0g (อาจไม่ตรง 451.32)
float zeroOffset = 0;     // offset = zeroFreq - MODEL_C
float driftFreqStart = 0; // ค่าเริ่มต้น drift phase
float driftFreqEnd = 0;   // ค่าสิ้นสุด drift phase
float driftRate = 0;       // Hz/s

unsigned long startTime = 0;
unsigned long driftStartTime = 0;
const unsigned long CAL_SKIP = 3000;
const unsigned long CAL_P1 = 10000;   // 0-10s zero
const unsigned long CAL_P2 = 20000;   // 10-20s drift

// --- TM1637 ---
#define CLK1  22
#define DIO1  24
#define CLK2  26
#define DIO2  28

uint8_t seg[] = {
  0x3f, 0x06, 0x5b, 0x4f,
  0x66, 0x6d, 0x7d, 0x07,
  0x7f, 0x6f
};

uint8_t TEXT_CAL[4]  = { 0x39, 0x77, 0x38, 0x00 };
uint8_t TEXT_ZR[4]   = { 0x5b, 0x50, 0x00, 0x00 };  // Zr
uint8_t TEXT_DR[4]   = { 0x5e, 0x50, 0x00, 0x00 };  // dr
uint8_t TEXT_GO[4]   = { 0x3d, 0x3f, 0x00, 0x00 };
uint8_t TEXT_DASH[4] = { 0x40, 0x40, 0x40, 0x40 };

void startTM(int clk, int dio) {
  pinMode(dio, OUTPUT);
  digitalWrite(dio, HIGH); digitalWrite(clk, HIGH);
  delayMicroseconds(5); digitalWrite(dio, LOW);
}
void stopTM(int clk, int dio) {
  digitalWrite(clk, LOW); digitalWrite(dio, LOW);
  delayMicroseconds(5); digitalWrite(clk, HIGH);
  delayMicroseconds(5); digitalWrite(dio, HIGH);
}
void writeByte(int clk, int dio, uint8_t b) {
  for (int i = 0; i < 8; i++) {
    digitalWrite(clk, LOW); delayMicroseconds(5);
    digitalWrite(dio, (b >> i) & 1); delayMicroseconds(5);
    digitalWrite(clk, HIGH); delayMicroseconds(5);
  }
  digitalWrite(clk, LOW); pinMode(dio, INPUT);
  delayMicroseconds(5); digitalWrite(clk, HIGH);
  delayMicroseconds(5); pinMode(dio, OUTPUT);
}
void showFloat(int clk, int dio, float val) {
  int v = (int)(val * 10);
  if (v > 9999) v = 9999; if (v < 0) v = 0;
  int d1=(v/1000)%10, d2=(v/100)%10, d3=(v/10)%10, d4=v%10;
  startTM(clk,dio); writeByte(clk,dio,0x40); stopTM(clk,dio);
  startTM(clk,dio); writeByte(clk,dio,0xC0);
  writeByte(clk,dio, d1>0 ? seg[d1] : 0x00);
  writeByte(clk,dio, seg[d2]);
  writeByte(clk,dio, seg[d3]|0x80);
  writeByte(clk,dio, seg[d4]);
  stopTM(clk,dio);
  startTM(clk,dio); writeByte(clk,dio,0x88|7); stopTM(clk,dio);
}
void showInt(int clk, int dio, int val) {
  if (val > 9999) val = 9999; if (val < 0) val = 0;
  int d1=(val/1000)%10, d2=(val/100)%10, d3=(val/10)%10, d4=val%10;
  startTM(clk,dio); writeByte(clk,dio,0x40); stopTM(clk,dio);
  startTM(clk,dio); writeByte(clk,dio,0xC0);
  writeByte(clk,dio, d1>0 ? seg[d1] : 0x00);
  writeByte(clk,dio, (d1>0||d2>0) ? seg[d2] : 0x00);
  writeByte(clk,dio, (d1>0||d2>0||d3>0) ? seg[d3] : 0x00);
  writeByte(clk,dio, seg[d4]);
  stopTM(clk,dio);
  startTM(clk,dio); writeByte(clk,dio,0x88|7); stopTM(clk,dio);
}
void showText(int clk, int dio, uint8_t data[]) {
  startTM(clk,dio); writeByte(clk,dio,0x40); stopTM(clk,dio);
  startTM(clk,dio); writeByte(clk,dio,0xC0);
  for (int i=0;i<4;i++) writeByte(clk,dio,data[i]);
  stopTM(clk,dio);
  startTM(clk,dio); writeByte(clk,dio,0x88|7); stopTM(clk,dio);
}

float measureFreq() {
  for (uint16_t i = 0; i < SAMPLES; i++) {
    unsigned long t = micros();
    vReal[i] = (double)analogRead(PIN_INPUT);
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

  double pm = 0; uint16_t pb = 1;
  for (uint16_t i = 1; i < (SAMPLES/2); i++) {
    if (vReal[i] > pm) { pm = vReal[i]; pb = i; }
  }
  float freq = (float)pb * sf / (float)SAMPLES;
  if (pb > 1 && pb < (SAMPLES/2-1)) {
    double y0=vReal[pb-1], y1=vReal[pb], y2=vReal[pb+1];
    double d = y0 - 2.0*y1 + y2;
    if (d != 0) freq = ((float)pb + 0.5*(y0-y2)/d) * sf / (float)SAMPLES;
  }
  if (first) { filteredFreq = freq; first = false; }
  else { filteredFreq = ALPHA * freq + (1.0-ALPHA) * filteredFreq; }
  return filteredFreq;
}

// Quadratic inverse: freq → weight
// freq = A*w² + B*w + C  →  A*w² + B*w + (C - freq) = 0
// w = (-B + sqrt(B² - 4A*(C - freq))) / (2A)
float freqToWeight(float freq) {
  float discriminant = MODEL_B * MODEL_B - 4.0 * MODEL_A * (MODEL_C - freq);
  if (discriminant < 0) return 0;
  float w = (-MODEL_B + sqrt(discriminant)) / (2.0 * MODEL_A);
  return w;
}

void setup() {
  Serial.begin(115200);
  pinMode(CLK1, OUTPUT); pinMode(DIO1, OUTPUT);
  pinMode(CLK2, OUTPUT); pinMode(DIO2, OUTPUT);
  startTime = millis();
  showText(CLK1, DIO1, TEXT_CAL);
  showText(CLK2, DIO2, TEXT_DASH);
  Serial.println("=== QUADRATIC MODEL + DRIFT ===");
  Serial.println("0-10s:  ว่าง (zero calibrate)");
  Serial.println("10-20s: ว่าง (วัด drift)");
}

void loop() {
  unsigned long elapsed = millis() - startTime;
  float freq = measureFreq();

  // === Phase 0: Zero calibrate (0-10s) ===
  if (calPhase == 0) {
    showText(CLK1, DIO1, TEXT_ZR);
    showFloat(CLK2, DIO2, freq);
    if (elapsed > CAL_SKIP) { calSum += freq; calCount++; }
    if (elapsed >= CAL_P1) {
      zeroFreq = calSum / calCount;
      zeroOffset = zeroFreq - MODEL_C;
      Serial.print("Zero freq = "); Serial.print(zeroFreq, 3);
      Serial.print(" | Offset = "); Serial.println(zeroOffset, 3);
      calPhase = 1; calSum = 0; calCount = 0;
      driftFreqStart = zeroFreq;
      Serial.println(">>> อย่าแตะ! วัด drift 10 วิ <<<");
    }
  }

  // === Phase 1: Drift measurement (10-20s) ===
  else if (calPhase == 1) {
    showText(CLK1, DIO1, TEXT_DR);
    showFloat(CLK2, DIO2, freq);
    if (elapsed > CAL_P1 + CAL_SKIP) { calSum += freq; calCount++; }
    if (elapsed >= CAL_P2) {
      driftFreqEnd = calSum / calCount;
      // drift ระหว่าง phase 0 กลาง (~5s) กับ phase 1 กลาง (~15s) = 10 วิ
      driftRate = (driftFreqEnd - driftFreqStart) / 10.0;
      Serial.print("Drift start = "); Serial.print(driftFreqStart, 3);
      Serial.print(" | end = "); Serial.print(driftFreqEnd, 3);
      Serial.print(" | rate = "); Serial.print(driftRate, 6);
      Serial.println(" Hz/s");

      calPhase = 2;
      driftStartTime = millis();
      showText(CLK1, DIO1, TEXT_GO);
      delay(1000);
      Serial.println("=== READY ===");
    }
  }

  // === Phase 2: วัดจริง ===
  else {
    // ชดเชย drift
    float secSinceStart = (millis() - driftStartTime) / 1000.0;
    float correctedFreq = freq - (driftRate * secSinceStart) - zeroOffset;

    float weight = freqToWeight(correctedFreq);
    if (weight < 0) weight = 0;
    if (weight > 800) weight = 800;
    int wr = ((int)(weight + 5)) / 10 * 10;

    Serial.print(freq, 1);
    Serial.print(" | corr: ");
    Serial.print(correctedFreq, 1);
    Serial.print(" | drift: ");
    Serial.print(driftRate * secSinceStart, 2);
    Serial.print(" | ");
    Serial.print(wr);
    Serial.println(" g");

    showFloat(CLK1, DIO1, correctedFreq);
    showInt(CLK2, DIO2, wr);
  }
}
