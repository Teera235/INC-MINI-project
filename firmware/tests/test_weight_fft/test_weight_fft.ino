// FFT วัดความถี่ + auto-zero baseline ชดเชย drift
#include "arduinoFFT.h"

const int PIN_INPUT = A0;
const uint16_t SAMPLES = 512;
double vReal[SAMPLES];
double vImag[SAMPLES];

const float ALPHA = 0.10;
float filteredFreq = 0;
bool first = true;

// Moving average
const int AVG_SIZE = 1;
float avgBuf[AVG_SIZE];
int avgIdx = 0;
bool avgFull = false;

const unsigned long SAMPLING_INTERVAL_US = 500;

// --- Auto-zero baseline ---
// baseline จะ drift ตามค่าจริงช้าๆ
// เมื่อมีน้ำหนักกด ค่าจะเปลี่ยนเร็ว → baseline ตามไม่ทัน → เห็น delta
float baseline = 0;
bool baselineInit = false;
const float BASELINE_ALPHA = 0.002;  // ช้ามาก — ตาม drift ได้แต่ไม่ตามน้ำหนัก

// ถ้า delta เปลี่ยนเร็ว (มีน้ำหนัก) → หยุดอัพเดท baseline
const float CHANGE_THRESHOLD = 0.3;  // Hz — ถ้า delta > นี้ ถือว่ามีน้ำหนัก

float getMovingAvg(float newVal) {
  avgBuf[avgIdx] = newVal;
  avgIdx++;
  if (avgIdx >= AVG_SIZE) {
    avgIdx = 0;
    avgFull = true;
  }
  int count = avgFull ? AVG_SIZE : avgIdx;
  float sum = 0;
  for (int i = 0; i < count; i++) sum += avgBuf[i];
  return sum / count;
}

float measureFreq() {
  for (uint16_t i = 0; i < SAMPLES; i++) {
    unsigned long t = micros();
    vReal[i] = (double)analogRead(PIN_INPUT);
    vImag[i] = 0.0;
    while ((micros() - t) < SAMPLING_INTERVAL_US);
  }

  float actualSamplingFreq = 1000000.0 / (float)SAMPLING_INTERVAL_US;

  double sum = 0;
  for (uint16_t i = 0; i < SAMPLES; i++) sum += vReal[i];
  double dcOffset = sum / SAMPLES;
  for (uint16_t i = 0; i < SAMPLES; i++) vReal[i] -= dcOffset;

  ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, actualSamplingFreq);
  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(FFT_FORWARD);
  FFT.complexToMagnitude();

  double peakMag = 0;
  uint16_t peakBin = 1;
  for (uint16_t i = 1; i < (SAMPLES / 2); i++) {
    if (vReal[i] > peakMag) {
      peakMag = vReal[i];
      peakBin = i;
    }
  }

  float freq = (float)peakBin * actualSamplingFreq / (float)SAMPLES;

  if (peakBin > 1 && peakBin < (SAMPLES / 2 - 1)) {
    double y0 = vReal[peakBin - 1];
    double y1 = vReal[peakBin];
    double y2 = vReal[peakBin + 1];
    double denom = y0 - 2.0 * y1 + y2;
    if (denom != 0) {
      double delta = 0.5 * (y0 - y2) / denom;
      freq = ((float)peakBin + delta) * actualSamplingFreq / (float)SAMPLES;
    }
  }

  if (first) {
    filteredFreq = freq;
    first = false;
  } else {
    filteredFreq = ALPHA * freq + (1.0 - ALPHA) * filteredFreq;
  }

  return getMovingAvg(filteredFreq);
}

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < AVG_SIZE; i++) avgBuf[i] = 0;
}

void loop() {
  float freq = measureFreq();

  // --- Auto-zero baseline ---
  if (!baselineInit) {
    baseline = freq;
    baselineInit = true;
  } else {
    float delta = abs(freq - baseline);

    // ถ้า delta น้อย (ไม่มีน้ำหนัก) → อัพเดท baseline ตาม drift
    if (delta < CHANGE_THRESHOLD) {
      baseline = BASELINE_ALPHA * freq + (1.0 - BASELINE_ALPHA) * baseline;
    }
    // ถ้า delta มาก (มีน้ำหนัก) → ไม่อัพเดท baseline
  }

  float deltaFreq = freq - baseline;

  // แสดงทั้ง raw freq, baseline, และ delta
  Serial.print(freq, 3);
  Serial.print(",");
  Serial.print(baseline, 3);
  Serial.print(",");
  Serial.println(deltaFreq, 3);
}
