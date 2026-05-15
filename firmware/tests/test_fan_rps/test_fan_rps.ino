#include <avr/io.h>

#define PIN_FAN_SENSOR  A2
#define PIN_PWM_FAN     11

const int POLES = 2;

int sensorMax = 800;
int sensorMin = 200;

unsigned long lastPulseTime = 0;
unsigned long lastPulseReceived = 0;
bool aboveThreshold = false;

float rpsRaw = 0;
float rps = 0;
float rpsBuf[5] = { 0, 0, 0, 0, 0 };
int rpsBufIdx = 0;

float targetRPS = 5.0;
float kP = 0.5;
float kI = 1.0;
float kD = 0.05;
float errorPrev = 0;
float integral = 0;
float pwmValue = 200.0;
unsigned long lastPIDTime = 0;
unsigned long lastPrintTime = 0;
unsigned long lastSweepTime = 0;
int sweepStep = 0;

void setupPWM10bit() {
  pinMode(PIN_PWM_FAN, OUTPUT);
  TCCR1A = _BV(COM1A1) | _BV(WGM11) | _BV(WGM10);
  TCCR1B = _BV(WGM12) | _BV(CS10);
  OCR1A = 200;
}

void writePWM10bit(int value) {
  OCR1A = constrain(value, 0, 1023);
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

int feedForwardPWM(float tgtRPS) {
  if (tgtRPS <= 1.0) return 150;
  if (tgtRPS >= 20.0) return 900;
  return (int)(150.0 + (tgtRPS - 1.0) * (750.0 / 19.0));
}

float weightToTargetRPS(float w) {
  if (w < 200) return 1.0;
  if (w > 800) return 20.0;
  return 1.0 + (w - 200.0) * (19.0 / 600.0);
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
      rpsRaw = 1000000.0 / (interval * POLES);
      rpsBuf[rpsBufIdx] = rpsRaw;
      rpsBufIdx = (rpsBufIdx + 1) % 5;
      rps = medianOf5(rpsBuf[0], rpsBuf[1], rpsBuf[2], rpsBuf[3], rpsBuf[4]);
    }
    lastPulseTime = nowMicros;
    lastPulseReceived = nowMicros;
  }
  if (aboveThreshold && val < threshold - hysteresis) {
    aboveThreshold = false;
  }
  if (nowMicros - lastPulseReceived > 500000) rps = 0;
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

void setTarget(float newRPS) {
  targetRPS = newRPS;
  pwmValue  = (float)feedForwardPWM(newRPS);
  writePWM10bit((int)pwmValue);
  integral  = 0;
  errorPrev = 0;
  Serial.print("SET targetRPS=");
  Serial.print(targetRPS, 2);
  Serial.print(" FF_PWM=");
  Serial.println((int)pwmValue);
}

void setup() {
  Serial.begin(115200);
  setupPWM10bit();
  delay(500);

  Serial.println("=== FAN RPS TEST ===");
  Serial.println("Commands via Serial:");
  Serial.println("  s<rps>   set target RPS  (e.g. s10)");
  Serial.println("  w<g>     set target by weight (e.g. w500)");
  Serial.println("  p<pwm>   raw PWM 0-1023 (open loop)");
  Serial.println("  S        auto sweep 1->20->1 RPS");
  Serial.println("  X        stop");
  Serial.println();

  setTarget(5.0);
  lastPrintTime = millis();
}

void handleSerial() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == 's') {
    float v = Serial.parseFloat();
    setTarget(constrain(v, 0.0, 20.0));
  } else if (c == 'w') {
    float w = Serial.parseFloat();
    setTarget(weightToTargetRPS(w));
    Serial.print("weight="); Serial.println(w);
  } else if (c == 'p') {
    int p = Serial.parseInt();
    p = constrain(p, 0, 1023);
    targetRPS = -1;
    pwmValue = p;
    writePWM10bit(p);
    Serial.print("RAW PWM="); Serial.println(p);
  } else if (c == 'S') {
    sweepStep = 1;
    lastSweepTime = millis();
    Serial.println("SWEEP START");
  } else if (c == 'X') {
    setTarget(0);
    writePWM10bit(0);
    Serial.println("STOP");
  }
}

void runSweep(unsigned long now) {
  if (sweepStep == 0) return;
  if (now - lastSweepTime < 4000) return;
  lastSweepTime = now;
  if (sweepStep <= 20) {
    setTarget((float)sweepStep);
    sweepStep++;
  } else if (sweepStep <= 40) {
    setTarget((float)(40 - sweepStep + 1));
    sweepStep++;
  } else {
    sweepStep = 0;
    Serial.println("SWEEP DONE");
  }
}

void loop() {
  unsigned long now = millis();
  unsigned long nowUs = micros();

  updateFanSensor();
  if (targetRPS >= 0) updateFanPID(now);
  handleSerial();
  runSweep(now);

  if (now - lastPrintTime >= 200) {
    lastPrintTime = now;
    Serial.print("tgt=");
    Serial.print(targetRPS, 2);
    Serial.print(" rps=");
    Serial.print(rps, 2);
    Serial.print(" rpm=");
    Serial.print((int)(rps * 60.0 + 0.5));
    Serial.print(" pwm=");
    Serial.print((int)pwmValue);
    Serial.print(" sMin=");
    Serial.print(sensorMin);
    Serial.print(" sMax=");
    Serial.println(sensorMax);
  }

  delay(2);
}
