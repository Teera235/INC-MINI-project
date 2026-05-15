#include <avr/io.h>

#define PIN_FAN_SENSOR  A2
#define PIN_PWM_FAN     11

const int POLES = 2;

int sensorMax = 0;
int sensorMin = 1023;
unsigned long lastPulseTime = 0;
unsigned long lastPulseReceived = 0;
bool aboveThreshold = false;

float rpsRaw = 0;
float rps = 0;
float rpsBuf[5] = { 0, 0, 0, 0, 0 };
int rpsBufIdx = 0;

bool pidEnabled = false;
float targetRPS = 5.0;

float kP = 0.3;
float kI = 0.4;
float kD = 0.02;
float errorPrev = 0;
float integral = 0;
const float INTEGRAL_LIMIT = 100.0;
const int PWM_MAX = 900;
const int PWM_MIN = 0;

float pwmValue = 200.0;
unsigned long lastPIDTime = 0;
unsigned long lastPrintTime = 0;
unsigned long lastSweepTime = 0;
int sweepStep = 0;

int lastSensorVal = 0;
unsigned long pulseCount = 0;

void setupPWM10bit() {
  pinMode(PIN_PWM_FAN, OUTPUT);
  TCCR1A = _BV(COM1A1) | _BV(WGM11) | _BV(WGM10);
  TCCR1B = _BV(WGM12) | _BV(CS10);
  OCR1A = 0;
}

void writePWM10bit(int value) {
  OCR1A = constrain(value, PWM_MIN, PWM_MAX);
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
  if (tgtRPS <= 0) return 0;
  if (tgtRPS <= 1.0) return 200;
  if (tgtRPS >= 20.0) return 800;
  return (int)(200.0 + (tgtRPS - 1.0) * (600.0 / 19.0));
}

float weightToTargetRPS(float w) {
  if (w < 200) return 1.0;
  if (w > 800) return 20.0;
  return 1.0 + (w - 200.0) * (19.0 / 600.0);
}

void updateFanSensor() {
  int val = analogRead(PIN_FAN_SENSOR);
  lastSensorVal = val;
  unsigned long nowMicros = micros();

  if (val > sensorMax) sensorMax = val;
  if (val < sensorMin) sensorMin = val;

  int range = sensorMax - sensorMin;
  if (range < 50) return;

  int threshold = (sensorMax + sensorMin) / 2;
  int hysteresis = range / 6;

  if (!aboveThreshold && val > threshold + hysteresis) {
    aboveThreshold = true;
    pulseCount++;
    if (lastPulseTime > 0) {
      unsigned long interval = nowMicros - lastPulseTime;
      if (interval > 1000) {
        rpsRaw = 1000000.0 / (interval * POLES);
        rpsBuf[rpsBufIdx] = rpsRaw;
        rpsBufIdx = (rpsBufIdx + 1) % 5;
        rps = medianOf5(rpsBuf[0], rpsBuf[1], rpsBuf[2], rpsBuf[3], rpsBuf[4]);
      }
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
  if (!pidEnabled) return;
  if (nowMillis - lastPIDTime < 20) return;

  float dt = (nowMillis - lastPIDTime) / 1000.0;
  lastPIDTime = nowMillis;

  float error = targetRPS - rps;
  integral += error * dt;
  integral = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  float derivative = (error - errorPrev) / dt;
  errorPrev = error;

  float output = (kP * error) + (kI * integral) + (kD * derivative);
  pwmValue = constrain(pwmValue + output, PWM_MIN, PWM_MAX);
  writePWM10bit((int)pwmValue);
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

  Serial.println();
  Serial.println("=== FAN RPS TEST (safe mode) ===");
  Serial.println("PID is OFF at boot. Use 'c' to enable closed-loop.");
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  p<pwm>  raw PWM 0-900 (open loop, safe)");
  Serial.println("  s<rps>  set target RPS (FF only until c is sent)");
  Serial.println("  w<g>    set target by weight");
  Serial.println("  c       enable closed-loop PID");
  Serial.println("  o       disable PID (open loop)");
  Serial.println("  S       sweep PWM 100..900..100");
  Serial.println("  R       reset sensorMin/Max");
  Serial.println("  X       stop");
  Serial.println();
  Serial.println("Tip: First send 'p400' to start fan, watch sMin/sMax change.");
  Serial.println("     If range stays small, tach is not wired correctly.");
  Serial.println();

  writePWM10bit(0);
  lastPrintTime = millis();
}

void handleSerial() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == '\n' || c == '\r' || c == ' ') return;

  if (c == 's') {
    float v = Serial.parseFloat();
    setTarget(constrain(v, 0.0, 20.0));
  } else if (c == 'w') {
    float w = Serial.parseFloat();
    setTarget(weightToTargetRPS(w));
    Serial.print("weight=");
    Serial.println(w);
  } else if (c == 'p') {
    int p = Serial.parseInt();
    p = constrain(p, 0, PWM_MAX);
    pidEnabled = false;
    pwmValue = p;
    writePWM10bit(p);
    Serial.print("OPEN LOOP PWM=");
    Serial.println(p);
  } else if (c == 'c') {
    if (sensorMax - sensorMin < 100) {
      Serial.println("WARN: tach signal range too small. PID may saturate.");
      Serial.print("sMin=");
      Serial.print(sensorMin);
      Serial.print(" sMax=");
      Serial.println(sensorMax);
      Serial.println("Try 'p400' first to ensure fan is spinning and tach reads pulses.");
      return;
    }
    pidEnabled = true;
    integral = 0;
    errorPrev = 0;
    lastPIDTime = millis();
    Serial.println("PID ENABLED");
  } else if (c == 'o') {
    pidEnabled = false;
    Serial.println("PID DISABLED (open loop)");
  } else if (c == 'S') {
    sweepStep = 1;
    lastSweepTime = millis();
    pidEnabled = false;
    Serial.println("PWM SWEEP START");
  } else if (c == 'R') {
    sensorMax = 0;
    sensorMin = 1023;
    pulseCount = 0;
    Serial.println("Sensor range reset");
  } else if (c == 'X') {
    pidEnabled = false;
    pwmValue = 0;
    writePWM10bit(0);
    sweepStep = 0;
    Serial.println("STOP");
  }
}

void runSweep(unsigned long now) {
  if (sweepStep == 0) return;
  if (now - lastSweepTime < 2000) return;
  lastSweepTime = now;
  int p;
  if (sweepStep <= 9) {
    p = 100 + sweepStep * 100;
  } else if (sweepStep <= 17) {
    p = 100 + (18 - sweepStep) * 100;
  } else {
    sweepStep = 0;
    pwmValue = 0;
    writePWM10bit(0);
    Serial.println("SWEEP DONE");
    return;
  }
  pwmValue = p;
  writePWM10bit(p);
  Serial.print("SWEEP step=");
  Serial.print(sweepStep);
  Serial.print(" PWM=");
  Serial.println(p);
  sweepStep++;
}

void loop() {
  unsigned long now = millis();

  updateFanSensor();
  updateFanPID(now);
  handleSerial();
  runSweep(now);

  if (now - lastPrintTime >= 200) {
    lastPrintTime = now;
    Serial.print("mode=");
    Serial.print(pidEnabled ? "PID" : "OPEN");
    Serial.print(" tgt=");
    Serial.print(targetRPS, 1);
    Serial.print(" rps=");
    Serial.print(rps, 2);
    Serial.print(" rpm=");
    Serial.print((int)(rps * 60.0 + 0.5));
    Serial.print(" pwm=");
    Serial.print((int)pwmValue);
    Serial.print(" adc=");
    Serial.print(lastSensorVal);
    Serial.print(" sMin=");
    Serial.print(sensorMin);
    Serial.print(" sMax=");
    Serial.print(sensorMax);
    Serial.print(" pulses=");
    Serial.println(pulseCount);
  }

  delay(2);
}
