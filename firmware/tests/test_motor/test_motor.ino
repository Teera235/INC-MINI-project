#include <avr/io.h>

int sensor = A2;
int pwmPin = 11;  
const int POLES = 2;

float targetRPS = 20.0;

int sensorMax = 0;
int sensorMin = 1023;
bool calibrated = false;
unsigned long calibStartTime = 0;
const unsigned long CALIB_TIME = 3000;

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
unsigned long lastPrintTime = 0;

// ===== ตั้ง Timer1 เป็น 10-bit Fast PWM =====
void setupPWM10bit() {
  pinMode(pwmPin, OUTPUT);

  // Timer1: Fast PWM 10-bit, Non-inverting, No prescaler (31.25 kHz)
  TCCR1A = _BV(COM1A1) | _BV(WGM11) | _BV(WGM10);
  TCCR1B = _BV(WGM12)  | _BV(CS10);  // CS10 = No prescaler

  OCR1A = 400;  // ค่าเริ่มต้น PWM (0-1023)
}

void writePWM10bit(int value) {
  OCR1A = constrain(value, 0, 1023);
}

void setup() {
  Serial.begin(115200);
  setupPWM10bit();
  calibStartTime = millis();
}

void loop() {
  int val = analogRead(sensor);
  unsigned long nowMicros = micros();
  unsigned long nowMillis = millis();

  // ===== Calibration =====
  if (!calibrated) {
    if (val > sensorMax) sensorMax = val;
    if (val < sensorMin) sensorMin = val;
    if (nowMillis - calibStartTime >= CALIB_TIME) {
      calibrated = true;
      Serial.print("Max="); Serial.print(sensorMax);
      Serial.print(" Min="); Serial.println(sensorMin);
    }
    delay(5);
    return;
  }

  // ===== นับ Pulse → RPS =====
  int threshold  = (sensorMax + sensorMin) / 2;
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

  if (nowMicros - lastPrintTime >= 200000) {
    lastPrintTime = nowMicros;
    Serial.print(rps, 2);
    Serial.print(",");
    Serial.println(targetRPS);
  }

  delay(2);
}