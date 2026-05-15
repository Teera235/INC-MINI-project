#include <OneWire.h>

#define PIN_DS18B20    10
#define PIN_RELAY      12
#define PIN_LED_KETTLE A3
#define PIN_BUZZER     9

#define RELAY_ON  LOW
#define RELAY_OFF HIGH

OneWire ds(PIN_DS18B20);

float targetTemp = 40.0;
float marginOn = 1.0;
float marginOff = 0.3;

unsigned long lastTempRead = 0;
const unsigned long TEMP_INTERVAL = 1000;

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

void setup() {
  Serial.begin(115200);
  pinMode(PIN_RELAY, OUTPUT);
  pinMode(PIN_LED_KETTLE, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_RELAY, RELAY_OFF);
  digitalWrite(PIN_LED_KETTLE, LOW);
  Serial.println("=== TEMP + RELAY TEST ===");
  Serial.println("Send t<C> to set target (e.g. t45)");
  Serial.print("Default target = ");
  Serial.println(targetTemp, 1);
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 't') {
      float v = Serial.parseFloat();
      if (v > 0 && v < 100) {
        targetTemp = v;
        Serial.print("Target = ");
        Serial.println(targetTemp, 1);
      }
    } else if (c == 'X') {
      digitalWrite(PIN_RELAY, RELAY_OFF);
      digitalWrite(PIN_LED_KETTLE, LOW);
      digitalWrite(PIN_BUZZER, LOW);
      Serial.println("STOP");
      while (1);
    }
  }

  if (millis() - lastTempRead >= TEMP_INTERVAL) {
    lastTempRead = millis();
    float t = readTemp();

    if (t < targetTemp - marginOn) {
      digitalWrite(PIN_RELAY, RELAY_ON);
      digitalWrite(PIN_LED_KETTLE, HIGH);
    } else if (t >= targetTemp - marginOff) {
      digitalWrite(PIN_RELAY, RELAY_OFF);
      digitalWrite(PIN_LED_KETTLE, LOW);
    }

    Serial.print("T=");
    Serial.print(t, 2);
    Serial.print(" TGT=");
    Serial.print(targetTemp, 1);
    Serial.print(" RELAY=");
    Serial.println(digitalRead(PIN_RELAY) == RELAY_ON ? "ON" : "OFF");

    if (t >= targetTemp) {
      digitalWrite(PIN_RELAY, RELAY_OFF);
      digitalWrite(PIN_LED_KETTLE, LOW);
      digitalWrite(PIN_BUZZER, HIGH);
      Serial.println("TARGET REACHED");
      delay(3000);
      digitalWrite(PIN_BUZZER, LOW);
      while (1);
    }
  }
}
