#define PIN_BUZZER     9
#define PIN_LED_CARD   8
#define PIN_LED_SCAN   7
#define PIN_LED_WEIGH  6
#define PIN_LED_KETTLE A3

uint8_t leds[4] = { PIN_LED_CARD, PIN_LED_SCAN, PIN_LED_WEIGH, PIN_LED_KETTLE };
const char* names[4] = { "CARD(D8)", "SCAN(D7)", "WEIGH(D6)", "KETTLE(A3)" };

unsigned long lastToggle[4] = { 0, 0, 0, 0 };
unsigned long period[4] = { 0, 0, 0, 0 };
bool state[4] = { false, false, false, false };

void setBlink(int idx, float hz) {
  if (hz <= 0) {
    period[idx] = 0;
    digitalWrite(leds[idx], LOW);
    state[idx] = false;
    Serial.print(names[idx]);
    Serial.println(" OFF");
    return;
  }
  period[idx] = (unsigned long)(500.0 / hz);
  if (period[idx] < 50) period[idx] = 50;
  Serial.print(names[idx]);
  Serial.print(" blink ");
  Serial.print(hz, 2);
  Serial.println(" Hz");
}

void update(unsigned long now) {
  for (int i = 0; i < 4; i++) {
    if (period[i] == 0) continue;
    if (now - lastToggle[i] >= period[i]) {
      lastToggle[i] = now;
      state[i] = !state[i];
      digitalWrite(leds[i], state[i] ? HIGH : LOW);
    }
  }
}

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 4; i++) {
    pinMode(leds[i], OUTPUT);
    digitalWrite(leds[i], LOW);
  }
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);

  Serial.println("=== BUZZER + LEDS TEST ===");
  Serial.println("Commands:");
  Serial.println("  1<hz>  card LED   (e.g. 12 = 2 Hz)");
  Serial.println("  2<hz>  scan LED");
  Serial.println("  3<hz>  weigh LED");
  Serial.println("  4<hz>  kettle LED");
  Serial.println("  b      single beep 200 ms");
  Serial.println("  B      long beep 3 s");
  Serial.println("  T      triple beep");
  Serial.println("  X      all off");
  Serial.println();

  Serial.println("Self-test: cycle each LED for 1 s");
  for (int i = 0; i < 4; i++) {
    Serial.print("Test ");
    Serial.println(names[i]);
    digitalWrite(leds[i], HIGH);
    digitalWrite(PIN_BUZZER, HIGH);
    delay(300);
    digitalWrite(PIN_BUZZER, LOW);
    delay(700);
    digitalWrite(leds[i], LOW);
  }
  Serial.println("Self-test done");
}

void loop() {
  update(millis());

  if (Serial.available()) {
    char c = Serial.read();
    if (c >= '1' && c <= '4') {
      int idx = c - '1';
      float hz = Serial.parseFloat();
      setBlink(idx, hz);
    } else if (c == 'b') {
      digitalWrite(PIN_BUZZER, HIGH);
      delay(200);
      digitalWrite(PIN_BUZZER, LOW);
      Serial.println("beep");
    } else if (c == 'B') {
      digitalWrite(PIN_BUZZER, HIGH);
      delay(3000);
      digitalWrite(PIN_BUZZER, LOW);
      Serial.println("long beep done");
    } else if (c == 'T') {
      for (int i = 0; i < 3; i++) {
        digitalWrite(PIN_BUZZER, HIGH);
        delay(150);
        digitalWrite(PIN_BUZZER, LOW);
        delay(150);
      }
      Serial.println("triple beep done");
    } else if (c == 'X') {
      for (int i = 0; i < 4; i++) {
        period[i] = 0;
        digitalWrite(leds[i], LOW);
      }
      digitalWrite(PIN_BUZZER, LOW);
      Serial.println("ALL OFF");
    }
  }
}
