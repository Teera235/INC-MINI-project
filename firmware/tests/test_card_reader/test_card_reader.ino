#define PIN_LDR_CLOCK A8
#define PIN_LDR_DATA  A9
#define PIN_LDR_COLOR A10
#define PIN_LED       A3

const int THRESHOLD_HOLE = 500;
const int THRESHOLD_DATA = 200;
const int PATTERN_SIZE = 10;

int CardA[10] = { 0, 1, 0, 1, 0, 1, 0, 1, 1, 1 };
int CardB[10] = { 0, 1, 0, 1, 1, 0, 0, 1, 1, 1 };
int CardC[10] = { 0, 1, 0, 0, 1, 1, 0, 1, 1, 1 };

int pattern[PATTERN_SIZE];
int indexCounter = 0;
long colorSum = 0;
int colorCount = 0;
bool cardInserted = false;
bool lastClock = HIGH;
unsigned long removalTimer = 0;

bool comparePattern(int a[], int b[]) {
  for (int i = 0; i < PATTERN_SIZE; i++) {
    if (a[i] != b[i]) return false;
  }
  return true;
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  Serial.println("=== CARD READER TEST ===");
  Serial.println("Swipe a card. Output: bit pattern + color average.");
  Serial.println();
}

void loop() {
  int clockRaw = analogRead(PIN_LDR_CLOCK);
  bool currentClock = (clockRaw > THRESHOLD_HOLE) ? HIGH : LOW;

  if (!cardInserted) {
    if (currentClock == LOW) {
      cardInserted = true;
      indexCounter = 0;
      colorSum = 0;
      colorCount = 0;
      digitalWrite(PIN_LED, HIGH);
      Serial.println("CARD IN");
    }
    lastClock = currentClock;
    return;
  }

  if (cardInserted && indexCounter < PATTERN_SIZE) {
    if (analogRead(PIN_LDR_COLOR) > 55) {
      if (removalTimer == 0) removalTimer = millis();
      if (millis() - removalTimer > 500) {
        Serial.println("REMOVED EARLY");
        indexCounter = 0;
        colorSum = 0;
        colorCount = 0;
        cardInserted = false;
        digitalWrite(PIN_LED, LOW);
        removalTimer = 0;
        return;
      }
    } else {
      removalTimer = 0;
    }
  }

  if (currentClock == HIGH && lastClock == LOW) {
    int rawData = analogRead(PIN_LDR_DATA);
    pattern[indexCounter] = (rawData > THRESHOLD_DATA) ? 1 : 0;
    int colorValue = analogRead(PIN_LDR_COLOR);
    colorSum += colorValue;
    colorCount++;
    Serial.print("Bit ");
    Serial.print(indexCounter);
    Serial.print(" = ");
    Serial.print(pattern[indexCounter]);
    Serial.print(" | rawData=");
    Serial.print(rawData);
    Serial.print(" | color=");
    Serial.println(colorValue);
    indexCounter++;
  }
  lastClock = currentClock;

  if (indexCounter >= PATTERN_SIZE) {
    Serial.print("PATTERN: ");
    for (int i = 0; i < PATTERN_SIZE; i++) Serial.print(pattern[i]);
    Serial.println();

    if      (comparePattern(pattern, CardA)) Serial.println("RESULT: 1 (Card A)");
    else if (comparePattern(pattern, CardB)) Serial.println("RESULT: 2 (Card B)");
    else if (comparePattern(pattern, CardC)) Serial.println("RESULT: 3 (Card C)");
    else                                     Serial.println("RESULT: E");

    int avg = (colorCount > 0) ? (colorSum / colorCount) : 0;
    Serial.print("Avg color: ");
    Serial.print(avg);
    Serial.print(" -> ");
    if (avg >= 500 && avg < 600)      Serial.println("Yellow");
    else if (avg >= 400)              Serial.println("White");
    else if (avg < 200)               Serial.println("Black");
    else                              Serial.println("Unknown");

    Serial.println("Remove card to continue...");
    while (analogRead(PIN_LDR_CLOCK) < 900) delay(10);

    indexCounter = 0;
    colorSum = 0;
    colorCount = 0;
    cardInserted = false;
    digitalWrite(PIN_LED, LOW);
    Serial.println("Ready for next card");
    Serial.println();
  }
}
