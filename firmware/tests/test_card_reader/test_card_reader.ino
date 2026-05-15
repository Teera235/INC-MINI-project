#define PIN_LDR_CLOCK A8
#define PIN_LDR_DATA  A9
#define PIN_LDR_COLOR A10
#define PIN_LED       A3
#define PIN_BUZZER    9

#define CLK1 22
#define DIO1 24
#define CLK2 26
#define DIO2 28
#define CLK3 30
#define DIO3 32

const int THRESHOLD_HOLE = 500;
const int THRESHOLD_DATA = 200;
const int THRESHOLD_REMOVE_COLOR = 55;
const int THRESHOLD_CARD_OUT = 900;
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

uint8_t seg[] = {
  0x3f, 0x06, 0x5b, 0x4f,
  0x66, 0x6d, 0x7d, 0x07,
  0x7f, 0x6f
};
uint8_t SEG_BLANK = 0x00;
uint8_t SEG_DASH  = 0x40;
uint8_t SEG_E     = 0x79;

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

void showDash(int clk, int dio) {
  writeRaw(clk, dio, SEG_BLANK, SEG_BLANK, SEG_BLANK, SEG_DASH);
}

void showE(int clk, int dio) {
  writeRaw(clk, dio, SEG_BLANK, SEG_BLANK, SEG_BLANK, SEG_E);
}

void showDigit(int clk, int dio, int d) {
  writeRaw(clk, dio, SEG_BLANK, SEG_BLANK, SEG_BLANK, seg[d]);
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

void showPattern4(int clk, int dio, int *pat, int len) {
  int start = (len > 4) ? len - 4 : 0;
  uint8_t b[4] = { SEG_BLANK, SEG_BLANK, SEG_BLANK, SEG_BLANK };
  for (int i = 0; i < 4 && start + i < len; i++) {
    b[i] = pat[start + i] ? seg[1] : seg[0];
  }
  writeRaw(clk, dio, b[0], b[1], b[2], b[3]);
}

bool comparePattern(int a[], int b[]) {
  for (int i = 0; i < PATTERN_SIZE; i++) {
    if (a[i] != b[i]) return false;
  }
  return true;
}

const char* identifyColor(int val) {
  if (val >= 500 && val < 600) return "Yellow";
  if (val >= 400)              return "White";
  if (val < 200)               return "Black";
  return "Unknown";
}

void shortBeep(int ms) {
  digitalWrite(PIN_BUZZER, HIGH);
  delay(ms);
  digitalWrite(PIN_BUZZER, LOW);
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(CLK1, OUTPUT); pinMode(DIO1, OUTPUT);
  pinMode(CLK2, OUTPUT); pinMode(DIO2, OUTPUT);
  pinMode(CLK3, OUTPUT); pinMode(DIO3, OUTPUT);

  digitalWrite(PIN_LED, LOW);
  digitalWrite(PIN_BUZZER, LOW);

  showDash(CLK1, DIO1);
  clearDisplay(CLK2, DIO2);
  clearDisplay(CLK3, DIO3);

  Serial.println();
  Serial.println("=== CARD READER TEST ===");
  Serial.println("Display 1 = result (1/2/3 or E)");
  Serial.println("Display 2 = last 4 bits of pattern");
  Serial.println("Display 3 = bit count");
  Serial.println();
  Serial.println("Swipe a card now.");
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
      clearDisplay(CLK2, DIO2);
      showInt(CLK3, DIO3, 0);
      Serial.println("CARD IN");
    }
    lastClock = currentClock;
    return;
  }

  if (cardInserted && indexCounter < PATTERN_SIZE) {
    if (analogRead(PIN_LDR_COLOR) > THRESHOLD_REMOVE_COLOR) {
      if (removalTimer == 0) removalTimer = millis();
      if (millis() - removalTimer > 500) {
        Serial.println("REMOVED EARLY");
        showE(CLK1, DIO1);
        shortBeep(500);
        delay(2000);
        showDash(CLK1, DIO1);
        clearDisplay(CLK2, DIO2);
        clearDisplay(CLK3, DIO3);
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
    showPattern4(CLK2, DIO2, pattern, indexCounter);
    showInt(CLK3, DIO3, indexCounter);
  }
  lastClock = currentClock;

  if (indexCounter >= PATTERN_SIZE) {
    Serial.print("PATTERN: ");
    for (int i = 0; i < PATTERN_SIZE; i++) Serial.print(pattern[i]);
    Serial.println();

    int result = 0;
    if      (comparePattern(pattern, CardA)) { result = 1; }
    else if (comparePattern(pattern, CardB)) { result = 2; }
    else if (comparePattern(pattern, CardC)) { result = 3; }
    else                                     { result = 0; }

    if (result > 0) {
      Serial.print("RESULT: ");
      Serial.println(result);
      showDigit(CLK1, DIO1, result);
      shortBeep(150);
    } else {
      Serial.println("RESULT: E (unknown pattern)");
      showE(CLK1, DIO1);
      shortBeep(500);
    }

    int avg = (colorCount > 0) ? (colorSum / colorCount) : 0;
    Serial.print("Avg color: ");
    Serial.print(avg);
    Serial.print(" -> ");
    Serial.println(identifyColor(avg));

    Serial.println("Remove card to continue...");
    while (analogRead(PIN_LDR_CLOCK) < THRESHOLD_CARD_OUT) delay(10);

    delay(2000);
    showDash(CLK1, DIO1);
    clearDisplay(CLK2, DIO2);
    clearDisplay(CLK3, DIO3);

    indexCounter = 0;
    colorSum = 0;
    colorCount = 0;
    cardInserted = false;
    digitalWrite(PIN_LED, LOW);

    Serial.println("Ready for next card");
    Serial.println();
  }
}
