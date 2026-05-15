#define CLK1 22
#define DIO1 24
#define CLK2 26
#define DIO2 28
#define CLK3 30
#define DIO3 32

uint8_t seg[] = {
  0x3f, 0x06, 0x5b, 0x4f,
  0x66, 0x6d, 0x7d, 0x07,
  0x7f, 0x6f
};
uint8_t SEG_DASH = 0x40;
uint8_t SEG_E    = 0x79;
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

void clearAll() {
  writeRaw(CLK1, DIO1, SEG_BLANK, SEG_BLANK, SEG_BLANK, SEG_BLANK);
  writeRaw(CLK2, DIO2, SEG_BLANK, SEG_BLANK, SEG_BLANK, SEG_BLANK);
  writeRaw(CLK3, DIO3, SEG_BLANK, SEG_BLANK, SEG_BLANK, SEG_BLANK);
}

void setup() {
  Serial.begin(115200);
  pinMode(CLK1, OUTPUT); pinMode(DIO1, OUTPUT);
  pinMode(CLK2, OUTPUT); pinMode(DIO2, OUTPUT);
  pinMode(CLK3, OUTPUT); pinMode(DIO3, OUTPUT);
  delay(200);
  Serial.println("=== TM1637 DISPLAY TEST ===");
}

void loop() {
  Serial.println("Step 1: All blank");
  clearAll();
  delay(1500);

  Serial.println("Step 2: Display 1=8888, 2=8888, 3=8888");
  writeRaw(CLK1, DIO1, seg[8], seg[8], seg[8], seg[8]);
  writeRaw(CLK2, DIO2, seg[8], seg[8], seg[8], seg[8]);
  writeRaw(CLK3, DIO3, seg[8], seg[8], seg[8], seg[8]);
  delay(2000);

  Serial.println("Step 3: D1=1, D2=2, D3=3 (single digit right-aligned)");
  writeRaw(CLK1, DIO1, SEG_BLANK, SEG_BLANK, SEG_BLANK, seg[1]);
  writeRaw(CLK2, DIO2, SEG_BLANK, SEG_BLANK, SEG_BLANK, seg[2]);
  writeRaw(CLK3, DIO3, SEG_BLANK, SEG_BLANK, SEG_BLANK, seg[3]);
  delay(2000);

  Serial.println("Step 4: D1=dash, D2=E, D3=blank");
  writeRaw(CLK1, DIO1, SEG_BLANK, SEG_BLANK, SEG_BLANK, SEG_DASH);
  writeRaw(CLK2, DIO2, SEG_BLANK, SEG_BLANK, SEG_BLANK, SEG_E);
  writeRaw(CLK3, DIO3, SEG_BLANK, SEG_BLANK, SEG_BLANK, SEG_BLANK);
  delay(2000);

  Serial.println("Step 5: Counter 0-9 on all displays");
  for (int i = 0; i <= 9; i++) {
    writeRaw(CLK1, DIO1, seg[i], seg[i], seg[i], seg[i]);
    writeRaw(CLK2, DIO2, seg[i], seg[i], seg[i], seg[i]);
    writeRaw(CLK3, DIO3, seg[i], seg[i], seg[i], seg[i]);
    delay(400);
  }

  Serial.println("Step 6: Decimal point demo (2.50, 7.50, 4.85)");
  writeRaw(CLK1, DIO1, SEG_BLANK, seg[2] | 0x80, seg[5], seg[0]);
  writeRaw(CLK2, DIO2, SEG_BLANK, seg[7] | 0x80, seg[5], seg[0]);
  writeRaw(CLK3, DIO3, SEG_BLANK, seg[4] | 0x80, seg[8], seg[5]);
  delay(3000);

  Serial.println("Loop done. Repeating...");
  delay(1000);
}
