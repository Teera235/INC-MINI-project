const int PIN = A0;
const int SAMPLE_COUNT = 200;
const float ALPHA = 0.05;

float filteredPP = 0;
bool first = true;

void setup() {
  Serial.begin(115200);
}

void loop() {
  int maxVal = 0;
  int minVal = 1023;

  for (int i = 0; i < SAMPLE_COUNT; i++) {
    int v = analogRead(PIN);
    if (v > maxVal) maxVal = v;
    if (v < minVal) minVal = v;
  }

  float pp = (float)(maxVal - minVal);

  if (first) {
    filteredPP = pp;
    first = false;
  } else {
    filteredPP = ALPHA * pp + (1.0 - ALPHA) * filteredPP;
  }

  Serial.println(filteredPP, 1);
  delay(20);
}
