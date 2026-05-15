#include <Servo.h>

#define PIN_SERVO_X 44
#define PIN_SERVO_Y 45

Servo servoX;
Servo servoY;

void sweepX() {
  Serial.println("Sweeping X 60 -> 120 -> 60");
  for (int a = 60; a <= 120; a++) {
    servoX.write(a);
    delay(20);
  }
  delay(300);
  for (int a = 120; a >= 60; a--) {
    servoX.write(a);
    delay(20);
  }
  servoX.write(90);
  Serial.println("X back to 90");
}

void rotateY(int targetDeg) {
  static int currentDeg = 0;
  Serial.print("Rotating Y from ");
  Serial.print(currentDeg);
  Serial.print(" to ");
  Serial.println(targetDeg);
  int step = (targetDeg > currentDeg) ? 1 : -1;
  while (currentDeg != targetDeg) {
    currentDeg += step;
    servoY.writeMicroseconds(map(currentDeg, 0, 180, 1000, 2000));
    delay(15);
  }
  delay(500);
}

void setup() {
  Serial.begin(115200);
  servoX.attach(PIN_SERVO_X);
  servoY.attach(PIN_SERVO_Y);
  servoX.write(90);
  servoY.writeMicroseconds(1000);
  delay(1000);

  Serial.println("=== SERVO TEST ===");
  Serial.println("Commands:");
  Serial.println("  x<deg>   X to angle (e.g. x90)");
  Serial.println("  y<deg>   Y to angle 0/90/180");
  Serial.println("  S        full sweep cycle");
  Serial.println("  H        home (X=90, Y=0)");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'x') {
      int a = Serial.parseInt();
      a = constrain(a, 0, 180);
      servoX.write(a);
      Serial.print("X=");
      Serial.println(a);
    } else if (c == 'y') {
      int a = Serial.parseInt();
      a = constrain(a, 0, 180);
      rotateY(a);
    } else if (c == 'S') {
      Serial.println("Full sweep test");
      rotateY(0);
      sweepX();
      rotateY(90);
      sweepX();
      rotateY(180);
      sweepX();
      rotateY(0);
      Serial.println("Sweep done");
    } else if (c == 'H') {
      servoX.write(90);
      rotateY(0);
      Serial.println("Home");
    }
  }
}
