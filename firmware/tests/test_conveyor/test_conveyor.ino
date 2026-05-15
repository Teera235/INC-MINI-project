#define PIN_MOTOR_ENA 5
#define PIN_MOTOR_IN1 4
#define PIN_MOTOR_IN2 3

void motorForward(int speed) {
  digitalWrite(PIN_MOTOR_IN1, HIGH);
  digitalWrite(PIN_MOTOR_IN2, LOW);
  analogWrite(PIN_MOTOR_ENA, speed);
}

void motorBackward(int speed) {
  digitalWrite(PIN_MOTOR_IN1, LOW);
  digitalWrite(PIN_MOTOR_IN2, HIGH);
  analogWrite(PIN_MOTOR_ENA, speed);
}

void motorStop() {
  analogWrite(PIN_MOTOR_ENA, 0);
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_MOTOR_ENA, OUTPUT);
  pinMode(PIN_MOTOR_IN1, OUTPUT);
  pinMode(PIN_MOTOR_IN2, OUTPUT);
  motorStop();
  delay(500);

  Serial.println("=== CONVEYOR TEST ===");
  Serial.println("Commands:");
  Serial.println("  f<speed>  forward (e.g. f120)");
  Serial.println("  b<speed>  backward");
  Serial.println("  s         stop");
  Serial.println("  T         transfer test (forward 4s)");
  Serial.println("  D         dispatch test (15 step-cycles + final)");
}

void runDispatch() {
  Serial.println("DISPATCH START");
  for (int i = 0; i < 15; i++) {
    Serial.print("Cycle ");
    Serial.println(i + 1);
    motorBackward(100);
    delay(250);
    motorStop();
    delay(300);
    motorForward(100);
    delay(250);
    motorStop();
    delay(300);
  }
  Serial.println("Final slow backward 2s");
  motorBackward(70);
  delay(2000);
  motorStop();
  Serial.println("DISPATCH DONE");
}

void runTransfer() {
  Serial.println("TRANSFER START (forward 4s @ PWM 120)");
  motorForward(120);
  delay(4000);
  motorStop();
  Serial.println("TRANSFER DONE");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'f') {
      int s = Serial.parseInt();
      s = constrain(s, 0, 255);
      motorForward(s);
      Serial.print("FWD speed=");
      Serial.println(s);
    } else if (c == 'b') {
      int s = Serial.parseInt();
      s = constrain(s, 0, 255);
      motorBackward(s);
      Serial.print("BWD speed=");
      Serial.println(s);
    } else if (c == 's') {
      motorStop();
      Serial.println("STOP");
    } else if (c == 'T') {
      runTransfer();
    } else if (c == 'D') {
      runDispatch();
    }
  }
}
