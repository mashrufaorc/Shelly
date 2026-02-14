// Motor A (left motor)
const int A_pin1 = 6;
const int A_pin2 = 5;

// Motor B (right motor)
const int B_pin1 = 10;
const int B_pin2 = 9;

void setup() {
  pinMode(A_pin1, OUTPUT);
  pinMode(A_pin2, OUTPUT);
  pinMode(B_pin1, OUTPUT);
  pinMode(B_pin2, OUTPUT);
}

void forward(int speedA, int speedB) {
  analogWrite(A_pin1, speedA);
  digitalWrite(A_pin2, LOW);

  analogWrite(B_pin1, speedB);
  digitalWrite(B_pin2, LOW);
}

void turnRight(int speed) {
  // Left motor forward
  digitalWrite(A_pin1, LOW);
  analogWrite(A_pin2, speed);

  // Right motor backward
  digitalWrite(B_pin1, LOW);
  analogWrite(B_pin2, speed);

  // // for turning left
  // analogWrite(B_pin1, speed);
  // digitalWrite(B_pin2, LOW);s
}

void stopMotors() {
  analogWrite(A_pin1, 0);
  analogWrite(B_pin1, 0);
  analogWrite(A_pin2, 0);
  analogWrite(B_pin2, 0);
}

void loop() {

  for (int i = 0; i < 1; i++) {

    // forward(210, 105);
    // delay(9500);

    // stopMotors();
    // delay(1600);

    // turnRight(180);
    // delay(1600);   // tune this carefully

    // stopMotors();
    // delay(1500);

    // forward(147, 102);
    // delay(8500);

    // stopMotors();
    // delay(600);

    // turnRight(180);
    // delay(900);   // tune this carefully

    // stopMotors();
    // delay(600);

    forward(152, 100);
    delay(8500);
    stopMotors();
    delay(1000);
    turnRight(180);
    delay(1600); // tune this carefully stopMotors(); // delay(1000);
  }
  stopMotors();
}