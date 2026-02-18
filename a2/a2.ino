///////////////////////////////////////////////////// This is just the prof's original T1 code because I am trying to figure out the github and arduino

// Motor A pins (left motor)
const int A_pin1 = 6;   // one side of motor A
const int A_pin2 = 5;   // other side of motor A

// Motor B pins (right motor)
const int B_pin1 = 10;  // one side of motor B
const int B_pin2 = 9;   // other side of motor B

void setup() {
  pinMode(A_pin1, OUTPUT);
  pinMode(A_pin2, OUTPUT);
  pinMode(B_pin1, OUTPUT);
  pinMode(B_pin2, OUTPUT);
}

void loop() {

  int speed = 180;  // 0–255, adjust to taste

  // FORWARD on L9110:
  // One pin gets PWM (speed), the other stays LOW (direction)
  analogWrite(A_pin1, speed);   // PWM → acts like 1
  digitalWrite(A_pin2, LOW);    // 0

  analogWrite(B_pin1, speed);
  digitalWrite(B_pin2, LOW);

  delay(4000);  // go forward for 5 seconds

  // Stop both motors
  analogWrite(A_pin2, 0);
  analogWrite(B_pin2, 0);

  while(true);  // freeze forever
}