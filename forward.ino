// Motor A pins
const int A_pin1 = 6;   // PWM
const int A_pin2 = 5;   // DIR

// Motor B pins
const int B_pin1 = 10;  // PWM
const int B_pin2 = 9;   // DIR

void setup() {
  pinMode(A_pin1, OUTPUT);
  pinMode(A_pin2, OUTPUT);
  pinMode(B_pin1, OUTPUT);
  pinMode(B_pin2, OUTPUT);

  Serial.begin(9600);
}

void loop() {

  int speed = 200;  // choose any PWM 0–255

  // Set direction FORWARD for both motors
  digitalWrite(A_pin2, HIGH);
  digitalWrite(B_pin2, HIGH);

  // Run forward for 10 seconds
  analogWrite(A_pin1, speed);
  analogWrite(B_pin1, speed);

  Serial.println("Running forward...");
  delay(10000);  // 10 seconds

  // Stop motors
  analogWrite(A_pin1, 0);
  analogWrite(B_pin1, 0);

  Serial.println("Stopped.");

  while(true);  // stop forever
}