/////////////////////////////////// so this is the code from sunfounder that makes the robot follow the line, but we want to avoid it

const int A_1B = 5;
const int A_1A = 6;
const int B_1B = 9;
const int B_1A = 10;

const int lineTrack = 2;

void setup() {
  Serial.begin(9600);

  //motor
  pinMode(A_1B, OUTPUT);
  pinMode(A_1A, OUTPUT);
  pinMode(B_1B, OUTPUT);
  pinMode(B_1A, OUTPUT);
  //line track
  pinMode(lineTrack, INPUT);
}

void loop() {

  int speed = 150;
  
  int lineColor = digitalRead(lineTrack); // 0:white  1:black
  Serial.println(lineColor); //print on the serial monitor

  // this is the original code - telling the robot to micro turn and continue forward everytime it hits the tape
  if (lineColor) {
  moveLeft(speed);
  } else {
    moveRight(speed);
  }

  // so i am thinking, if it does not detect tape, go forward
  // and if it does detect tape, maybe we can place one sensor on the right so it then turns left?
  //if (lineColor) {
  //  moveLeft(speed);
  //} else {
  //  moveForward(speed);
  //}
}

}
void moveLeft(int speed) {
  analogWrite(A_1B, 0);
  analogWrite(A_1A, speed);
  analogWrite(B_1B, 0);
  analogWrite(B_1A, 0);
}

void moveRight(int speed) {
  analogWrite(A_1B, 0);
  analogWrite(A_1A, 0);
  analogWrite(B_1B, speed);
  analogWrite(B_1A, 0);
}

void moveForward(int speed) {
  analogWrite(A_pin1, speed);
  digitalWrite(A_pin2, LOW);
  analogWrite(B_pin1, speed);
  digitalWrite(B_pin2, LOW);
}
