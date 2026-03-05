//motor pins
const int A_1B = 5;
const int A_1A = 6;
const int B_1B = 9;
const int B_1A = 10;

//sensor pins 
const int lineTrack = 2;  //line sensor
const int trigPin = 3;   //ultrasonic sensor pin
const int echoPin = 4;   //ultrasonic sensor pin
const int leftIR = 8;   //IR obstacle avoidance pin
const int rightIR = 7;   //IR obstacle avoidance pin

//motor speed
int speed = 160;

// note;we can change these constants as we see fit
const int SPEED_SEARCH   = 150;
const int SPEED_APPROACH = 120;
const int SPEED_PUSH     = 200;

const float DETECT_RANGE_CM = 40.0; // start approaching if ultrasonic sees something
const float VERIFY_RANGE_CM = 18.0; // start close-up verification below this (at 15 cm it will change from approach to verify)
const float FINAL_VERIFY_CM = 4.0;  // creep until about this far before reading IR. In VERIFY: it'llcreep forward until 8 cm, then stop and run isWhiteBoxClose().

// For the sensors: 0=detected, 1=no detect. By tuning the potentiometer, the IR for R and L can detect the white object withing 0-5 cm distance from the IR sensors
const int IR_DETECT_VALUE = 0;

// STATE Machine based on the ones the prof mentioned. 
enum State { SEARCH, APPROACH, VERIFY, PUSH, REJECT };
State state = SEARCH;

float readUltrasonic() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // timeout prevents hanging if no echo
  long duration = pulseIn(echoPin, HIGH, 25000); // 25ms ~ 4m
  if (duration == 0) return 999.0;
  float distance = duration / 58.0; // cm
  return distance;
}

bool isWhiteBoxClose() {
  
  Serial.println("Checking Box colour....");
  int LEFT_IR = digitalRead(leftIR);
  int RIGHT_IR = digitalRead(rightIR);

  // Case 1: both sensors detect white object --> object is centered RETURN TRUE to PUSH
  if (LEFT_IR == IR_DETECT_VALUE && RIGHT_IR == IR_DETECT_VALUE){
    Serial.println("Detected white object from BOTH sensors");
    return true;
  }
  //Case 2: left sensor detects --> object slightly left
  else if (LEFT_IR == IR_DETECT_VALUE && RIGHT_IR != IR_DETECT_VALUE){
    Serial.println("Detected white object from LEFT sensor. STEER LEFT");
    moveLeft(110);
    delay(80);
    return false;
  }
  //Case 3: right sensor detects --> object slightly right
  else if (LEFT_IR != IR_DETECT_VALUE && RIGHT_IR == IR_DETECT_VALUE){
    Serial.println("Detected white object from RIGHT sensor. STEER RIGHT");
    moveRight(110);
    delay(80);
    moveForward(0); //stop the motors
    return false;
  }
  //Case 4: Nothing detected --> object is likely to be black
  else {
    Serial.println("Detected object is NOT WHITE");
    return false;
  }


}

void setup() {
  Serial.begin(9600);

  pinMode(A_1B, OUTPUT);
  pinMode(A_1A, OUTPUT);
  pinMode(B_1B, OUTPUT);
  pinMode(B_1A, OUTPUT);

  pinMode(lineTrack, INPUT);

  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT);

  pinMode(leftIR, INPUT);
  pinMode(rightIR, INPUT);
}

void loop() {
  int lineColor = digitalRead(lineTrack); // 0=white floor, 1=black boundary
  float distance = readUltrasonic();

  // Boundary protection (we're prioritizing this)
  if (lineColor == 1) {
    Serial.println("Boundary detected -> move back and turn left");
    moveBackward(speed);
    delay(600);
    moveLeft(speed);
    delay(900);
    state = SEARCH;
    return;
  }

  // Reading IR (used for steering + verification)
  int left  = digitalRead(leftIR);   // 0=detected, 1=no obj
  int right = digitalRead(rightIR);  // 0=detected, 1=no obj

  switch (state) {
    // Roam forward (SEARCH), occasionally drift to scan
    case SEARCH:
      moveForward(SPEED_SEARCH);
      if (distance < DETECT_RANGE_CM) {
        Serial.println("Ultrasonic sensor detects something -> APPROACH");
        state = APPROACH;
      }
      break;

    // If object disappears, go back to SEARCH
    case APPROACH:
      Serial.print("APPROACH distance: ");
      Serial.println(distance);
      //we can add this in the report for how we handled edge cases. technically the boxes are stale and not moving but if it were live things in the boundary that continuosly changed position then it would ensure that the thing it's approaching is still there and didn't loose it
      // if (distance > DETECT_RANGE_CM + 20) {
      //   Serial.println("Lost object -> SEARCH");
      //   state = SEARCH;
      //   break;
      // }

      // If very close (this would mean it didn't loose the object), go verify
      if (distance <= VERIFY_RANGE_CM) {
        Serial.println("Close enough to the object-> VERIFY");
        state = VERIFY;
        break;
      }

      // Steering toward object using IR sensors (coarse)
      // If left sees it more, steer left; if right sees it more, steer right
      if (left == IR_DETECT_VALUE && right != IR_DETECT_VALUE) {
        // object more on left front -> turn slightly left while moving
        Serial.println("Approach: steer left");
        moveLeft(120);
        delay(80);
        moveForward(SPEED_APPROACH);
      } 
      else if (right == IR_DETECT_VALUE && left != IR_DETECT_VALUE) {
        Serial.println("Approach: steer right");
        moveRight(120);
        delay(80);
        moveForward(SPEED_APPROACH);
      } 
      else {
        moveForward(SPEED_APPROACH);
      }
      break;
    
    // Move to consistent final distance for reliable reflectivity check
    case VERIFY:
      distance = readUltrasonic();

      if (distance > FINAL_VERIFY_CM) {
        Serial.println("In state VERIFY; moving Forward");
        moveForward(80);
        delay(60);
        moveForward(0);
      } else {
        // stop motors briefly before reading values from sensorss
        moveForward(0);
        delay(150);

        bool white = isWhiteBoxClose();

        if (white) {
          Serial.println("VERIFY: WHITE box -> PUSH");
          state = PUSH;
        }
        else if (left != IR_DETECT_VALUE && right != IR_DETECT_VALUE) {
          Serial.println("VERIFY: BLACK box -> REJECT");
          state = REJECT;
        }
        else {
          Serial.println("VERIFY: Adjusting alignment");
        }
      }
      break;

    case PUSH:
      // Push forward until boundary is detected (handled by override at top)
      Serial.println("PUSHING...");
      moveForward(SPEED_PUSH);

      // If ultrasonic suddenly says "no object", stop pushing and search again (we can include in our edge case handling)
      if (distance > DETECT_RANGE_CM + 30) {
        Serial.println("Object gone -> SEARCH");
        state = SEARCH;
      }
      break;

    // Back away and turn away, then continue searching
    case REJECT:
      Serial.println("REJECT: retreat and turn left");
      moveBackward(190);
      delay(200);
      moveLeft(180);
      delay(900);
      state = SEARCH;
      break;
  }
}


void moveLeft(int speed) {
  analogWrite(A_1B, speed);
  analogWrite(A_1A, 0);
  analogWrite(B_1B, 0);
  analogWrite(B_1A, 0);
}

void moveRight(int speed) {
  analogWrite(A_1B, 0);
  analogWrite(A_1A, 0);
  analogWrite(B_1B, 0);
  analogWrite(B_1A, speed);
}

void moveForward(int speed) {
  analogWrite(A_1B, 0);
  analogWrite(A_1A, speed);
  analogWrite(B_1B, 0);
  analogWrite(B_1A, speed);
}

void moveBackward(int speed) {
  analogWrite(A_1B, speed);
  analogWrite(A_1A, 0);
  analogWrite(B_1B, speed);
  analogWrite(B_1A, 0);
}

void backRight(int speed){
  analogWrite(A_1B, speed);
  analogWrite(A_1A, 0);
  analogWrite(B_1B, speed / 2);
  analogWrite(B_1A, 0);
}

void backLeft(int speed) {
  analogWrite(A_1B, speed / 2);
  analogWrite(A_1A, 0);
  analogWrite(B_1B, speed);
  analogWrite(B_1A, 0);
}