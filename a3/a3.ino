#include <Arduino.h>
#include <math.h>

// motor pins
const int A_1B = 5;   
const int A_1A = 6;   
const int B_1B = 9;   
const int B_1A = 10;  

// ultrasonic sensor pins
const int TRIG_FRONT = 3;  
const int ECHO_FRONT = 4;  
const int TRIG_FR = 11;    
const int ECHO_FR = 12;    
const int TRIG_BR = 13;    
const int ECHO_BR = A0;    

// robot constants
const float DESIRED_WALL_DIST = 15.0;
const float SENSOR_SPACING_CM = 10.0;

const int BASE_SPEED = 115;
const int ADJUST_SPEED = 100;
const int TURN_SPEED = 115;

const float FRONT_WALL_THRESHOLD = 20.0;
const float RIGHT_WALL_LOST_THRESHOLD = 26.0;
const float RIGHT_WALL_REACQUIRE_THRESHOLD = 25.0;
const float HEADING_ADJUST_ENTER = 0.18;  // radians
const float HEADING_ADJUST_EXIT  = 0.07;  // radians

// PI gains for distance and heading error
float Kp_dist = 3.8;
float Ki_dist = 0.04;

float Kp_heading = 85.0;
float Ki_heading = 0.45;

// clamp the controller output so the robot does not overreact by using these values to set a range (similar to clip in prof example code)
const float MAX_CONTROL = 55.0;
const float MAX_INT_DIST = 25.0;
const float MAX_INT_HEAD = 0.50;

// since robot turns a tiny bit and buffers, set a minimum amount of time to make sure the robot doesnt stop turning too quickly
const unsigned long MIN_TURN_TIME_MS = 350;

// time between sensor triggers so they do not all go off at once and confuse the robot
const unsigned long SENSOR_INTERVAL_MS = 30;

// debugging  --> troubleshooting
const unsigned long PRINT_INTERVAL_MS = 120;

// using a filter to smooooooth where higher values acts faster, and lower acts smoother
const float FILTER_ALPHA = 0.45;

// our five states
enum State {
  FOLLOW_WALL,   // normal wall following
  TURN_LEFT,     // 90-degree left turn 
  TURN_RIGHT,    // 90-degree right turn 
  ADJUST_LEFT,   // minor left correction for slope
  ADJUST_RIGHT   // minor right correction for slope
};

// starting state
State state = FOLLOW_WALL;

// to differentiate between our initial readings and our smooooooooooth ones
struct Sensor {
  float raw;       // raw distance reading
  float filtered;  // smooooooothed reading
  bool valid;      // reading is valid or not
};

// sensor records for each ultrasonic sensor
Sensor frontSensor = {0, 0, false};
Sensor frSensor    = {0, 0, false};
Sensor brSensor    = {0, 0, false};

// theta = 0 means robot is parallel to the wall
float headingTheta = 0.0;
float wallDistance = DESIRED_WALL_DIST;
float eL = 0.0;   // lateral distance error
float eH = 0.0;   // heading error

// integral memory terms for PI control
float integralDist = 0.0;
float integralHead = 0.0;

// time variables
unsigned long lastControlTime = 0;
unsigned long lastSensorTime = 0;
unsigned long lastPrintTime = 0;
unsigned long stateStartTime = 0;

// what sensor reading we should look at first
int sensorTurn = 0;

// --> similar to professor's clip method in T06_CodeExample.py by keeping a float inside a safe range
float clampFloat(float value, float low, float high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

// --> similar to professor's clip method in T06_CodeExample.py by keeping an int inside a safe range
int clampInt(int value, int low, int high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

// debugging/what state are we in --> troubleshooting
const char* stateName(State s) {
  switch (s) {
    case FOLLOW_WALL: return "FOLLOW_WALL";
    case TURN_LEFT: return "TURN_LEFT";
    case TURN_RIGHT: return "TURN_RIGHT";
    case ADJUST_LEFT: return "ADJUST_LEFT";
    case ADJUST_RIGHT: return "ADJUST_RIGHT";
    default: return "UNKNOWN";
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
  analogWrite(A_1A, speed - 15);
  analogWrite(B_1B, 0);
  analogWrite(B_1A, speed + 5);
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

void stopMotors() {
  moveForward(0);
}

// do not let the speed numbers go beyond their limitis
void driveForwardDifferential(int leftSpeed, int rightSpeed) {
  leftSpeed = clampInt(leftSpeed, 0, 255);
  rightSpeed = clampInt(rightSpeed, 0, 255);
  analogWrite(A_1B, 0);
  analogWrite(A_1A, leftSpeed);
  analogWrite(B_1B, 0);
  analogWrite(B_1A, rightSpeed);
}

// reading an ultrasonic sensor in cm
float readUltrasonicCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // timeout prevents hanging if no echo and if 0, bad reading so pretend its invalid
  unsigned long duration = pulseIn(echoPin, HIGH, 25000);
  if (duration == 0) return -1.0;
  float distance = duration / 58.0; // cm

  // do not let the distance go crazyyyyyyyyyyy
  if (distance < 2.0 || distance > 250.0) {
    return -1.0;
  }

  return distance;

}

// smooooooooth the sensor
void updateSensor(Sensor &s, float newValue) {
  // If the reading is invalid, mark it invalid and leave
  if (newValue < 0) {
    s.valid = false;
    return;
  }

  // raw reading
  s.raw = newValue;

  // use good reading
  if (!s.valid) {
    s.filtered = newValue;
  } else {
    // the actual smoooooothing function
    s.filtered = FILTER_ALPHA * newValue + (1.0 - FILTER_ALPHA) * s.filtered;
  }

  s.valid = true;
}

// spacing out sensor readings
void serviceSensors() {
  unsigned long now = millis();
  if (now - lastSensorTime < SENSOR_INTERVAL_MS) {
    return;
  }

  lastSensorTime = now;

  // one sensor at a time
  if (sensorTurn == 0) {
    updateSensor(frontSensor, readUltrasonicCM(TRIG_FRONT, ECHO_FRONT));
  } else if (sensorTurn == 1) {
    updateSensor(frSensor, readUltrasonicCM(TRIG_FR, ECHO_FR));
  } else {
    updateSensor(brSensor, readUltrasonicCM(TRIG_BR, ECHO_BR));
  }

  // move it move it to next sensor
  sensorTurn = (sensorTurn + 1) % 3;
}

bool rightWallVisible() {
  return frSensor.valid &&
         brSensor.valid &&
         frSensor.filtered < RIGHT_WALL_LOST_THRESHOLD &&
         brSensor.filtered < RIGHT_WALL_LOST_THRESHOLD;
}
bool rightWallLost() {
  bool frLost = (!frSensor.valid) || (frSensor.filtered > RIGHT_WALL_LOST_THRESHOLD);
  bool brLost = (!brSensor.valid) || (brSensor.filtered > RIGHT_WALL_LOST_THRESHOLD);
  return frLost && brLost;
}
bool frontBlocked() {
  return frontSensor.valid && frontSensor.filtered <= FRONT_WALL_THRESHOLD;
}
bool frontClear() {
  return (!frontSensor.valid) || (frontSensor.filtered > FRONT_WALL_THRESHOLD + 7.0);
}

void estimateWall() {
  if (!(frSensor.valid && brSensor.valid)) {
    return;
  }
  float dfr = frSensor.filtered;
  float dbr = brSensor.filtered;
  headingTheta = atan2((dfr - dbr), SENSOR_SPACING_CM);
  wallDistance = (dfr + dbr) / 2.0;
}

// resetting the integral memory
void resetIntegrals() {
  integralDist = 0.0;
  integralHead = 0.0;
}

// PI controller logic calculations
float computePIControl(float dt) {
  estimateWall();

  eL = wallDistance - DESIRED_WALL_DIST;
  eH = headingTheta;
  integralDist += eL * dt;
  integralHead += eH * dt;

  // similar to professor's clip method in T06
  integralDist = clampFloat(integralDist, -MAX_INT_DIST, MAX_INT_DIST);
  integralHead = clampFloat(integralHead, -MAX_INT_HEAD, MAX_INT_HEAD);

  // PI controller for angular correction from prof code T06 where omega = -Kp,H eH - Ki,H ∫eH - Kp,L eL - Ki,L ∫eL
  float omegaCmd =
      -Kp_heading * eH
      -Ki_heading * integralHead
      -Kp_dist * eL
      -Ki_dist * integralDist;

  // don't go crazy with the omega
  omegaCmd = clampFloat(omegaCmd, -MAX_CONTROL, MAX_CONTROL);

  return omegaCmd;

}

// flip between states and when it turns, forget the old integral info so the robot does not oscillate
void setState(State newState) {
  if (state == newState) return;
  state = newState;
  stateStartTime = millis();
  if (state == TURN_LEFT || state == TURN_RIGHT) {
    resetIntegrals();
  }
}

// length of current state
unsigned long timeInState() {
  return millis() - stateStartTime;
}


// wall following using PI steering by computing dt in seconds since last state
void doFollowWall(int baseSpeed) {
  unsigned long now = millis();
  float dt;
  if (lastControlTime == 0) {
    dt = 0.02;
  } else {
    dt = (now - lastControlTime) / 1000.0;
  }
  lastControlTime = now;

  dt = clampFloat(dt, 0.005, 0.08);
  float control = computePIControl(dt);

  // make one wheel faster and the other one slower as needed
  int leftSpeed  = (int)(baseSpeed + control);
  int rightSpeed = (int)(baseSpeed - control);

  dt = clampFloat(dt, 0.005, 0.08);
  leftSpeed  = clampInt(leftSpeed, 0, 255);
  rightSpeed = clampInt(rightSpeed, 0, 255);

  driveForwardDifferential(leftSpeed, rightSpeed);
}


void doTurnLeft() {
  moveLeft(TURN_SPEED);
}

void doTurnRight() {
  moveRight(TURN_SPEED);
}

void doAdjustLeft() {
  doFollowWall(ADJUST_SPEED);
}

void doAdjustRight() {
  doFollowWall(ADJUST_SPEED);
}


// when to switch states
void updateFSM() {
  estimateWall();

  switch (state) {

    case FOLLOW_WALL:
    if (timeInState() < 10){
      moveForward(255);
    }
      
      if (frontBlocked()) {
        //moveForward(200);
        setState(TURN_LEFT);
      }
      else if (rightWallLost()) {
         //moveForward(200);
        setState(TURN_RIGHT);
      }
      else if (rightWallVisible()) {
        if (headingTheta > HEADING_ADJUST_ENTER) {
          setState(ADJUST_RIGHT);
        } else if (headingTheta < -HEADING_ADJUST_ENTER) {
          setState(ADJUST_LEFT);
        }
      }
      break;

    case ADJUST_LEFT:
      if (frontBlocked()) {
        setState(TURN_LEFT);
      } else if (rightWallLost()) {
        setState(TURN_RIGHT);
      } else if (fabs(headingTheta) < HEADING_ADJUST_EXIT) {
        setState(FOLLOW_WALL);
      }
      break;

    case ADJUST_RIGHT:
      if (frontBlocked()) {
        setState(TURN_LEFT);
      } else if (rightWallLost()) {
        setState(TURN_RIGHT);
      } else if (fabs(headingTheta) < HEADING_ADJUST_EXIT) {
        setState(FOLLOW_WALL);
      }
      break;

    case TURN_LEFT:
      if (timeInState() < 10){
      moveLeft(255);
    }
      if (timeInState() > MIN_TURN_TIME_MS &&
          frontClear() &&
          rightWallVisible()) {

        if (fabs(headingTheta) < HEADING_ADJUST_EXIT) {
          setState(FOLLOW_WALL);
        } else if (headingTheta > 0) {
          setState(ADJUST_RIGHT);
        } else {
          setState(ADJUST_LEFT);
        }
      }
      break;

    case TURN_RIGHT:
      if (timeInState() > MIN_TURN_TIME_MS &&
          frSensor.valid && brSensor.valid &&
          frSensor.filtered < RIGHT_WALL_REACQUIRE_THRESHOLD &&
          brSensor.filtered < RIGHT_WALL_REACQUIRE_THRESHOLD) {

        if (fabs(headingTheta) < HEADING_ADJUST_EXIT) {
          setState(FOLLOW_WALL);
        } else if (headingTheta > 0) {
          setState(ADJUST_RIGHT);
        } else {
          setState(ADJUST_LEFT);
        }
      }
      break;
  }
}

// what the states do
void runState() {
  switch (state) {
    case FOLLOW_WALL:
      doFollowWall(BASE_SPEED);
      break;

    case TURN_LEFT:
      doTurnLeft();
      break;

    case TURN_RIGHT:
      doTurnRight();
      break;

    case ADJUST_LEFT:
      doAdjustLeft();
      break;

    case ADJUST_RIGHT:
      doAdjustRight();
      break;
  }
}

// debugging / print statements --> troubleshooting
void printDebug() {
  unsigned long now = millis();
  if (now - lastPrintTime < PRINT_INTERVAL_MS) {
    return;
  }
  lastPrintTime = now;

  Serial.print("STATE: ");
  Serial.print(stateName(state));

  Serial.print(" | F=");
  if (frontSensor.valid) Serial.print(frontSensor.filtered, 1);
  else Serial.print("NA");

  Serial.print(" | FR=");
  if (frSensor.valid) Serial.print(frSensor.filtered, 1);
  else Serial.print("NA");

  Serial.print(" | BR=");
  if (brSensor.valid) Serial.print(brSensor.filtered, 1);
  else Serial.print("NA");

  Serial.print(" | d=");
  Serial.print(wallDistance, 2);

  Serial.print(" | theta(deg)=");
  Serial.print(headingTheta * 180.0 / PI, 2);

  Serial.print(" | eL=");
  Serial.print(eL, 2);

  Serial.print(" | eH=");
  Serial.print(eH, 3);

  Serial.println();
}

// pin setup

void setup() {
  // serial monitor for debugging --> troubleshooting
  Serial.begin(115200);

  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_FR, OUTPUT);
  pinMode(ECHO_FR, INPUT);
  pinMode(TRIG_BR, OUTPUT);
  pinMode(ECHO_BR, INPUT);

  pinMode(A_1B, OUTPUT);
  pinMode(A_1A, OUTPUT);
  pinMode(B_1B, OUTPUT);
  pinMode(B_1A, OUTPUT);

  stopMotors();
  delay(500);

  // start by following the wall
  state = FOLLOW_WALL;
  stateStartTime = millis();
  lastControlTime = millis();

  Serial.println("Wall follower started.");
}

void loop() {
  // update one sensor at a time
  serviceSensors();
  // when to switch states
  updateFSM();
  // what state does
  runState();
  // Print debug information
  printDebug();
}