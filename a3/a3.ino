<<<<<<< HEAD
/*
  ============================================================
  CPS607 Assignment 3 - Wall Following Robot
  ============================================================

  What this code does in simple words:
  - The robot follows a wall on its RIGHT side
  - It tries to stay 15 cm away from the wall
  - It uses 3 ultrasonic sensors:
      1) Front sensor        -> sees a wall in front
      2) Front-right sensor  -> helps measure wall angle/distance
      3) Back-right sensor   -> helps measure wall angle/distance
  - It uses a PI controller to steer smoothly
  - It uses a finite state machine (FSM) for:
      FOLLOW_WALL
      TURN_LEFT
      TURN_RIGHT
      ADJUST_LEFT
      ADJUST_RIGHT

  Assignment 3 asks for:
  - 15 cm desired distance
  - PI control
  - FSM for corners
  - smooth reaction to wall slope changes
  - non-blocking sensor logic + interference mitigation

  ------------------------------------------------------------
  IMPORTANT:
  ------------------------------------------------------------
  1) Right-side sensor pins are placeholders at the moment.
  2) NEED to tune the gains and timing on the real robot.
*/

// Include Arduino basics
#include <Arduino.h>
#include <math.h>

// ============================================================
// 1. PIN SETUP
// ============================================================

// ------------------------------------------------------------
// Motor pins
// ------------------------------------------------------------
const int A_1B = 5;   // left motor backward direction pin
const int A_1A = 6;   // left motor forward direction pin
const int B_1B = 9;   // right motor backward direction pin
const int B_1A = 10;  // right motor forward direction pin

// ------------------------------------------------------------
// Front ultrasonic sensor
// ------------------------------------------------------------
const int TRIG_FRONT = 3;  // front sensor trigger
const int ECHO_FRONT = 4;  // front sensor echo

// ------------------------------------------------------------
// Right-side ultrasonic sensors
// These are placeholders because Assignment 2 only had one ultrasonic
// Need to update based on Shelly
// ------------------------------------------------------------
const int TRIG_FR = 11;    // front-right ultrasonic trigger
const int ECHO_FR = 12;    // front-right ultrasonic echo

const int TRIG_BR = 13;    // back-right ultrasonic trigger
const int ECHO_BR = A0;    // back-right ultrasonic echo

// ============================================================
// 2. CONTROL / ROBOT CONSTANTS
// ============================================================

// Desired distance from wall in cm
// Assignment 3 explicitly requires 15 cm
const float DESIRED_WALL_DIST = 15.0;

// Distance between the two right-side sensors in cm
// Measure this on your real robot from sensor center to sensor center
const float SENSOR_SPACING_CM = 10.0;

// Forward base speed for normal wall following
const int BASE_SPEED = 115;

// Slightly slower speed for fine adjustment
const int ADJUST_SPEED = 100;

// Speed used for in-place corner turns
const int TURN_SPEED = 115;

// PI gains for distance error
float Kp_dist = 3.8;
float Ki_dist = 0.04;

// PI gains for heading error
float Kp_heading = 85.0;
float Ki_heading = 0.45;

// Clamp the controller output so the robot does not overreact
const float MAX_CONTROL = 55.0;

// Integral anti-windup limits
const float MAX_INT_DIST = 25.0;
const float MAX_INT_HEAD = 0.50;

// Front wall threshold for left turn
// If the front sensor sees a wall close enough, start turning left
const float FRONT_WALL_THRESHOLD = 15.0;

// If both right-side sensors read too large, we assume the robot lost the wall
const float RIGHT_WALL_LOST_THRESHOLD = 35.0;

// Once turning right, if both right-side sensors see the wall again under this,
// we say the wall has been reacquired
const float RIGHT_WALL_REACQUIRE_THRESHOLD = 25.0;

// Heading thresholds for adjustment states
const float HEADING_ADJUST_ENTER = 0.18;  // radians
const float HEADING_ADJUST_EXIT  = 0.07;  // radians

// Minimum turn time so the robot does not leave a turn too early
const unsigned long MIN_TURN_TIME_MS = 350;

// Time between sensor triggers
// We trigger one ultrasonic at a time to reduce interference
const unsigned long SENSOR_INTERVAL_MS = 35;

// Debug print timing
const unsigned long PRINT_INTERVAL_MS = 120;

// Filter smoothing factor
// Higher = reacts faster, lower = smoother
const float FILTER_ALPHA = 0.45;

// ============================================================
// 3. STATE MACHINE
// ============================================================

// These are the 5 high-level robot states
enum State {
  FOLLOW_WALL,   // normal wall following
  TURN_LEFT,     // 90-degree left turn when front wall appears
  TURN_RIGHT,    // 90-degree right turn when wall on right disappears
  ADJUST_LEFT,   // minor left correction for slope/angle changes
  ADJUST_RIGHT   // minor right correction for slope/angle changes
};

// Start in normal wall following mode
State state = FOLLOW_WALL;

// Save when the current state started
unsigned long stateStartTime = 0;

// ============================================================
// 4. SENSOR STORAGE
// ============================================================

// A small structure to store each sensor reading neatly
struct Sensor {
  float raw;       // newest raw distance reading
  float filtered;  // smoothed reading
  bool valid;      // whether the reading is valid
};

// Create one sensor record for each ultrasonic
=======
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
>>>>>>> 66e489883efd531756123948bb19e18249df7d51
Sensor frontSensor = {0, 0, false};
Sensor frSensor    = {0, 0, false};
Sensor brSensor    = {0, 0, false};

<<<<<<< HEAD
// ============================================================
// 5. CONTROL VARIABLES
// ============================================================

// Estimated heading angle relative to the wall
// theta = 0 means robot is parallel to the wall
float headingTheta = 0.0;

// Estimated lateral distance from robot to wall
float wallDistance = DESIRED_WALL_DIST;

// Error terms 
float eL = 0.0;   // lateral distance error
float eH = 0.0;   // heading error

// Integral memory terms for PI control
float integralDist = 0.0;
float integralHead = 0.0;

// Time trackers
unsigned long lastControlTime = 0;
unsigned long lastSensorTime = 0;
unsigned long lastPrintTime = 0;

// Which sensor gets triggered next
// 0 = front, 1 = front-right, 2 = back-right
int sensorTurn = 0;

// ============================================================
// 6. SMALL HELPER FUNCTIONS
// ============================================================

// Keep a float inside a safe range
=======
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
>>>>>>> 66e489883efd531756123948bb19e18249df7d51
float clampFloat(float value, float low, float high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

<<<<<<< HEAD
// Keep an int inside a safe range
=======
// --> similar to professor's clip method in T06_CodeExample.py by keeping an int inside a safe range
>>>>>>> 66e489883efd531756123948bb19e18249df7d51
int clampInt(int value, int low, int high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

<<<<<<< HEAD
// Return a readable name for the current state
=======
// debugging/what state are we in --> troubleshooting
>>>>>>> 66e489883efd531756123948bb19e18249df7d51
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

<<<<<<< HEAD
// ============================================================
// 7. MOTOR FUNCTIONS
// ============================================================

// Move the robot left by pivoting
void moveLeft(int speed) {
  analogWrite(A_1B, speed);
  analogWrite(A_1A, 0);
=======
void moveLeft(int speed) {
  analogWrite(A_1B, 0);
  analogWrite(A_1A, speed);
>>>>>>> 66e489883efd531756123948bb19e18249df7d51
  analogWrite(B_1B, 0);
  analogWrite(B_1A, 0);
}

<<<<<<< HEAD
// Move the robot right by pivoting
void moveRight(int speed) {
  analogWrite(A_1B, 0);
  analogWrite(A_1A, 0);
  analogWrite(B_1B, 0);
  analogWrite(B_1A, speed);
}

// Move both wheels forward
=======
void moveRight(int speed) {
  analogWrite(A_1B, 0);
  analogWrite(A_1A, speed - 15);
  analogWrite(B_1B, 0);
  analogWrite(B_1A, speed + 5);
}

>>>>>>> 66e489883efd531756123948bb19e18249df7d51
void moveForward(int speed) {
  analogWrite(A_1B, 0);
  analogWrite(A_1A, speed);
  analogWrite(B_1B, 0);
  analogWrite(B_1A, speed);
}

<<<<<<< HEAD
// Move both wheels backward
=======
>>>>>>> 66e489883efd531756123948bb19e18249df7d51
void moveBackward(int speed) {
  analogWrite(A_1B, speed);
  analogWrite(A_1A, 0);
  analogWrite(B_1B, speed);
  analogWrite(B_1A, 0);
}

<<<<<<< HEAD
// Stop motors
=======
>>>>>>> 66e489883efd531756123948bb19e18249df7d51
void stopMotors() {
  moveForward(0);
}

<<<<<<< HEAD
// This lets us set different left and right forward speeds
// Useful for PI steering during wall following
void driveForwardDifferential(int leftSpeed, int rightSpeed) {
  leftSpeed = clampInt(leftSpeed, 0, 255);
  rightSpeed = clampInt(rightSpeed, 0, 255);

  // left motor forward
  analogWrite(A_1B, 0);
  analogWrite(A_1A, leftSpeed);

  // right motor forward
=======
// do not let the speed numbers go beyond their limitis
void driveForwardDifferential(int leftSpeed, int rightSpeed) {
  leftSpeed = clampInt(leftSpeed, 0, 255);
  rightSpeed = clampInt(rightSpeed, 0, 255);
  analogWrite(A_1B, 0);
  analogWrite(A_1A, leftSpeed);
>>>>>>> 66e489883efd531756123948bb19e18249df7d51
  analogWrite(B_1B, 0);
  analogWrite(B_1A, rightSpeed);
}

<<<<<<< HEAD
// ============================================================
// 8. ULTRASONIC SENSOR FUNCTIONS
// ============================================================

// Read one ultrasonic sensor in cm
float readUltrasonicCM(int trigPin, int echoPin) {
  // Make sure trigger starts low
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Send the trigger pulse
=======
// reading an ultrasonic sensor in cm
float readUltrasonicCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
>>>>>>> 66e489883efd531756123948bb19e18249df7d51
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

<<<<<<< HEAD
  // Wait for echo, but don't wait forever
  unsigned long duration = pulseIn(echoPin, HIGH, 18000UL);

  // If nothing came back, treat it as invalid
  if (duration == 0) {
    return -1.0;
  }

  // Convert microseconds to cm
  float distance = duration * 0.0343 / 2.0;

  // Reject crazy values
=======
  // timeout prevents hanging if no echo and if 0, bad reading so pretend its invalid
  unsigned long duration = pulseIn(echoPin, HIGH, 25000);
  if (duration == 0) return -1.0;
  float distance = duration / 58.0; // cm

  // do not let the distance go crazyyyyyyyyyyy
>>>>>>> 66e489883efd531756123948bb19e18249df7d51
  if (distance < 2.0 || distance > 250.0) {
    return -1.0;
  }

  return distance;
<<<<<<< HEAD
}

// Update sensor record with smoothing
=======

}

// smooooooooth the sensor
>>>>>>> 66e489883efd531756123948bb19e18249df7d51
void updateSensor(Sensor &s, float newValue) {
  // If the reading is invalid, mark it invalid and leave
  if (newValue < 0) {
    s.valid = false;
    return;
  }

<<<<<<< HEAD
  // Save raw reading
  s.raw = newValue;

  // First good reading: use it directly
  if (!s.valid) {
    s.filtered = newValue;
  } else {
    // After that, smooth it with exponential filtering
=======
  // raw reading
  s.raw = newValue;

  // use good reading
  if (!s.valid) {
    s.filtered = newValue;
  } else {
    // the actual smoooooothing function
>>>>>>> 66e489883efd531756123948bb19e18249df7d51
    s.filtered = FILTER_ALPHA * newValue + (1.0 - FILTER_ALPHA) * s.filtered;
  }

  s.valid = true;
}

<<<<<<< HEAD
// This function spreads out ultrasonic triggers over time
// That helps avoid the side sensors hearing each other
void serviceSensors() {
  unsigned long now = millis();

  // Only do a sensor read every SENSOR_INTERVAL_MS
=======
// spacing out sensor readings
void serviceSensors() {
  unsigned long now = millis();
>>>>>>> 66e489883efd531756123948bb19e18249df7d51
  if (now - lastSensorTime < SENSOR_INTERVAL_MS) {
    return;
  }

  lastSensorTime = now;

<<<<<<< HEAD
  // Read only one sensor each time
=======
  // one sensor at a time
>>>>>>> 66e489883efd531756123948bb19e18249df7d51
  if (sensorTurn == 0) {
    updateSensor(frontSensor, readUltrasonicCM(TRIG_FRONT, ECHO_FRONT));
  } else if (sensorTurn == 1) {
    updateSensor(frSensor, readUltrasonicCM(TRIG_FR, ECHO_FR));
  } else {
    updateSensor(brSensor, readUltrasonicCM(TRIG_BR, ECHO_BR));
  }

<<<<<<< HEAD
  // Move to next sensor for next loop
  sensorTurn = (sensorTurn + 1) % 3;
}

// ============================================================
// 9. WALL GEOMETRY ESTIMATION
// ============================================================

// Check if both right wall sensors are giving usable data
=======
  // move it move it to next sensor
  sensorTurn = (sensorTurn + 1) % 3;
}

>>>>>>> 66e489883efd531756123948bb19e18249df7d51
bool rightWallVisible() {
  return frSensor.valid &&
         brSensor.valid &&
         frSensor.filtered < RIGHT_WALL_LOST_THRESHOLD &&
         brSensor.filtered < RIGHT_WALL_LOST_THRESHOLD;
}
<<<<<<< HEAD

// Check if the robot has lost the wall on the right
=======
>>>>>>> 66e489883efd531756123948bb19e18249df7d51
bool rightWallLost() {
  bool frLost = (!frSensor.valid) || (frSensor.filtered > RIGHT_WALL_LOST_THRESHOLD);
  bool brLost = (!brSensor.valid) || (brSensor.filtered > RIGHT_WALL_LOST_THRESHOLD);
  return frLost && brLost;
}
<<<<<<< HEAD

// Check if there is a wall in front
bool frontBlocked() {
  return frontSensor.valid && frontSensor.filtered <= FRONT_WALL_THRESHOLD;
}

// Check if front is clear enough after a turn
=======
bool frontBlocked() {
  return frontSensor.valid && frontSensor.filtered <= FRONT_WALL_THRESHOLD;
}
>>>>>>> 66e489883efd531756123948bb19e18249df7d51
bool frontClear() {
  return (!frontSensor.valid) || (frontSensor.filtered > FRONT_WALL_THRESHOLD + 7.0);
}

<<<<<<< HEAD
// Estimate wall distance and heading from the two right sensors
void estimateWall() {
  // Need both right sensors to estimate angle properly
  if (!(frSensor.valid && brSensor.valid)) {
    return;
  }

  float dfr = frSensor.filtered;
  float dbr = brSensor.filtered;

  // Heading estimate:
  // if front-right is farther than back-right, robot is angled away from wall
  headingTheta = atan2((dfr - dbr), SENSOR_SPACING_CM);

  // Simple wall distance estimate:
  // average of the two side distances
  wallDistance = (dfr + dbr) / 2.0;
}

// ============================================================
// 10. PI CONTROLLER
// ============================================================

// Reset integral memory when useful
=======
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
>>>>>>> 66e489883efd531756123948bb19e18249df7d51
void resetIntegrals() {
  integralDist = 0.0;
  integralHead = 0.0;
}

<<<<<<< HEAD
// Compute steering command from distance + heading error
float computePIControl(float dt) {
  // Update wall geometry first
  estimateWall();

  // Lecture uses:
  // eL = d - d_des
  // eH = theta
  // and omega uses both errors
  // We follow that sign style here
  eL = wallDistance - DESIRED_WALL_DIST;
  eH = headingTheta;

  // Accumulate integral terms
  integralDist += eL * dt;
  integralHead += eH * dt;

  // Anti-windup clamps
  integralDist = clampFloat(integralDist, -MAX_INT_DIST, MAX_INT_DIST);
  integralHead = clampFloat(integralHead, -MAX_INT_HEAD, MAX_INT_HEAD);

  // PI controller for angular correction
  // Lecture idea:
  // omega = -Kp,H eH - Ki,H ∫eH - Kp,L eL - Ki,L ∫eL
=======
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
>>>>>>> 66e489883efd531756123948bb19e18249df7d51
  float omegaCmd =
      -Kp_heading * eH
      -Ki_heading * integralHead
      -Kp_dist * eL
      -Ki_dist * integralDist;

<<<<<<< HEAD
  // Prevent extreme steering
  omegaCmd = clampFloat(omegaCmd, -MAX_CONTROL, MAX_CONTROL);

  return omegaCmd;
}

// ============================================================
// 11. STATE CHANGING
// ============================================================

// Change robot state safely
void setState(State newState) {
  if (state == newState) return;

  state = newState;
  stateStartTime = millis();

  // Reset integrals when entering a big turn
=======
  // don't go crazy with the omega
  omegaCmd = clampFloat(omegaCmd, -MAX_CONTROL, MAX_CONTROL);

  return omegaCmd;

}

// flip between states and when it turns, forget the old integral info so the robot does not oscillate
void setState(State newState) {
  if (state == newState) return;
  state = newState;
  stateStartTime = millis();
>>>>>>> 66e489883efd531756123948bb19e18249df7d51
  if (state == TURN_LEFT || state == TURN_RIGHT) {
    resetIntegrals();
  }
}

<<<<<<< HEAD
// How long we have been in the current state
=======
// length of current state
>>>>>>> 66e489883efd531756123948bb19e18249df7d51
unsigned long timeInState() {
  return millis() - stateStartTime;
}

<<<<<<< HEAD
// ============================================================
// 12. ACTIONS FOR EACH STATE
// ============================================================

// Normal wall following using PI steering
void doFollowWall(int baseSpeed) {
  unsigned long now = millis();

  // Compute dt in seconds
=======

// wall following using PI steering by computing dt in seconds since last state
void doFollowWall(int baseSpeed) {
  unsigned long now = millis();
>>>>>>> 66e489883efd531756123948bb19e18249df7d51
  float dt;
  if (lastControlTime == 0) {
    dt = 0.02;
  } else {
    dt = (now - lastControlTime) / 1000.0;
  }
<<<<<<< HEAD

  lastControlTime = now;

  // Keep dt in a reasonable range
  dt = clampFloat(dt, 0.005, 0.08);

  // Get steering command
  float control = computePIControl(dt);

  // Turn steering command into left/right wheel speeds
  // Positive control -> stronger right wheel -> left correction
  int leftSpeed  = (int)(baseSpeed + control);
  int rightSpeed = (int)(baseSpeed - control);

=======
  lastControlTime = now;

  dt = clampFloat(dt, 0.005, 0.08);
  float control = computePIControl(dt);

  // make one wheel faster and the other one slower as needed
  int leftSpeed  = (int)(baseSpeed + control);
  int rightSpeed = (int)(baseSpeed - control);

  dt = clampFloat(dt, 0.005, 0.08);
>>>>>>> 66e489883efd531756123948bb19e18249df7d51
  leftSpeed  = clampInt(leftSpeed, 0, 255);
  rightSpeed = clampInt(rightSpeed, 0, 255);

  driveForwardDifferential(leftSpeed, rightSpeed);
}

<<<<<<< HEAD
// Hard left turn for a 90-degree left corner
=======

>>>>>>> 66e489883efd531756123948bb19e18249df7d51
void doTurnLeft() {
  moveLeft(TURN_SPEED);
}

<<<<<<< HEAD
// Hard right turn for when the wall disappears on the right
=======
>>>>>>> 66e489883efd531756123948bb19e18249df7d51
void doTurnRight() {
  moveRight(TURN_SPEED);
}

<<<<<<< HEAD
// Small left correction state
=======
>>>>>>> 66e489883efd531756123948bb19e18249df7d51
void doAdjustLeft() {
  doFollowWall(ADJUST_SPEED);
}

<<<<<<< HEAD
// Small right correction state
=======
>>>>>>> 66e489883efd531756123948bb19e18249df7d51
void doAdjustRight() {
  doFollowWall(ADJUST_SPEED);
}

<<<<<<< HEAD
// ============================================================
// 13. FSM TRANSITIONS
// ============================================================

// Decide when to switch states
=======

// when to switch states
>>>>>>> 66e489883efd531756123948bb19e18249df7d51
void updateFSM() {
  estimateWall();

  switch (state) {

    case FOLLOW_WALL:
<<<<<<< HEAD
      // If a wall appears in front, this is likely a left turn
      if (frontBlocked()) {
        setState(TURN_LEFT);
      }
      // If the wall disappears on the right, this is likely a right turn
      else if (rightWallLost()) {
        setState(TURN_RIGHT);
      }
      // If heading is a bit off, go into fine correction state
=======
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
>>>>>>> 66e489883efd531756123948bb19e18249df7d51
      else if (rightWallVisible()) {
        if (headingTheta > HEADING_ADJUST_ENTER) {
          setState(ADJUST_RIGHT);
        } else if (headingTheta < -HEADING_ADJUST_ENTER) {
          setState(ADJUST_LEFT);
        }
      }
      break;

    case ADJUST_LEFT:
<<<<<<< HEAD
      // Higher-priority transitions still override
=======
>>>>>>> 66e489883efd531756123948bb19e18249df7d51
      if (frontBlocked()) {
        setState(TURN_LEFT);
      } else if (rightWallLost()) {
        setState(TURN_RIGHT);
      } else if (fabs(headingTheta) < HEADING_ADJUST_EXIT) {
        setState(FOLLOW_WALL);
      }
      break;

    case ADJUST_RIGHT:
<<<<<<< HEAD
      // Higher-priority transitions still override
=======
>>>>>>> 66e489883efd531756123948bb19e18249df7d51
      if (frontBlocked()) {
        setState(TURN_LEFT);
      } else if (rightWallLost()) {
        setState(TURN_RIGHT);
      } else if (fabs(headingTheta) < HEADING_ADJUST_EXIT) {
        setState(FOLLOW_WALL);
      }
      break;

    case TURN_LEFT:
<<<<<<< HEAD
      // Leave left turn when enough time passed and front is clear
      // and the wall on the right is seen again
=======
      if (timeInState() < 10){
      moveLeft(255);
    }
>>>>>>> 66e489883efd531756123948bb19e18249df7d51
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
<<<<<<< HEAD
      // Leave right turn when the wall is found again
=======
>>>>>>> 66e489883efd531756123948bb19e18249df7d51
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

<<<<<<< HEAD
// ============================================================
// 14. RUN THE CURRENT STATE
// ============================================================

// Perform the action for the current state
=======
// what the states do
>>>>>>> 66e489883efd531756123948bb19e18249df7d51
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

<<<<<<< HEAD
// ============================================================
// 15. DEBUG PRINTS
// ============================================================

// Print useful values for tuning and debugging
void printDebug() {
  unsigned long now = millis();

  if (now - lastPrintTime < PRINT_INTERVAL_MS) {
    return;
  }

=======
// debugging / print statements --> troubleshooting
void printDebug() {
  unsigned long now = millis();
  if (now - lastPrintTime < PRINT_INTERVAL_MS) {
    return;
  }
>>>>>>> 66e489883efd531756123948bb19e18249df7d51
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

<<<<<<< HEAD
// ============================================================
// 16. SETUP
// ============================================================

void setup() {
  // Start serial monitor for debugging
  Serial.begin(115200);

  // Set front ultrasonic pins
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);

  // Set front-right ultrasonic pins
  pinMode(TRIG_FR, OUTPUT);
  pinMode(ECHO_FR, INPUT);

  // Set back-right ultrasonic pins
  pinMode(TRIG_BR, OUTPUT);
  pinMode(ECHO_BR, INPUT);

  // Set motor pins
=======
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

>>>>>>> 66e489883efd531756123948bb19e18249df7d51
  pinMode(A_1B, OUTPUT);
  pinMode(A_1A, OUTPUT);
  pinMode(B_1B, OUTPUT);
  pinMode(B_1A, OUTPUT);

<<<<<<< HEAD
  // Start with motors stopped
  stopMotors();

  // Small delay to settle everything
  delay(500);

  // Start in follow-wall mode
=======
  stopMotors();
  delay(500);

  // start by following the wall
>>>>>>> 66e489883efd531756123948bb19e18249df7d51
  state = FOLLOW_WALL;
  stateStartTime = millis();
  lastControlTime = millis();

  Serial.println("Wall follower started.");
}

<<<<<<< HEAD
// ============================================================
// 17. MAIN LOOP
// ============================================================

void loop() {
  // Update one sensor at a time
  serviceSensors();

  // Decide whether we need to change state
  updateFSM();

  // Run the current state's motor behavior
  runState();

=======
void loop() {
  // update one sensor at a time
  serviceSensors();
  // when to switch states
  updateFSM();
  // what state does
  runState();
>>>>>>> 66e489883efd531756123948bb19e18249df7d51
  // Print debug information
  printDebug();
}