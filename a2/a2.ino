//////////////////////////////////////////////////////////////////////////////////////////////////////////// prof's t4 example

// Pins for Ultrasonic
const int frontTrigPin = 11;
const int frontEchoPin = 12;
const int backTrigPin = 2;
const int backEchoPin = 3;

// Pins for Motors
const int A_1B = 5;
const int A_1A = 6;
const int B_1B = 9;
const int B_1A = 10;

// IR pin
const int irPin = 7;


// Timing for sensors
unsigned long currentMillis = 0;
unsigned long previousMillis = 0; // Stores the last time the sensor was read
const long interval = 60; // Defines the interval in which we want to trigger the sensor (60ms)

//Filtering for distance
const int front_window_size = 5; // Number of last measurement to use is moving average filter
float front_window[front_window_size]; // Array that stores the last ten measurements
int front_window_index = 0; // Keeps track of where the oldest data is in the array (will be replaced with the newest data)


const int back_window_size = 5; // Number of last measurement to use is moving average filter
float back_window[front_window_size]; // Array that stores the last ten measurements
int back_window_index = 0; // Keeps track of where the oldest data is in the array (will be replaced with the newest data)


// Distance thresholds
float distance_threshold = 0.05; // Trigger behaviour change when 5 cm within obstacle 

// Variables for detecting hand
const int ir_window_size = 5; // Number of last measurement to use is moving average filter
int ir_window[ir_window_size]; // Array that stores the last ten measurements
int ir_window_index = 0; // Keeps track of where the oldest data is in the array (will be replaced with the newest data)
int ir_sum = 0;

// Timing for states
unsigned long previousMillisStateChange = 0;
const long state_change_interval = 100; // state can change max once per second


// State transition logic
enum State {
  STATE_IDLE,
  STATE_FORWARD,
  STATE_BACKWARD
};

State cur_state = STATE_FORWARD;
State new_state = cur_state;

// Initialize distance with 100 m (to avoid instant state transition triggering)
float front_filtered_distance = 100;
float front_raw_distance = 100;

float back_filtered_distance = 100;
float back_raw_distance = 100;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);


  // Ultrasonic pin mode defition
  pinMode(frontEchoPin, INPUT);
  pinMode(frontTrigPin, OUTPUT);
  pinMode(backEchoPin, INPUT);
  pinMode(backTrigPin, OUTPUT);

  
  // IR pin mode definition
  pinMode(irPin, INPUT);

  
  // Motor pin mode definition
  pinMode(A_1B, OUTPUT);
  pinMode(A_1A, OUTPUT);
  pinMode(B_1B, OUTPUT);
  pinMode(B_1A, OUTPUT);


  // Init distances to large values to avoid instant state triggering
  for(unsigned int i = 0; i < front_window_size; i++)
  {
    front_window[i] = 100;
  }
  
  for(unsigned int i = 0; i < back_window_size; i++)
  {
    back_window[i] = 100;
  }

  // Init color
  for(unsigned int i = 0; i < ir_window_size; i++)
  {
    ir_window[i] = 0;
  }

  // wait 5 seconds before starting for safety
  delay(5000);

}

void loop() {

  // Grab the current time
  currentMillis = millis();

  // SENSING AND DATA PROCESSING
  if(currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis; // Save the last time we pinged (which is now)

    // FRONT DISTANCE
    front_raw_distance = readFrontSensorData();

    // Overwrite the oldest data in our array
    front_window[front_window_index] = front_raw_distance;
    
    // Advance index
    front_window_index = (front_window_index + 1)%front_window_size; // 0 --> 1 --> ... --> 9 --> 0

    // Calculate the average of the whole window

    float sum = 0;
    for(int i = 0; i < front_window_size; i++)
    {
      sum += front_window[i];
    }
    front_filtered_distance = sum / front_window_size;

    // BACK DISTANCE
    back_raw_distance = readBackSensorData();

    // Overwrite the oldest data in our array
    back_window[back_window_index] = back_raw_distance;
    
    // Advance index
    back_window_index = (back_window_index + 1)%back_window_size; // 0 --> 1 --> ... --> 9 --> 0

    // Calculate the average of the whole window

    sum = 0;
    for(int i = 0; i < back_window_size; i++)
    {
      sum += back_window[i];
    }
    back_filtered_distance = sum / back_window_size;

    


    // COLOR
    int hand_detected = !digitalRead(irPin);

    // Overwrite the oldest data in our array
    ir_window[ir_window_index] = hand_detected;
    
    // Advance index
    ir_window_index = (ir_window_index + 1)%ir_window_size;

    // Sum up the detections over the window
    // Helps us to see if we got a reliable reading (similar to filtering above)
    ir_sum = 0;
    for(int i = 0; i < ir_window_size; i++)
    {
      ir_sum += ir_window[i];
    }

    Serial.println("dist_front,dist_back,detect_hand");
    Serial.print(front_filtered_distance * 100);
    Serial.print(",");
    Serial.print(back_filtered_distance * 100);
    Serial.print(",");
    Serial.println(ir_sum);

    
  }

  // ROBOT BEHAVIOUR FSM
  new_state = cur_state;
  
  switch (cur_state) {

    case STATE_IDLE:
        
        // Behaviour
        stopMove();
        
        // Transitions and Trigger
        if(ir_sum == 0)
        {
          new_state = STATE_FORWARD;
        }
      break;

      
    case STATE_FORWARD:
      
      // Behaviour
      moveForward(150);
      
      // Transitions and Trigger
      if(front_filtered_distance <= distance_threshold)
      {
        new_state = STATE_BACKWARD;
      }
      if(ir_sum == ir_window_size)
      {
        new_state = STATE_IDLE;
      }
      break;

      
    case STATE_BACKWARD:
    
      // Behaviour
      moveBackward(150);
        
      // Transitions and Trigger
      if(back_filtered_distance <= distance_threshold)
      {
        new_state = STATE_FORWARD;
      }
      if(ir_sum == ir_window_size)
      {
        new_state = STATE_IDLE;
      }
      break;

    default:
      stopMove();
  }

  // Set new state
  switch_state(new_state);

}


void switch_state(State new_state) {
  // We can only switch states ever N ms and we will not switch, if the state remains the same
  if(currentMillis - previousMillisStateChange >= state_change_interval && new_state != cur_state)
  {
    cur_state = new_state;
    previousMillisStateChange = currentMillis;
  }
  
}


float readFrontSensorData() {
  // Sets Trigger Pin to HIGH for 10 us
  digitalWrite(frontTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(frontTrigPin, LOW);

  // Wait for return signal echo pin
  float time = pulseIn(frontEchoPin, HIGH, 30000) * 1e-6;
  float distance = time * 343 * 0.5;

  return distance;
}

float readBackSensorData() {
  // Sets Trigger Pin to HIGH for 10 us
  digitalWrite(backTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(backTrigPin, LOW);

  // Wait for return signal echo pin
  float time = pulseIn(backEchoPin, HIGH, 30000) * 1e-6;
  float distance = time * 343 * 0.5;

  return distance;
}

void moveForward(int speed) {
  analogWrite(A_1B, 0);
  analogWrite(A_1A, speed);
  analogWrite(B_1B, speed);
  analogWrite(B_1A, 0);
}

void moveBackward(int speed) {
  analogWrite(A_1B, speed);
  analogWrite(A_1A, 0);
  analogWrite(B_1B, 0);
  analogWrite(B_1A, speed);
}

void turnRight(int speed) {
  analogWrite(A_1B, speed);
  analogWrite(A_1A, 0);
  analogWrite(B_1B, speed);
  analogWrite(B_1A, 0);
}

void turnLeft(int speed) {
  analogWrite(A_1B, 0);
  analogWrite(A_1A, speed);
  analogWrite(B_1B, 0);
  analogWrite(B_1A, speed);
}

void stopMove() {
  analogWrite(A_1A, 0);
  analogWrite(A_1B, 0);
  analogWrite(B_1A, 0);
  analogWrite(B_1B, 0);
}
