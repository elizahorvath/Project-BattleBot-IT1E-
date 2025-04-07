#include <Adafruit_NeoPixel.h>
#define PIN 4          // Pin connected to the NeoPixels
constexpr int NUMPIXELS = 4;   // Number of NeoPixels

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
bool isOn = false;
int neoPreviousTime = 0;
int interval;

enum RobotState { 
  WAITING_FOR_CONE,
  CALIBRATE, 
  PICK_UP, 
  FOLLOW_LINE, 
  CHECK_PATH, 
  TURN_LEFT, 
  TURN_RIGHT, 
  TURN_AROUND,
  AVOID_OBJECT, 
  GAME_OVER,
  MOVE_OUT_OF_SQUARE,
  DONE
};

enum Direction { 
  NONE, 
  LEFT_TURN, 
  RIGHT_TURN, 
  T_JUNCTION 
};

struct PathCheck {
  bool active;
  int tickThreshold;
};

struct TurnState {
  bool turning = false;
  int targetPulses = 0;
  unsigned long lastCheck = 0;
};

RobotState currentState = WAITING_FOR_CONE;
Direction storedDirection = NONE;
TurnState turnState;  // Global turn state
PathCheck pathCheck = { false, 2 };

//Line Sensors
#define NUM_SENSORS 8  // Number of sensors
int sensorPins[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5, A6, A7};  // Sensor pin mapping
int sensorValues[NUM_SENSORS];  // Array to store sensor readings
int sensorMin[NUM_SENSORS];
int sensorMax[NUM_SENSORS];
int sensorThreshold[NUM_SENSORS]; // Array to store thresholds for each sensor

// Ultrasonic Sensor Pins
const int trigPin = 13;
const int echoPin = 12;
const unsigned long requiredTime = 200;  // Time in ms the object must be detected
unsigned long detectedStart = 0;

bool conePickedUp = false;
bool junctionDetected;

//Measurements
const float WHEEL_CIRCUMFERENCE = 20.4;
const int PULSE_PER_REVOLUTION = 20;
const float DISTANCE_BETWEEN_WHEELS = 22.75;
static const int DISTANCE_FROM_BASE_TO_CONE = 27; // Distance is in ticks
const int target = DISTANCE_FROM_BASE_TO_CONE;

// Encoder Pulse Counters
volatile signed int _leftTicks = 0;
volatile signed int _rightTicks = 0;

// Motor Pins
#define MOTOR_A_1 11  // Left Forward
#define MOTOR_A_2 10  // Left Reverse
#define MOTOR_B_1 5 // Right Forward
#define MOTOR_B_2 6 // Right Reverse
int baseSpeed = 255;
int _leftSpeed;
int _rightSpeed;

//Servo Control
#define GRIPPER_OPEN 1750
#define GRIPPER_CLOSE 1020
#define SERVO 9
const int pulse = 2000;
int previousTime = 0;
const int gripperInterval = 20;

//Rotation Sensors
#define MOTOR_R1 2
#define MOTOR_R2 3

#define ISR_INTERVAL  20 // interval of 20 milli seconds to update counter by interupt

void leftEncoderISR() {
  static unsigned long timer;
  if (millis() > timer) {
     _leftTicks++;
     timer = millis() + ISR_INTERVAL;
  }
}

void rightEncoderISR() {
  static unsigned long timer;
  if (millis() > timer) {
    _rightTicks++;
    timer = millis() + ISR_INTERVAL;
  }
}

//PID
float Kp, Ki, Kd, integral = 0, derivative = 0, turnDistance = 0;
int correction, error = 0, lastError = 0, pulses, angle, raduis = DISTANCE_BETWEEN_WHEELS, turn_Circumference = 2 * 3.14 * raduis;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  // Set Motor Pins
  pinMode(MOTOR_A_1, OUTPUT);
  pinMode(MOTOR_A_2, OUTPUT);
  pinMode(MOTOR_B_1, OUTPUT);
  pinMode(MOTOR_B_2, OUTPUT);
  digitalWrite(MOTOR_A_1, LOW);
  digitalWrite(MOTOR_A_2, LOW);
  digitalWrite(MOTOR_B_1, LOW);
  digitalWrite(MOTOR_B_2, LOW); 

  // Attach Interrupts for Encoders
  attachInterrupt(digitalPinToInterrupt(MOTOR_R1), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_R2), rightEncoderISR, CHANGE);

  //SERVO
  pinMode(SERVO, OUTPUT);
  digitalWrite(SERVO, LOW);

  // Set Ultrasonic Sensor Pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  //NEOPIXELS
  pixels.begin();  // Initialize NeoPixel library
  pixels.clear();  // Ensure all pixels are off
  pixels.show();
}

void loop() {
  gripperFunction();

  //readSensors();

  solveMaze();

  //readSensors();
}

bool isLeftTurn() {
  return sensorValues[4] > sensorThreshold[4] &&
         sensorValues[5] > sensorThreshold[5] &&
         sensorValues[6] > sensorThreshold[6] &&
         sensorValues[7] > sensorThreshold[7] &&
         sensorValues[0] < sensorThreshold[0] &&
         sensorValues[1] < sensorThreshold[1];
}

bool isTJunction() {
  return sensorValues[0] > sensorThreshold[0] &&
         sensorValues[1] > sensorThreshold[1] &&
         sensorValues[2] > sensorThreshold[2] &&
         sensorValues[3] > sensorThreshold[3] &&
         sensorValues[4] > sensorThreshold[4] &&
         sensorValues[5] > sensorThreshold[5] &&
         sensorValues[6] > sensorThreshold[6] &&
         sensorValues[7] > sensorThreshold[7];
}

bool isRightTurn() {
  return sensorValues[6] < sensorThreshold[6] &&
         sensorValues[7] < sensorThreshold[7] &&
         sensorValues[0] > sensorThreshold[0] &&
         sensorValues[1] > sensorThreshold[1] &&
         sensorValues[2] > sensorThreshold[2] &&
         sensorValues[3] > sensorThreshold[3];
}

bool isDeadEnd() {
  return sensorValues[0] < sensorThreshold[0] &&
         sensorValues[1] < sensorThreshold[1] &&
         sensorValues[2] < sensorThreshold[2] &&
         sensorValues[3] < sensorThreshold[3] &&
         sensorValues[4] < sensorThreshold[4] &&
         sensorValues[5] < sensorThreshold[5] &&
         sensorValues[6] < sensorThreshold[6] &&
         sensorValues[7] < sensorThreshold[7];
}


void solveMaze() {
  switch(currentState) {
    case WAITING_FOR_CONE:
      waitingForCone();
      waitingForConeLight();
      break;
    case CALIBRATE:
      calibrateSensors();
      calibrationLight();
      break;
    case PICK_UP:
      pickUpCone();
      break;
    case FOLLOW_LINE:
      followLine();
      break;
    case CHECK_PATH:
      checkPathAhead();
      break;
    case TURN_LEFT:
      turn(55, true);
      turningLeft();
      break;
    case TURN_RIGHT:
      turn(55, false);
      turningRight();
      break;
    case TURN_AROUND:
      turnAround(false);
      turningRight();
      break;
    case GAME_OVER:
      stopMotors();
      resetTicks();
      gameEndedLight();
      conePickedUp = false;
      currentState = MOVE_OUT_OF_SQUARE;
      break;
    case MOVE_OUT_OF_SQUARE:
      moveOutOfSquare();
      gameEndedLight();
      break;
    case AVOID_OBJECT:
      turnAround(true);
      turningRight();
      break;
    case DONE:
      stopMotors();
      gameEndedLight();
      break;
  }  
}

void toggleLights(int newInterval, int g, int r, int b, bool useUpdateLights = true, int pixel1 = -1, int pixel2 = -1) {
  interval = newInterval;
  unsigned int currentTime = millis();
  
  if (currentTime - neoPreviousTime >= interval) {
    neoPreviousTime = currentTime;    
    if (isOn) {
      pixels.clear(); // Turn off all LEDs
    } else {
      if (useUpdateLights) {
        updateLights(g, r, b);
      } else {
        if (pixel1 >= 0) pixels.setPixelColor(pixel1, pixels.Color(g, r, b));
        if (pixel2 >= 0) pixels.setPixelColor(pixel2, pixels.Color(g, r, b));
      }
    }
    pixels.show(); // Update NeoPixels
    isOn = !isOn;  // Toggle state
  }
}

void updateLights(int g, int r, int b) {
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(g, r, b)); // Red color
  }
}
void turningRight() { toggleLights(150, 165, 255, 0, false, 2, 1); }
void turningLeft() { toggleLights(150, 165, 255, 0, false, 3, 0); }

void lineFollowLight() {
  interval = 5;
  unsigned int currentTime = millis();
  
  if (currentTime - neoPreviousTime >= interval) {
    neoPreviousTime = currentTime;
    pixels.setPixelColor(2, pixels.Color(255, 255, 255)); 
    pixels.setPixelColor(3, pixels.Color(255, 255, 255)); 
    pixels.setPixelColor(0, pixels.Color(0, 255, 0)); 
    pixels.setPixelColor(1, pixels.Color(0, 255, 0)); 
    pixels.show();
  }
}
void waitingForConeLight() { toggleLights(150, 165, 255, 0); }
void gameEndedLight() { toggleLights(1000, 0, 255, 0); }
void calibrationLight() { toggleLights(150, 0, 0, 255); }

void waitingForCone(){

  if (objectDetected(35)) {
    delay(1500);
    currentState = CALIBRATE;
  }
  waitingForConeLight();
}

void moveOutOfSquare() {
  if (_leftTicks > target || _rightTicks > target) {
    stopMotors();
    currentState = DONE;
  }

  moveForwardPID(baseSpeed, baseSpeed, true, false);
}

void pickUpCone() {
  conePickedUp = true;
  currentState = TURN_LEFT;
}

void calibrateSensors() {
  static bool firstRun = true;
  readSensors();

  // Initialize sensor min/max values on the first run
  if (firstRun) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      sensorMin[i] = 1023;  // Set min to max ADC value
      sensorMax[i] = 0;     // Set max to min ADC value
    }
    firstRun = false;
  }

  // Check if target distance is reached
  if (_leftTicks > target || _rightTicks > target) {
    stopMotors();
    //sensorsCalibrated = true;
    currentState = PICK_UP;

    // Calculate and store threshold values
    for (int i = 0; i < NUM_SENSORS; i++) {
      sensorThreshold[i] = (sensorMin[i] + sensorMax[i]) / 2;
      Serial.print("Threshold [");
      Serial.print(i);
      Serial.print("]: ");
      Serial.println(sensorThreshold[i]);
      Serial.println(sensorMin[i]);
      Serial.println(sensorMax[i]);
    }
    return;
  }

  // Read sensor values and update min/max
  for (int i = 0; i < NUM_SENSORS; i++) {
    int sensorValue = analogRead(sensorPins[i]);

    if (sensorValue < sensorMin[i]) {
        sensorMin[i] = sensorValue;
    }
    if (sensorValue > sensorMax[i]) {
        sensorMax[i] = sensorValue;
    }
  }

  moveForwardPID(255, 255, true, false);
}

void moveForwardPID(int _leftSpeed, int _rightSpeed, bool withOutLine, bool lineTracking) {
  if (withOutLine) {
    // PID Variables
    Kp = 6.5;  // Proportional Gain
    Ki = 0.1;  // Integral Gain
    Kd = 7;  // Derivative Gain

    // Calculate error
    error = _leftTicks - _rightTicks;

    // Update PID terms
    integral += error;
    derivative = error - lastError;
    lastError = error;

    // Prevent integral windup
    integral = constrain(integral, -10, 10);

  } else if (lineTracking) {
    readSensors();
    int position = calculateLinePosition();

    // PID Variables
    Kp = 0.65;  // Proportional Gain
    Ki = 0.00004;  // Integral Gain
    Kd = 2.0;  // Derivative Gain

    int center = (NUM_SENSORS - 1) * 1000 / 2;  // Midpoint of sensor array
    // Calculate error
    error = position - center;

    // Update PID terms
    integral += error;
    derivative = error - lastError;
    lastError = error;   
  }

  // Apply PID correction
  correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
  _leftSpeed -= correction;
  _rightSpeed += correction;

  // Ensure PWM values are within valid range (0 - 255)
  _leftSpeed = constrain(_leftSpeed, 0, 255);
  _rightSpeed = constrain(_rightSpeed, 0, 255);

  if (currentState == FOLLOW_LINE || currentState == CALIBRATE || currentState == CHECK_PATH)
  {
    moveForward(_leftSpeed, _rightSpeed);
  } 
  else if (currentState == TURN_AROUND || currentState == AVOID_OBJECT)
  {
    moveForward(_leftSpeed, -_rightSpeed);
  } 
  else if (currentState == MOVE_OUT_OF_SQUARE) 
  {
    moveForward(-_leftSpeed, -_rightSpeed);
  }
}

void startPathCheck() {

  resetTicks();
  
  pathCheck.active = true;

}

void checkPathAhead() {
  // This function is called repeatedly when currentState == CHECK_PATH
    moveForwardPID(250, 250, true, false);
  if (pathCheck.active) {
    // Wait until both wheels have moved forward enough
    if (_leftTicks >= pathCheck.tickThreshold && _rightTicks >= pathCheck.tickThreshold) {
      // Stop the robot
      stopMotors();
      // Update sensor readings
      readSensors();
      
      // Evaluate sensor data to decide if a valid path exists
      bool pathExists = lineDetected();
      bool endOfMaze = isTJunction();
      if (storedDirection == LEFT_TURN || storedDirection == T_JUNCTION) {
        pathExists = false;
      }

      if (pathExists) {
        // The intended path is confirmed—resume following the line
        currentState = FOLLOW_LINE;
        lineFollowLight();
        return;
      } else if(endOfMaze) {
        stopMotors();
        currentState = GAME_OVER;
      }else {
        // No valid path detected, so use the stored direction decision:
        switch (storedDirection) {
          case LEFT_TURN:
            currentState = TURN_LEFT;
            break;
          case RIGHT_TURN:
            currentState = TURN_RIGHT;
            break;
          case T_JUNCTION:
            currentState = TURN_LEFT; // Example fallback
            break;
          default:
            // If no direction was stored, perhaps perform a safe action (like stopping or turning around)
            currentState = TURN_AROUND;
            break;
        }
      }
      
      // Deactivate path checking once done
      pathCheck.active = false;
    }
  }
}

void turn(int angle, bool isLeftTurn) {
  if (!turnState.turning) {
    resetTicks();
    turnDistance = (angle / 360.0) * turn_Circumference;  
    turnState.targetPulses = (turnDistance / WHEEL_CIRCUMFERENCE) * PULSE_PER_REVOLUTION;
    
    if (isLeftTurn) {
      moveForward(0, 200);  // Left wheel stops, right wheel moves forward
      currentState = TURN_LEFT;
      storedDirection = NONE;
    } else {
      moveForward(200, 0);  // Right wheel stops, left wheel moves forward
      currentState = TURN_RIGHT;
      storedDirection = NONE;
    }

    turnState.turning = true;
  }

  if (turnState.turning) {
    
    if (((isLeftTurn && _rightTicks >= turnState.targetPulses) || (!isLeftTurn && _leftTicks >= turnState.targetPulses)) && lineDetected()) {
      stopMotors();
      currentState = FOLLOW_LINE;
      turnState.turning = false;
      lineFollowLight();
    }
  }
}

void turnAround(bool avoidObstacle) {
  if (!turnState.turning) {
    resetTicks();
    turnState.turning = true;
    currentState = AVOID_OBJECT;

    if (avoidObstacle) { // Obstacle turn: uses encoder-based turning
      turnDistance = (3.14 * (DISTANCE_BETWEEN_WHEELS / 2)); // Half the turning circumference
      turnState.targetPulses = ((turnDistance / WHEEL_CIRCUMFERENCE) * PULSE_PER_REVOLUTION) / 3;
      Serial.print("Target Pulses Around: "); Serial.println(turnState.targetPulses);
    }
  }

  if (turnState.turning && millis() - turnState.lastCheck >= 5) {
    turnState.lastCheck = millis();
    moveForwardPID(200, 200, true, false); // Start the turn
    //readSensors();

    // Check if turn is complete
    if (lineDetected() && (!avoidObstacle || _rightTicks >= turnState.targetPulses)) {
      stopMotors();     
      turnState.turning = false;
      currentState = FOLLOW_LINE;
      Serial.println("Turn Around Complete");
      lineFollowLight();
    }
  }
}

bool objectDetected(int threshold) {
  // Trigger the ultrasonic sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the echo time (with timeout)
  long duration = pulseIn(echoPin, HIGH, 30000);
  long distance = duration * 0.034 / 2;

  // Check if an object is within the threshold distance
  if (distance > 0 && distance < threshold) {
    if (detectedStart == 0) {
      detectedStart = millis();  // Start timing on first detection
    }
    // If the object has been present for the required time, return true
    if (millis() - detectedStart >= requiredTime) {
      return true;
    }
  } else {
    // Reset the timer if the object is not detected
    detectedStart = 0;
  }
  return false;
}

int calculateLinePosition() {
  long weightedSum = 0;
  long sum = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    int value = sensorValues[i];

    // Determine if the sensor is on the line
    weightedSum += (long)value * i * 1000;
    sum += value;
  }

  return weightedSum / sum;  // No line detected
}

void resetTicks() {
  noInterrupts();
  _leftTicks = 0;
  _rightTicks = 0;
  interrupts();
}

void readSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
  }
}

void followLine() {
  moveForwardPID(baseSpeed, baseSpeed, false, true);
  getLinePosition();

  if (junctionDetected) {
    currentState = CHECK_PATH;
    startPathCheck();
    junctionDetected = false;
   } else if (objectDetected(19)) {
      currentState = AVOID_OBJECT;
  }
  
}

void stopMotors() {
  analogWrite(MOTOR_A_1, 0);
  analogWrite(MOTOR_A_2, 0);
  analogWrite(MOTOR_B_1, 0);
  analogWrite(MOTOR_B_2, 0);
}

void moveForward(int _leftSpeed, int _rightSpeed) {
  analogWrite(MOTOR_A_1, _leftSpeed > 0 ? _leftSpeed : 0);
  analogWrite(MOTOR_A_2, _leftSpeed < 0 ? -_leftSpeed : 0);
  analogWrite(MOTOR_B_1, _rightSpeed > 0 ? _rightSpeed : 0);
  analogWrite(MOTOR_B_2, _rightSpeed < 0 ? -_rightSpeed : 0);
}

void gripper(int pulse) {
  static unsigned long timer;
  static int lastPulse;
  if (millis() > timer) {
    if (pulse > 0) {
      lastPulse = pulse;
    } else {
      pulse = lastPulse;
    }

    digitalWrite(SERVO, HIGH);
    delayMicroseconds(pulse);
    digitalWrite(SERVO, LOW);
    timer = millis() + 20;
  }
}

void getLinePosition() {
  //readSensors();  // Ensure sensorValues are updated

  if (isTJunction()) {
    storedDirection = T_JUNCTION;
    junctionDetected = true;
    stopMotors();
    Serial.println("T-Junction detected");
    debugSensors();
  } else if (isLeftTurn()) {
    storedDirection = LEFT_TURN;
    junctionDetected = true;
    stopMotors();
    Serial.println("Left turn detected");
    // Optionally verify position if needed
    verifyPosition();
    debugSensors();
  } else if (isRightTurn()) {
    storedDirection = RIGHT_TURN;
    junctionDetected = true;
    stopMotors();
    Serial.println("Right turn detected");
    debugSensors();
  } else if (isDeadEnd()) {
    storedDirection = NONE;
    currentState = TURN_AROUND;
    junctionDetected = false;
    stopMotors();
    Serial.println("Dead end detected");
  } else {
    storedDirection = NONE;
  }
}

void verifyPosition() {
  readSensors();
  if (isTJunction()) {
    storedDirection = T_JUNCTION;
  }
}

bool lineDetected() {
  readSensors();
  int blackCount = 0;
  for (int i = 0; i < 8; i++) {  
    if (sensorValues[i] > sensorThreshold[0]) {  // Count black detections
      blackCount++;
    }
  }
  return (blackCount > 0 && blackCount <= 3);  
}

void gripperFunction() {
  unsigned int currentTime = millis();
  if(conePickedUp) {
    if (currentTime - previousTime >= gripperInterval) {
      previousTime = currentTime;
      gripper(GRIPPER_CLOSE);
    }
  } else {
      if (currentTime - previousTime >= gripperInterval) {
        previousTime = currentTime;
        gripper(GRIPPER_OPEN);
      }
  }
}

void changeState(RobotState newState) {
  Serial.print("Changing state from ");
  Serial.print(currentState);
  Serial.print(" to ");
  Serial.println(newState);
  currentState = newState;
}

void debugSensors() {
  Serial.print("Sensors: ");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(sensorValues[i]);
    Serial.print(" ");
  }
  Serial.println();

  Serial.print("isTJunction: ");
  Serial.println(isTJunction() ? "true" : "false");
  Serial.print("isLeftTurn: ");
  Serial.println(isLeftTurn() ? "true" : "false");
  Serial.print("isRightTurn: ");
  Serial.println(isRightTurn() ? "true" : "false");
  Serial.print("isDeadEnd: ");
  Serial.println(isDeadEnd() ? "true" : "false");
}

