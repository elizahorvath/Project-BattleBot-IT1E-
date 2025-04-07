// Motor, servo and sonar pins
#define SERVO_PIN            4
#define MOTOR_A_IN1_PIN     10    // LEFT motor - FORWARD
#define MOTOR_B_IN1_PIN      6    // RIGHT motor - FORWARD
#define MOTOR_A_IN2_PIN      3    // LEFT motor - BACKWARDS
#define MOTOR_B_IN2_PIN      5    // RIGHT motor - BACKWARDS

#define TRIGGER_PIN         12    // Ultrasonic sensor TRIGGER pin
#define ECHO_PIN            13    // Ultrasonic sensor ECHO pin

#define GRIPPER_OPEN      1800    // Pulse length for open gripper
#define GRIPPER_CLOSE     1100    // Pulse length for closed gripper

// Sensor array for line following
int sensors[] = {A0, A1, A2, A3, A4, A5, A6, A7};

// PD control parameters
const float _Kp = 0.7;
const float _Kd = 0.6;

const int _baseSpeed = 250; 

// State variables
int lastError = 0;
unsigned long timer = 0;
int state = 0;
bool gripperClosed = false;
bool blackCrossedOnce = false;
int blackCrossCount = 0;
unsigned long blackStartTime = 0;
bool movingBackward = false;
unsigned long backwardStartTime = 0;
bool stopRobot = false;

void setup() {
  pinMode(SERVO_PIN, OUTPUT);

  pinMode(MOTOR_A_IN1_PIN, OUTPUT);
  pinMode(MOTOR_B_IN1_PIN, OUTPUT);

  // Giving the motors 0 power initially;
  analogWrite(MOTOR_A_IN1_PIN, 0);
  analogWrite(MOTOR_B_IN1_PIN, 0);
  analogWrite(MOTOR_A_IN2_PIN, 0);
  analogWrite(MOTOR_B_IN2_PIN, 0);

  // Ultrasonic sensors roles
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  digitalWrite(SERVO_PIN, LOW);
  stopMotors();
}

void loop() {
  if (stopRobot) {
    return;  // If the robot has completed the track once, stop
  }

  long distance = getDistance();  
  unsigned long now = millis();

  // Start the robot when the flag is lifted
  if (distance > 20 && state == 0) {
    timer = now;
    state = 1;
  }
  
  // Move forward for 1200 millisecond
  if (state == 1) {
    if (now - timer < 1200) {
      moveForward(230, 255);
    } else {
      stopMotors();
      timer = now;
      state = 2;
    }
  }
  
  // Close gripper
  if (state == 2) {
    if (now - timer < 500) {
      gripper(GRIPPER_CLOSE);
    } else {
      timer = now;
      state = 3;
    }
  }

  // Turn left for 800ms
  if (state == 3) {
    if (now - timer < 800) {
      moveForward(0, 255);
    } else {
      stopMotors();
      timer = now;
      state = 4;
    }
  }
  
  // Small delay before line-following
  if (state == 4) {
    if (now - timer < 200) {
      moveForward(240, 255);
    } else {
      timer = now;
      state = 5;
    }
  }

  // Line following and object avoidance
  if (state >= 5) {
    if (allSensorsBlack()) {
      if (blackStartTime == 0) {
        blackStartTime = now;
      } else if (now - blackStartTime >= 170) {
        blackCrossCount++;
        if (blackCrossCount == 2 && !movingBackward) {
          gripper(GRIPPER_OPEN);
          moveBackward(230, 240);
          backwardStartTime = now;
          movingBackward = true;
        }
      }
    } else {
      blackStartTime = 0;
      if (movingBackward) {
        if (now - backwardStartTime >= 1000) {
          stopMotors();
          movingBackward = false;
          stopRobot = true;
        } else {
          moveBackward(230, 240);
        }
      } else {
        // Object avoidance logic
        if (distance < 17) {
          avoidObject();
        } else {
          followLine(5, 2, 240, 240);
        }
        gripper(GRIPPER_CLOSE);
      }
    }
  }
}

void avoidObject() {
  // Turn left
  int counter = millis() + 900;
  while (millis() < counter) {
    moveForward(0, 200);
    gripper(GRIPPER_CLOSE);
  }

  // Move forward
  counter = millis() + 1300;
  while (millis() < counter) {
    moveForward(175, 200);
    gripper(GRIPPER_CLOSE);
  }

  // Turn right
  counter = millis() + 400;
  while (millis() < counter) {
    moveForward(200, 0);
    gripper(GRIPPER_CLOSE);
  }

  // Move forward
  counter = millis() + 200;
  while (millis() < counter) {
    moveForward(165, 200);
    gripper(GRIPPER_CLOSE);
  }
}

void moveForward(int left, int right) {
  analogWrite(MOTOR_A_IN1_PIN, left);
  analogWrite(MOTOR_B_IN1_PIN, right);
}

void moveBackward(int left, int right) {
  analogWrite(MOTOR_A_IN1_PIN, 0);
  analogWrite(MOTOR_B_IN1_PIN, 0);
  analogWrite(MOTOR_A_IN2_PIN, left);
  analogWrite(MOTOR_B_IN2_PIN, right);
}

void stopMotors() {
  analogWrite(MOTOR_A_IN1_PIN, 0);
  analogWrite(MOTOR_B_IN1_PIN, 0);
  analogWrite(MOTOR_A_IN2_PIN, 0);
  analogWrite(MOTOR_B_IN2_PIN, 0);
}

void gripper(int pulse) {
  digitalWrite(SERVO_PIN, HIGH);
  delayMicroseconds(pulse);
  digitalWrite(SERVO_PIN, LOW);
}

void followLine(int leftSensor, int rightSensor, int leftSpeed, int rightSpeed) {
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate >= 5) { // Updating every 5ms
    // Calculating the difference between the left and the right sensor
    int error = analogRead(sensors[leftSensor]) - analogRead(sensors[rightSensor]);

    int derivative = error - lastError;
    lastError = error;
    
    int correction = (_Kp * error) + (_Kd * derivative);

    // Calculating how much the wheels have to speed up / slow down
    int lSpeed = constrain(_baseSpeed - correction, 0, leftSpeed);
    int rSpeed = constrain(_baseSpeed + correction, 0, rightSpeed);
    
    // Setting the speed of the wheels based on the corrections
    analogWrite(MOTOR_A_IN1_PIN, lSpeed);
    analogWrite(MOTOR_B_IN1_PIN, rSpeed);
    
    lastUpdate = millis();
  }
}

bool allSensorsBlack() {
  int blackCount = 0;
  for (int i = 0; i < 8; i++) {
    if (analogRead(sensors[i]) > 600) {
      blackCount++;
    }
  }

  return (blackCount >= 8);
}

long getDistance() {
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  return pulseIn(ECHO_PIN, HIGH) * 0.034 / 2;
}